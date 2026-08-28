// Ebara Hydrostation Gateway — NimBLE implementation.
//
// Owns the entire NimBLE BLE host stack and speaks directly to the pump's
// BM70/BM71 Bluetooth module over its Transparent UART GATT service: pairs
// and bonds with it, discovers its GATT database, subscribes to
// notifications, and runs a sequential GET/SET command queue against it.

#include "ebara_hydrostation.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "store/config/ble_store_config.h"
#include "nvs_flash.h"
// Not declared in store/config/ble_store_config.h (only read/write/delete
// are), but present and linkable in ble_store_config.c, which initializes
// NVS-backed bond storage when CONFIG_BT_NIMBLE_NVS_PERSIST=y.
void ble_store_config_init(void);
}

namespace esphome {
namespace ebara_hydrostation {

static const char *const TAG = "ebara_hydrostation";

// Coherent bounds for the user-configurable poll update_interval: below 5s
// there is no real benefit since a full poll cycle (8 sequential commands,
// or a few more during the one-time static-data fetch after a fresh pairing)
// already takes several seconds; above 300s (5 min) sensor data would be too
// stale to be useful for a pressure-monitoring pump.
static constexpr float kUpdateIntervalMinS = 5.0f;
static constexpr float kUpdateIntervalMaxS = 300.0f;
static constexpr float kUpdateIntervalDefaultS = 15.0f;

// ── BM71 GATT constants ──────────────────────────────────────────────────────
// UUID bytes are stored little-endian (least-significant byte first), per
// NimBLE/Bluetooth convention — i.e. reversed from the human-readable string.
//   Service:   49535343-fe7d-4ae5-8fa9-9fafd205e455
//   TX/notify: 49535343-1e4d-4bd9-ba61-23c647249616  (handle 0x0052, CCCD 0x0053)
//   RX/write:  49535343-8841-43f4-a8d4-ecbe34729bb3  (handle 0x0055)
static const ble_uuid128_t kServiceUuid = BLE_UUID128_INIT(
    0x55, 0xe4, 0x05, 0xd2, 0xaf, 0x9f, 0xa9, 0x8f, 0xe5, 0x4a, 0x7d, 0xfe, 0x43, 0x53, 0x53, 0x49);
static const ble_uuid128_t kTxNotifyUuid = BLE_UUID128_INIT(
    0x16, 0x96, 0x24, 0x47, 0xc6, 0x23, 0x61, 0xba, 0xd9, 0x4b, 0x4d, 0x1e, 0x43, 0x53, 0x53, 0x49);
static const ble_uuid128_t kRxWriteUuid = BLE_UUID128_INIT(
    0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8, 0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49);

// Standard Client Characteristic Configuration Descriptor (CCCD) UUID.
// Predefined as a named object (rather than using BLE_UUID16_DECLARE() inline)
// because that macro takes the address of a C99 compound literal, which is
// not an lvalue in C++ and fails to compile under -fpermissive-off.
static const ble_uuid16_t kCccdUuid = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);

static EbaraHydrostationGateway *global_ebara_instance = nullptr;

namespace {

// Parses "AA:BB:CC:DD:EE:FF" into a PUBLIC ble_addr_t. The BM71 uses a public
// (non-random) address, so address_type PUBLIC is mandatory for a successful
// connection. The string is scanned into a reversed-index array, then
// copied linearly into ble_addr_t.val[] (NimBLE stores address bytes
// little-endian).
bool parse_mac_address(const std::string &s, ble_addr_t *out) {
  unsigned int v[6];
  int n = std::sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &v[5], &v[4], &v[3], &v[2], &v[1], &v[0]);
  if (n != 6)
    return false;
  out->type = BLE_ADDR_PUBLIC;
  for (int i = 0; i < 6; i++)
    out->val[i] = static_cast<uint8_t>(v[i]);
  return true;
}

// Parses a "Dato: N[,N...]" response, finding the "Dato:" prefix, stripping
// trailing ';'/whitespace, and returning the first token as unsigned 32-bit.
// Used for fields whose range spans the full uint32_t (e.g. the serial
// number), where parse_dato()'s signed int32_t/strtol parsing below would
// saturate instead of returning the true value.
uint32_t parse_dato_uint32(const std::string &raw) {
  size_t idx = raw.find("Dato:");
  if (idx == std::string::npos)
    return 0;
  std::string rest = raw.substr(idx + 5);
  size_t start = rest.find_first_not_of(' ');
  if (start == std::string::npos)
    return 0;
  rest = rest.substr(start);
  while (!rest.empty()) {
    char c = rest.back();
    if (c == ';' || c == ' ' || c == '\r' || c == '\n' || c == '\0')
      rest.pop_back();
    else
      break;
  }
  size_t comma = rest.find(',');
  std::string tok = (comma == std::string::npos) ? rest : rest.substr(0, comma);
  char *endptr = nullptr;
  unsigned long val = std::strtoul(tok.c_str(), &endptr, 10);
  return (endptr != tok.c_str()) ? static_cast<uint32_t>(val) : 0;
}

std::vector<int32_t> parse_dato(const std::string &raw) {
  std::vector<int32_t> out;
  size_t idx = raw.find("Dato:");
  if (idx == std::string::npos)
    return out;
  std::string rest = raw.substr(idx + 5);
  size_t start = rest.find_first_not_of(' ');
  if (start == std::string::npos)
    return out;
  rest = rest.substr(start);
  while (!rest.empty()) {
    char c = rest.back();
    if (c == ';' || c == ' ' || c == '\r' || c == '\n' || c == '\0')
      rest.pop_back();
    else
      break;
  }
  size_t pos = 0;
  while (pos <= rest.size()) {
    size_t comma = rest.find(',', pos);
    std::string tok = (comma == std::string::npos) ? rest.substr(pos) : rest.substr(pos, comma - pos);
    if (!tok.empty()) {
      // Exception-free integer parse (ESP-IDF builds disable C++ exceptions
      // by default, so std::stoi cannot be used here).
      char *endptr = nullptr;
      long val = std::strtol(tok.c_str(), &endptr, 10);
      if (endptr != tok.c_str())
        out.push_back(static_cast<int32_t>(val));
    }
    if (comma == std::string::npos)
      break;
    pos = comma + 1;
  }
  return out;
}

// Decodes the gm-0011 error-word bitmask into a human-readable list.
std::string decode_errors(int32_t val) {
  struct Bit {
    uint16_t mask;
    const char *desc;
  };
  static const Bit kBits[] = {
      {0x0001, "E01 Over-temperature"},        {0x0002, "E02 Voltage low"},
      {0x0004, "E02 Voltage high"},             {0x0008, "E03 Over-current"},
      {0x0020, "E04 Short circuit"},            {0x0100, "E06 Pressure sensor fault"},
      {0x0400, "H01 Dry run"},                  {0x0800, "H01 Dry run (2)"},
      {0x1000, "H02 Hourly restarts exceeded"}, {0x2000, "C01 Communication fault"},
  };
  std::string out;
  for (const auto &b : kBits) {
    if (val & b.mask) {
      if (!out.empty())
        out += ", ";
      out += b.desc;
    }
  }
  return out.empty() ? "No errors" : out;
}

// ── NimBLE C-linkage trampolines (the stack requires plain function pointers) ──
void reset_cb_trampoline(int reason) {
  if (global_ebara_instance != nullptr)
    global_ebara_instance->handle_host_reset(reason);
}
void sync_cb_trampoline() {
  if (global_ebara_instance != nullptr)
    global_ebara_instance->handle_host_sync();
}
int gap_event_trampoline(struct ble_gap_event *event, void *arg) {
  return static_cast<EbaraHydrostationGateway *>(arg)->handle_gap_event(event);
}
void host_task_trampoline(void *param) {
  // Returns only when nimble_port_stop() is called (never, in this component).
  nimble_port_run();
  nimble_port_freertos_deinit();
}

}  // namespace

// ── EbaraTargetMacText ──────────────────────────────────────────────────────
void EbaraTargetMacText::control(const std::string &value) {
  this->publish_state(value);
  if (this->parent_ != nullptr)
    this->parent_->on_target_mac_set(value);
}

// ── EbaraMotorSwitch ─────────────────────────────────────────────────────────
void EbaraMotorSwitch::write_state(bool state) {
  this->publish_state(state);
  // SET commands ("sm-XXXX:N;") always end in ';', unlike GET commands
  // ("gm-XXXX"), which never do.
  if (this->parent_ != nullptr)
    this->parent_->enqueue_set_command(state ? "sm-0005:1;" : "sm-0005:0;");
}

// ── EbaraEnableSwitch ────────────────────────────────────────────────────────
void EbaraEnableSwitch::setup() {
  // gateway_enabled_ already defaults to true (see header), matching
  // RESTORE_DEFAULT_ON's own default — so this only actually changes
  // anything when a persisted OFF preference is being restored.
  bool initial = this->get_initial_state_with_restore_mode().value_or(true);
  this->write_state(initial);
}

void EbaraEnableSwitch::write_state(bool state) {
  this->publish_state(state);
  if (this->parent_ != nullptr)
    this->parent_->set_gateway_enabled(state);
}

// ── EbaraSetpointNumber ──────────────────────────────────────────────────────
void EbaraSetpointNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  int raw = static_cast<int>(std::lround(value * 10));
  std::string cmd;
  switch (this->kind_) {
    case EbaraSetpointKind::TARGET_PRESSURE:
      cmd = "sm-0001:" + std::to_string(raw);
      break;
    case EbaraSetpointKind::START_PRESSURE:
      cmd = "sm-0002:" + std::to_string(raw);
      break;
    case EbaraSetpointKind::DELTA_PRESSURE:
      cmd = "sm-0012:" + std::to_string(raw);
      break;
  }
  this->parent_->enqueue_set_command(cmd + ";");
}

// ── EbaraUpdateIntervalNumber ────────────────────────────────────────────────
namespace {
struct PersistedInterval {
  uint32_t seconds;
  bool valid;
};
}  // namespace

void EbaraUpdateIntervalNumber::setup() {
  this->pref_ = global_preferences->make_preference<PersistedInterval>(
      fnv1_hash("ebara_hydrostation_update_interval"));
  PersistedInterval pi{};
  float initial = kUpdateIntervalDefaultS;
  if (this->pref_.load(&pi) && pi.valid)
    initial = static_cast<float>(pi.seconds);
  this->publish_state(initial);
  if (this->parent_ != nullptr)
    this->parent_->set_update_interval_seconds(initial);
}

void EbaraUpdateIntervalNumber::control(float value) {
  this->publish_state(value);
  PersistedInterval pi{};
  pi.seconds = static_cast<uint32_t>(value);
  pi.valid = true;
  this->pref_.save(&pi);
  if (this->parent_ != nullptr)
    this->parent_->set_update_interval_seconds(value);
}

// ── EbaraHydrostationGateway ─────────────────────────────────────────────────

void EbaraHydrostationGateway::setup() {
  global_ebara_instance = this;
  this->mac_pref_ = global_preferences->make_preference<PersistedMac>(fnv1_hash("ebara_hydrostation_target_mac"));
  this->static_data_pref_ =
      global_preferences->make_preference<PersistedStaticData>(fnv1_hash("ebara_hydrostation_static_data"));
  PersistedStaticData sd{};
  if (this->static_data_pref_.load(&sd) && sd.valid) {
    this->static_data_valid_ = true;
    // Published immediately at boot, with no BLE traffic at all, instead of
    // waiting for the pump to be reachable.
    if (this->firmware_version_sensor_ != nullptr)
      this->firmware_version_sensor_->publish_state(sd.firmware_version_raw / 100.0f);
    if (this->hardware_version_sensor_ != nullptr)
      this->hardware_version_sensor_->publish_state(sd.hardware_version);
    if (this->serial_number_text_ != nullptr)
      this->serial_number_text_->publish_state(std::to_string(sd.serial_number));
  }

  // ESPHome's own esp32/preferences.cpp already called nvs_flash_init() during
  // core startup; calling it again here is the same idiom ESPHome's own
  // esp32_ble component uses (esp32_ble/ble.cpp: ble_pre_setup_()) — it is a
  // cheap no-op once the default partition is initialized, and MUST NOT be
  // followed by an "erase on failure" fallback, since that would wipe
  // ESPHome's own persisted preferences.
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %d", nvs_err);
    this->mark_failed();
    return;
  }

  int rc = nimble_port_init();
  if (rc != 0) {
    ESP_LOGE(TAG, "nimble_port_init failed: rc=%d", rc);
    this->mark_failed();
    return;
  }

  ble_hs_cfg.reset_cb = &reset_cb_trampoline;
  ble_hs_cfg.sync_cb = &sync_cb_trampoline;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  // LE Secure Connections with Numeric Comparison for pairing.
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_YESNO;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 1;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  // NVS-backed persistent bond storage (LTK survives ESP reboots).
  ble_store_config_init();

  nimble_port_freertos_init(&host_task_trampoline);

  this->set_gw_status_text_("Starting BLE stack...");
}

void EbaraHydrostationGateway::loop() {
  // Intentionally empty: the whole session is driven by NimBLE GAP callbacks
  // and named component timeouts (set_timeout/cancel_timeout), not polling.
}

void EbaraHydrostationGateway::update() {
  if (!this->gateway_enabled_)
    return;
  if (this->state_ == GwState::POLL_RUNNING) {
    if (this->cmd_queue_.empty() && !this->cmd_pending_) {
      this->enqueue_full_poll_cycle_();
    }
    return;
  }
  if (this->bond_verify_done_ && this->stop_after_bond_verify_) {
    // Bonding is done and stop_after_bond_verify_ is set — stay idle instead
    // of reconnecting for a service session.
    return;
  }
  if ((this->state_ == GwState::IDLE || this->state_ == GwState::COOLDOWN) && this->have_target_addr_) {
    this->start_connect_(this->bond_verify_done_ ? ConnectPurpose::SERVICE_SESSION : ConnectPurpose::INITIAL_BOND);
  }
}

void EbaraHydrostationGateway::dump_config() {
  ESP_LOGCONFIG(TAG, "Ebara Hydrostation Gateway (NimBLE):");
  ESP_LOGCONFIG(TAG, "  State: %s", state_name_(this->state_));
  ESP_LOGCONFIG(TAG, "  Stop after bond verify: %s", this->stop_after_bond_verify_ ? "YES" : "NO");
}

void EbaraHydrostationGateway::handle_host_reset(int reason) { ESP_LOGW(TAG, "NimBLE host reset; reason=%d", reason); }

void EbaraHydrostationGateway::handle_host_sync() {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: rc=%d", rc);
    return;
  }
  // Generate a non-resolvable-private random address once at startup and
  // use it for every connection, instead of ble_hs_id_infer_auto()'s PUBLIC
  // own-address-type.
  ble_addr_t rnd_addr{};
  rc = ble_hs_id_gen_rnd(1 /* nrpa */, &rnd_addr);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_hs_id_gen_rnd failed: rc=%d — staying on PUBLIC own-address-type", rc);
  } else {
    rc = ble_hs_id_set_rnd(rnd_addr.val);
    if (rc != 0) {
      ESP_LOGW(TAG, "ble_hs_id_set_rnd failed: rc=%d — staying on PUBLIC own-address-type", rc);
    } else {
      this->use_random_own_addr_ = true;
      ESP_LOGD(TAG, "Random own-address configured.");
    }
  }
  ESP_LOGI(TAG, "NimBLE host synced and ready.");
  this->set_gw_status_text_("BLE ready");
  if (!this->have_target_addr_ && !this->default_target_mac_.empty()) {
    ESP_LOGI(TAG, "Applying default_target_mac from YAML: %s", this->default_target_mac_.c_str());
    if (this->target_mac_text_ != nullptr)
      this->target_mac_text_->publish_state(this->default_target_mac_);
    this->on_target_mac_set(this->default_target_mac_);
    return;  // on_target_mac_set() already starts the connection.
  }
  if (!this->have_target_addr_) {
    PersistedMac pm{};
    if (this->mac_pref_.load(&pm) && pm.valid) {
      char mac_str[18];
      snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", pm.mac[5], pm.mac[4], pm.mac[3],
               pm.mac[2], pm.mac[1], pm.mac[0]);
      ESP_LOGI(TAG, "Restored target MAC from flash: %s", mac_str);
      if (this->target_mac_text_ != nullptr)
        this->target_mac_text_->publish_state(mac_str);
      this->on_target_mac_set(mac_str);
      return;  // on_target_mac_set() already starts the connection.
    }
  }
  if (this->gateway_enabled_ && this->have_target_addr_ && this->state_ == GwState::IDLE) {
    this->start_connect_(this->bond_verify_done_ ? ConnectPurpose::SERVICE_SESSION : ConnectPurpose::INITIAL_BOND);
  } else if (!this->have_target_addr_) {
    // No MAC known yet (first-time setup) — scan so the HA config flow can
    // list nearby Hydrostations for the user to pick from.
    this->start_scan_();
  }
}

void EbaraHydrostationGateway::on_target_mac_set(const std::string &mac_str) {
  if (mac_str.empty()) {
    // Explicit clear: forget the target entirely instead of silently doing
    // nothing — otherwise the persisted MAC in flash and have_target_addr_
    // were left untouched, so the old MAC reappeared unchanged after a
    // reboot and scanning never resumed.
    ESP_LOGI(TAG, "Target MAC cleared — forgetting persisted MAC and resuming scan.");
    this->have_target_addr_ = false;
    this->target_addr_ = ble_addr_t{};
    this->bond_verify_done_ = false;
    this->session_primed_ = false;
    PersistedMac pm{};
    pm.valid = false;
    this->mac_pref_.save(&pm);
    PersistedStaticData sd{};
    this->static_data_pref_.save(&sd);
    this->static_data_valid_ = false;
    this->cmd_pending_ = false;
    this->cmd_queue_.clear();
    this->consecutive_timeouts_ = 0;
    this->cancel_timeout("cmd_timeout");
    this->cancel_timeout("intercmd_delay");
    this->cancel_timeout("subscribe_delay");
    this->cancel_timeout("bond_verify_delay");
    this->cancel_timeout("uart_prime_hold");
    if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
      // Scanning resumes from the BLE_GAP_EVENT_DISCONNECT handler once the
      // link actually tears down.
      ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
      this->set_state_(GwState::COOLDOWN);
    } else {
      this->set_state_(GwState::COOLDOWN);
      if (this->gateway_enabled_)
        this->start_scan_();
    }
    return;
  }

  ble_addr_t addr{};
  if (!parse_mac_address(mac_str, &addr)) {
    ESP_LOGE(TAG, "Invalid MAC address: '%s' (expected AA:BB:CC:DD:EE:FF)", mac_str.c_str());
    return;
  }

  // Only an actual change of target (a different physical pump) should
  // invalidate the cached static data below — a reboot restoring the same,
  // already-known MAC from flash must not, or the one-time fetch would
  // repeat on every single boot. Compared against the persisted MAC (not
  // target_addr_/have_target_addr_, which reset to their in-RAM defaults on
  // every boot regardless of what's actually on flash).
  PersistedMac old_pm{};
  bool had_valid_mac = this->mac_pref_.load(&old_pm) && old_pm.valid;
  bool mac_changed = !had_valid_mac || memcmp(addr.val, old_pm.mac, sizeof(addr.val)) != 0;

  this->stop_scan_();
  this->target_addr_ = addr;
  this->have_target_addr_ = true;
  PersistedMac pm{};
  memcpy(pm.mac, addr.val, sizeof(pm.mac));
  pm.valid = true;
  this->mac_pref_.save(&pm);
  if (mac_changed) {
    PersistedStaticData sd{};
    this->static_data_pref_.save(&sd);
    this->static_data_valid_ = false;
  }
  ESP_LOGI(TAG, "Target MAC set: %s (saved to flash)", mac_str.c_str());
  if (this->gateway_enabled_ && this->state_ == GwState::IDLE) {
    this->start_connect_(this->bond_verify_done_ ? ConnectPurpose::SERVICE_SESSION : ConnectPurpose::INITIAL_BOND);
  }
}

void EbaraHydrostationGateway::set_gateway_enabled(bool enabled) {
  if (enabled == this->gateway_enabled_)
    return;
  this->gateway_enabled_ = enabled;

  if (!enabled) {
    ESP_LOGI(TAG, "Gateway disabled — disconnecting.");
    this->cmd_pending_ = false;
    this->cmd_queue_.clear();
    this->consecutive_timeouts_ = 0;
    this->cancel_timeout("cmd_timeout");
    this->cancel_timeout("intercmd_delay");
    this->cancel_timeout("subscribe_delay");
    this->cancel_timeout("bond_verify_delay");
    this->cancel_timeout("uart_prime_hold");
    if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
      ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
    }
    this->set_state_(GwState::COOLDOWN);
    this->set_gw_status_text_("Disabled");
    return;
  }

  ESP_LOGI(TAG, "Gateway enabled — resuming.");
  if (this->have_target_addr_ && (this->state_ == GwState::IDLE || this->state_ == GwState::COOLDOWN)) {
    this->start_connect_(this->bond_verify_done_ ? ConnectPurpose::SERVICE_SESSION : ConnectPurpose::INITIAL_BOND);
  }
}

void EbaraHydrostationGateway::set_update_interval_seconds(float seconds) {
  if (seconds < kUpdateIntervalMinS)
    seconds = kUpdateIntervalMinS;
  else if (seconds > kUpdateIntervalMaxS)
    seconds = kUpdateIntervalMaxS;
  this->set_update_interval(static_cast<uint32_t>(seconds * 1000));
  ESP_LOGI(TAG, "Update interval set to %.0fs", seconds);
}

void EbaraHydrostationGateway::enqueue_set_command(const std::string &cmd) {
  if (this->state_ != GwState::POLL_RUNNING) {
    ESP_LOGW(TAG, "Cannot send '%s': gateway is not connected/polling yet (state=%s)", cmd.c_str(),
             state_name_(this->state_));
    return;
  }
  // SET commands issued by the user jump ahead of the periodic GET queue.
  this->cmd_queue_.insert(this->cmd_queue_.begin(), cmd);
  this->pump_command_queue_();
}

void EbaraHydrostationGateway::start_connect_(ConnectPurpose purpose) {
  if (!this->have_target_addr_) {
    ESP_LOGW(TAG, "start_connect_: no target MAC configured yet");
    return;
  }
  this->connect_purpose_ = purpose;
  this->passkey_seen_this_connection_ = false;
  this->set_state_(GwState::CONNECTING);
  this->set_gw_status_text_("Connecting...");

  uint8_t own_addr_type;
  int rc;
  if (this->use_random_own_addr_) {
    own_addr_type = BLE_OWN_ADDR_RANDOM;
  } else {
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d", rc);
      this->set_state_(GwState::COOLDOWN);
      return;
    }
  }

  // No preceding scan: the target address is already known (entered via
  // Home Assistant or restored from flash).
  //
  // Explicit, tight connection parameters (scan_itvl/window=96, itvl_min=12
  // (15ms), itvl_max=24 (30ms), latency=0, supervision_timeout=72 (720ms))
  // are used from the very first connection event, instead of NimBLE's
  // looser defaults — the BM71 issues unsolicited GATT queries immediately
  // after connecting, before service discovery and the CCCD write, and
  // needs a short connection interval active for that exchange rather than
  // one corrected afterward via ble_gap_update_params().
  struct ble_gap_conn_params conn_params {};
  conn_params.scan_itvl = 96;
  conn_params.scan_window = 96;
  conn_params.itvl_min = 12;
  conn_params.itvl_max = 24;
  conn_params.latency = 0;
  conn_params.supervision_timeout = 72;
  conn_params.min_ce_len = 0;
  conn_params.max_ce_len = 0;
  rc = ble_gap_connect(own_addr_type, &this->target_addr_, 15000 /* ms connect timeout */,
                        &conn_params, &gap_event_trampoline, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_connect failed: rc=%d", rc);
    this->set_state_(GwState::COOLDOWN);
  }
}

// ── BLE scan for nearby Hydrostations (pre-configuration only) ──────────────
//
// Active scan (passive=0) so SCAN_RSP packets are captured too, in case the
// device name isn't in the primary ADV_IND. filter_duplicates=1 since a
// one-time discovery list doesn't need continuous RSSI updates per device.
// Runs for BLE_HS_FOREVER, until a target MAC is chosen and set.

void EbaraHydrostationGateway::start_scan_() {
  if (this->scanning_)
    return;

  uint8_t own_addr_type;
  int rc;
  if (this->use_random_own_addr_) {
    own_addr_type = BLE_OWN_ADDR_RANDOM;
  } else {
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d", rc);
      return;
    }
  }

  struct ble_gap_disc_params disc_params {};
  disc_params.passive = 0;
  disc_params.filter_duplicates = 1;
  disc_params.itvl = 0;
  disc_params.window = 0;
  disc_params.filter_policy = 0;
  disc_params.limited = 0;

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, &gap_event_trampoline, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_disc failed: rc=%d", rc);
    return;
  }
  this->scanning_ = true;
  this->discovered_hydros_.clear();
  this->publish_discovered_();
  ESP_LOGI(TAG, "Scanning for nearby Hydrostations...");
  this->set_gw_status_text_("Scanning...");
  this->set_timeout("scan_heartbeat", 10000, [this]() { this->scan_heartbeat_(); });
}

void EbaraHydrostationGateway::scan_heartbeat_() {
  if (!this->scanning_)
    return;
  // Without this, the log goes completely silent after the last new device
  // is found even though the scan is still running — this makes "still
  // alive" observable instead of indistinguishable from "stopped working".
  ESP_LOGI(TAG, "Still scanning for Hydrostations... (%u found so far)", this->discovered_hydros_.size());
  this->set_timeout("scan_heartbeat", 10000, [this]() { this->scan_heartbeat_(); });
}

void EbaraHydrostationGateway::stop_scan_() {
  if (!this->scanning_)
    return;
  ble_gap_disc_cancel();
  this->scanning_ = false;
  this->cancel_timeout("scan_heartbeat");
  ESP_LOGI(TAG, "Stopped scanning for Hydrostations (%u found).", this->discovered_hydros_.size());
}

void EbaraHydrostationGateway::publish_discovered_() {
  if (this->discovered_text_ == nullptr)
    return;
  std::string json = "[";
  for (size_t i = 0; i < this->discovered_hydros_.size(); i++) {
    const auto &h = this->discovered_hydros_[i];
    if (i > 0)
      json += ",";
    json += "{\"mac\":\"" + h.mac + "\",\"name\":\"" + h.name + "\",\"rssi\":" + std::to_string(h.rssi) + "}";
  }
  json += "]";
  ESP_LOGI(TAG, "Discovered Hydrostations entity updated: %s", json.c_str());
  this->discovered_text_->publish_state(json);
}

void EbaraHydrostationGateway::fail_discovery_(const char *reason) {
  ESP_LOGE(TAG, "%s", reason);
  if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
  }
  this->set_state_(GwState::COOLDOWN);
}

void EbaraHydrostationGateway::begin_bond_verify_disconnect_() {
  this->set_state_(GwState::BOND_VERIFY_DISCONNECTING);
  if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
  }
}

void EbaraHydrostationGateway::after_bond_confirmed_() {
  if (this->stop_after_bond_verify_) {
    ESP_LOGI(TAG, "stop_after_bond_verify is enabled — halting after bond verification.");
    this->set_state_(GwState::COOLDOWN);
    if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
      ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
    }
    return;
  }
  this->begin_service_discovery_();
}

int EbaraHydrostationGateway::handle_gap_event(struct ble_gap_event *event) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
      if (event->connect.status == 0) {
        this->conn_handle_ = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connected (conn_handle=%u), purpose=%d", this->conn_handle_,
                 static_cast<int>(this->connect_purpose_));
        if (this->connect_purpose_ == ConnectPurpose::INITIAL_BOND) {
          // No bond exists yet: run GATT discovery first, then trigger
          // pairing reactively via the (still unencrypted) CCCD write in
          // write_cccd_cb_, rather than calling ble_gap_security_initiate()
          // proactively before any GATT operation.
          this->begin_service_discovery_();
        } else {
          // A bond already exists (BOND_VERIFY reconnect or SERVICE_SESSION)
          // — encrypt proactively right after connect.
          int rc = ble_gap_security_initiate(this->conn_handle_);
          if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_security_initiate failed: rc=%d", rc);
            ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
            this->set_state_(GwState::COOLDOWN);
            return 0;
          }
          this->set_state_(GwState::ENCRYPTING);
        }
      } else {
        ESP_LOGW(TAG, "Connection attempt failed: status=%d", event->connect.status);
        this->set_state_(GwState::COOLDOWN);
        this->set_gw_status_text_("Connect failed");
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
      ESP_LOGI(TAG, "Disconnected; reason=%d (state=%s)", event->disconnect.reason, state_name_(this->state_));
      this->conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
      if (this->state_ == GwState::BOND_VERIFY_DISCONNECTING) {
        this->set_gw_status_text_("Verifying bond persistence...");
        this->set_timeout("bond_verify_delay", 3000,
                           [this]() { this->start_connect_(ConnectPurpose::BOND_VERIFY); });
      } else if (this->state_ == GwState::COOLDOWN) {
        // Expected (e.g. the stop_after_bond_verify_ disconnect, or a
        // target-MAC clear tearing down an active session) — resume
        // scanning if no target is set anymore.
        if (!this->have_target_addr_ && this->gateway_enabled_) {
          this->start_scan_();
        }
      } else {
        ESP_LOGW(TAG, "Unexpected disconnect while in state %s", state_name_(this->state_));
        this->cmd_pending_ = false;
        this->cmd_queue_.clear();
        this->consecutive_timeouts_ = 0;
        this->cancel_timeout("cmd_timeout");
        this->cancel_timeout("intercmd_delay");
        this->cancel_timeout("subscribe_delay");
        this->set_state_(GwState::COOLDOWN);
        this->set_gw_status_text_("Disconnected");
      }
      return 0;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
      if (event->enc_change.status != 0) {
        ESP_LOGE(TAG, "Encryption failed: status=%d", event->enc_change.status);
        // Self-heal a stale/mismatched local bond: this connection may have
        // been a SERVICE_SESSION/BOND_VERIFY reconnect using a locally
        // stored LTK the BM71 no longer recognizes (e.g. the pump was power
        // cycled and lost its own bond record independently of our NVS).
        // Wipe our side too and fall back to a fresh INITIAL_BOND attempt
        // next time, instead of retrying the same doomed proactive encrypt
        // forever.
        struct ble_gap_conn_desc desc {};
        if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
          ble_store_util_delete_peer(&desc.peer_id_addr);
          ESP_LOGW(TAG, "Deleted local bond for this peer — will retry as a fresh pairing.");
        }
        this->bond_verify_done_ = false;
        if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE)
          ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
        this->set_state_(GwState::COOLDOWN);
        return 0;
      }
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
        ESP_LOGI(TAG, "Encrypted: authenticated=%d bonded=%d", desc.sec_state.authenticated,
                 desc.sec_state.bonded);
      }
      switch (this->connect_purpose_) {
        case ConnectPurpose::INITIAL_BOND:
          if (this->cccd_retry_pending_) {
            // The BM71 reacted to our failed CCCD write by initiating
            // pairing; now that we're encrypted, retry the same write.
            ESP_LOGD(TAG, "Encrypted after reactive pairing — retrying CCCD write.");
            uint8_t value[2] = {0x01, 0x00};
            int rc = ble_gattc_write_flat(this->conn_handle_, this->cccd_handle_, value, sizeof(value),
                                           &write_cccd_cb_, this);
            if (rc != 0) {
              ESP_LOGE(TAG, "CCCD write retry failed to start: rc=%d", rc);
              ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
              this->set_state_(GwState::COOLDOWN);
              return 0;
            }
            this->set_state_(GwState::WRITE_CCCD);
          }
          // Else: encryption completed before discovery even reached the
          // CCCD write (e.g. the BM71 initiated pairing on its own).
          // Nothing to do — discovery is already in flight and the
          // eventual CCCD write will simply succeed on its first attempt.
          break;
        case ConnectPurpose::BOND_VERIFY:
          if (!this->passkey_seen_this_connection_) {
            ESP_LOGD(TAG, "Reconnected using stored bond — no new pairing dialog.");
          } else {
            ESP_LOGW(TAG, "Pairing dialog reappeared during bond-verify reconnect — bond was not reused.");
          }
          this->bond_verify_done_ = true;
          this->set_state_(GwState::BOND_CONFIRMED);
          this->set_gw_status_text_("Bonded (verified)");
          this->after_bond_confirmed_();
          break;
        case ConnectPurpose::SERVICE_SESSION:
          this->begin_service_discovery_();
          break;
      }
      return 0;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
      this->passkey_seen_this_connection_ = true;
      if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
        ESP_LOGI(TAG, "Numeric comparison passkey: %06" PRIu32 " — auto-confirming.",
                 event->passkey.params.numcmp);
        struct ble_sm_io pkey {};
        pkey.action = BLE_SM_IOACT_NUMCMP;
        pkey.numcmp_accept = 1;
        int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        if (rc != 0) {
          ESP_LOGE(TAG, "ble_sm_inject_io failed: rc=%d", rc);
        }
      } else {
        // Only NUMCMP is expected for this SC+MITM+DisplayYesNo configuration.
        ESP_LOGW(TAG, "Unexpected passkey action=%d", event->passkey.params.action);
      }
      return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
        ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
      if (this->tx_val_handle_ == 0 || event->notify_rx.attr_handle != this->tx_val_handle_) {
        return 0;
      }
      uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      if (len == 0 || len > 256)
        return 0;
      uint8_t buf[256];
      os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
      this->on_notification_(buf, len);
      return 0;
    }

    case BLE_GAP_EVENT_MTU:
      ESP_LOGD(TAG, "MTU update: conn_handle=%u mtu=%u", event->mtu.conn_handle, event->mtu.value);
      return 0;

    case BLE_GAP_EVENT_L2CAP_UPDATE_REQ: {
      // NimBLE auto-accepts and applies the peer-requested params
      // internally (rc==0 here); this handler only logs it.
      ESP_LOGD(TAG, "L2CAP conn-param update requested by peer: itvl_min=%u itvl_max=%u latency=%u timeout=%u",
               event->conn_update_req.peer_params->itvl_min, event->conn_update_req.peer_params->itvl_max,
               event->conn_update_req.peer_params->latency,
               event->conn_update_req.peer_params->supervision_timeout);
      return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE: {
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
        ESP_LOGD(TAG, "Conn params updated: status=%d itvl=%u (%.2fms) latency=%u timeout=%u (%ums)",
                 event->conn_update.status, desc.conn_itvl, desc.conn_itvl * 1.25,
                 desc.conn_latency, desc.supervision_timeout, desc.supervision_timeout * 10);
      } else {
        ESP_LOGD(TAG, "Conn params updated: status=%d", event->conn_update.status);
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISC: {
      struct ble_hs_adv_fields fields {};
      if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0)
        return 0;
      if (fields.name == nullptr || fields.name_len == 0)
        return 0;
      // Only "HYDRO_XXXX" / "HYSTA_XXXX" advertised names are Hydrostation
      // pumps.
      char name_buf[24];
      size_t name_len = std::min<size_t>(fields.name_len, sizeof(name_buf) - 1);
      memcpy(name_buf, fields.name, name_len);
      name_buf[name_len] = '\0';
      bool is_hydro = (strncmp(name_buf, "HYDRO_", 6) == 0) || (strncmp(name_buf, "HYSTA_", 6) == 0);
      if (!is_hydro)
        return 0;

      // Address is little-endian on the wire; human "AA:BB:..." form is
      // big-endian — same reversal convention as parse_mac_address().
      char mac_buf[18];
      const uint8_t *v = event->disc.addr.val;
      snprintf(mac_buf, sizeof(mac_buf), "%02X:%02X:%02X:%02X:%02X:%02X", v[5], v[4], v[3], v[2], v[1], v[0]);

      for (auto &h : this->discovered_hydros_) {
        if (h.mac == mac_buf) {
          h.rssi = event->disc.rssi;
          this->publish_discovered_();
          return 0;
        }
      }
      if (this->discovered_hydros_.size() >= 10)
        return 0;
      this->discovered_hydros_.push_back({mac_buf, name_buf, event->disc.rssi});
      ESP_LOGI(TAG, "Discovered Hydrostation: %s (%s, %d dBm)", name_buf, mac_buf, event->disc.rssi);
      this->publish_discovered_();
      return 0;
    }

    default:
      return 0;
  }
}

// ── GATT discovery (ALL services → ALL chars per service → ALL descriptors
//     per characteristic → write CCCD) ──────────────────────────────────────
//
// Walks the entire GATT database — Generic Access, Generic Attribute,
// Device Information, then the custom UART service — reading
// characteristics and descriptors from every service, not just the one
// actually needed.

void EbaraHydrostationGateway::begin_service_discovery_() {
  this->set_state_(GwState::DISCOVER_SVC);
  this->tx_val_handle_ = this->tx_def_handle_ = this->cccd_handle_ = this->rx_val_handle_ = 0;
  this->disc_services_.clear();
  this->disc_svc_idx_ = 0;
  this->disc_chrs_.clear();
  this->disc_chr_idx_ = 0;
  int rc = ble_gattc_disc_all_svcs(this->conn_handle_, &disc_svc_cb_, this);
  if (rc != 0) {
    this->fail_discovery_("ble_gattc_disc_all_svcs failed");
  }
}

int EbaraHydrostationGateway::disc_svc_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                            const struct ble_gatt_svc *service, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  switch (error->status) {
    case 0:
      self->disc_services_.push_back({service->start_handle, service->end_handle});
      return 0;
    case BLE_HS_EDONE:
      if (self->disc_services_.empty()) {
        self->fail_discovery_("No services found on peer at all");
        return 0;
      }
      ESP_LOGD(TAG, "Discovered %u services — discovering characteristics for each", self->disc_services_.size());
      self->set_state_(GwState::DISCOVER_CHR);
      self->disc_svc_idx_ = 0;
      self->disc_chrs_svc_start_idx_ = self->disc_chrs_.size();
      {
        const auto &svc = self->disc_services_[0];
        int rc = ble_gattc_disc_all_chrs(self->conn_handle_, svc.start_handle, svc.end_handle, &disc_chr_cb_, self);
        if (rc != 0) {
          self->fail_discovery_("ble_gattc_disc_all_chrs failed");
        }
      }
      return 0;
    default:
      self->fail_discovery_("Service discovery error");
      return 0;
  }
}

int EbaraHydrostationGateway::disc_chr_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                            const struct ble_gatt_chr *chr, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  switch (error->status) {
    case 0: {
      if (ble_uuid_cmp(&chr->uuid.u, (const ble_uuid_t *) &kTxNotifyUuid) == 0) {
        self->tx_val_handle_ = chr->val_handle;
        self->tx_def_handle_ = chr->def_handle;
      } else if (ble_uuid_cmp(&chr->uuid.u, (const ble_uuid_t *) &kRxWriteUuid) == 0) {
        self->rx_val_handle_ = chr->val_handle;
      }
      // Tighten the previous characteristic's descriptor-search upper bound
      // to just before this one, since they're reported in ascending handle
      // order within the same service scan. Only touch disc_chrs_.back() if
      // it was added during this service's scan (disc_chrs_.size() >
      // disc_chrs_svc_start_idx_) — otherwise, on the first characteristic
      // of a new service, this would corrupt the previous service's last
      // characteristic with a handle from an unrelated service.
      if (self->disc_chrs_.size() > self->disc_chrs_svc_start_idx_)
        self->disc_chrs_.back().dsc_range_end = chr->def_handle - 1;
      const auto &svc = self->disc_services_[self->disc_svc_idx_];
      self->disc_chrs_.push_back({chr->val_handle, svc.end_handle});
      return 0;
    }
    case BLE_HS_EDONE: {
      self->disc_svc_idx_++;
      if (self->disc_svc_idx_ < self->disc_services_.size()) {
        self->disc_chrs_svc_start_idx_ = self->disc_chrs_.size();
        const auto &svc = self->disc_services_[self->disc_svc_idx_];
        int rc = ble_gattc_disc_all_chrs(self->conn_handle_, svc.start_handle, svc.end_handle, &disc_chr_cb_, self);
        if (rc != 0) {
          self->fail_discovery_("ble_gattc_disc_all_chrs failed");
        }
        return 0;
      }
      // All services' characteristics discovered.
      if (self->tx_val_handle_ == 0 || self->rx_val_handle_ == 0) {
        self->fail_discovery_("TX/RX characteristic not found");
        return 0;
      }
      ESP_LOGD(TAG, "Discovered %u characteristics across all services — discovering descriptors for each",
               self->disc_chrs_.size());
      self->set_state_(GwState::DISCOVER_CCCD);
      self->disc_chr_idx_ = 0;
      while (self->disc_chr_idx_ < self->disc_chrs_.size() &&
             self->disc_chrs_[self->disc_chr_idx_].dsc_range_end <
                 static_cast<uint16_t>(self->disc_chrs_[self->disc_chr_idx_].val_handle + 1)) {
        self->disc_chr_idx_++;
      }
      if (self->disc_chr_idx_ >= self->disc_chrs_.size()) {
        // No characteristic anywhere has room for descriptors — impossible
        // in practice (our own TX char always has a CCCD), but handle it.
        self->fail_discovery_("CCCD for TX characteristic not found");
        return 0;
      }
      {
        const auto &c = self->disc_chrs_[self->disc_chr_idx_];
        // NimBLE quirk verified directly in ble_gattc.c: ble_gattc_disc_all_dscs()
        // internally does ANOTHER +1 on start_handle before sending the ATT
        // Find Information Request (ble_gattc_disc_all_dscs_tx() uses
        // proc->disc_all_dscs.prev_handle + 1, and prev_handle is set to
        // whatever start_handle we pass). Passing val_handle+1 here (as if
        // start_handle meant "first handle to search") made the real
        // on-the-wire search start at val_handle+2 — one past the actual
        // CCCD — producing an inverted, invalid range and a hard EINVAL.
        // Passing val_handle directly (unshifted) lets NimBLE's own +1 land
        // on the correct first handle.
        int rc = ble_gattc_disc_all_dscs(self->conn_handle_, c.val_handle, c.dsc_range_end, &disc_cccd_cb_, self);
        if (rc != 0) {
          self->fail_discovery_("ble_gattc_disc_all_dscs failed");
        }
      }
      return 0;
    }
    default:
      self->fail_discovery_("Characteristic discovery error");
      return 0;
  }
}

int EbaraHydrostationGateway::disc_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                             uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  switch (error->status) {
    case 0:
      if (chr_val_handle == self->tx_val_handle_ &&
          ble_uuid_cmp(&dsc->uuid.u, (const ble_uuid_t *) &kCccdUuid) == 0) {
        self->cccd_handle_ = dsc->handle;
      }
      return 0;
    case BLE_HS_EDONE: {
      self->disc_chr_idx_++;
      // Skip any characteristic with no room for descriptors before the next
      // one (dsc_range_end < val_handle+1) — nothing to search there.
      while (self->disc_chr_idx_ < self->disc_chrs_.size() &&
             self->disc_chrs_[self->disc_chr_idx_].dsc_range_end <
                 static_cast<uint16_t>(self->disc_chrs_[self->disc_chr_idx_].val_handle + 1)) {
        self->disc_chr_idx_++;
      }
      if (self->disc_chr_idx_ < self->disc_chrs_.size()) {
        const auto &c = self->disc_chrs_[self->disc_chr_idx_];
        // See the matching comment above (in disc_chr_cb_) — NimBLE applies
        // its own +1 to start_handle internally, so pass val_handle unshifted.
        int rc = ble_gattc_disc_all_dscs(self->conn_handle_, c.val_handle, c.dsc_range_end, &disc_cccd_cb_, self);
        if (rc != 0) {
          self->fail_discovery_("ble_gattc_disc_all_dscs failed");
        }
        return 0;
      }
      // All characteristics' descriptors discovered.
      if (self->cccd_handle_ == 0) {
        self->fail_discovery_("CCCD for TX characteristic not found");
        return 0;
      }
      ESP_LOGD(TAG, "Full GATT database discovery complete");
      self->set_state_(GwState::WRITE_CCCD);
      {
        struct ble_gap_conn_desc desc {};
        if (ble_gap_conn_find(self->conn_handle_, &desc) == 0) {
          ESP_LOGD(TAG, "Pre-CCCD-write link state: encrypted=%d authenticated=%d bonded=%d",
                   desc.sec_state.encrypted, desc.sec_state.authenticated, desc.sec_state.bonded);
        }
        // 0x0001 enables notifications (not indications) on the CCCD.
        uint8_t value[2] = {0x01, 0x00};
        int rc = ble_gattc_write_flat(self->conn_handle_, self->cccd_handle_, value, sizeof(value), &write_cccd_cb_,
                                       self);
        if (rc != 0) {
          self->fail_discovery_("CCCD write failed to start");
        }
      }
      return 0;
    }
    default:
      self->fail_discovery_("Descriptor discovery error");
      return 0;
  }
}

int EbaraHydrostationGateway::write_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                              struct ble_gatt_attr *attr, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  if (error->status != 0) {
    if (self->connect_purpose_ == ConnectPurpose::INITIAL_BOND && !self->cccd_retry_pending_) {
      // Expected: the unencrypted CCCD write is rejected — this is the
      // reactive bonding trigger. Wait for the BM71's Security Request /
      // the resulting ENC_CHANGE, which retries this same write.
      ESP_LOGD(TAG, "CCCD write rejected (status=%d) as expected pre-bond — waiting for reactive pairing.",
               error->status);
      self->cccd_retry_pending_ = true;
      return 0;
    }
    self->fail_discovery_("CCCD write failed");
    return 0;
  }
  self->cccd_retry_pending_ = false;

  // INITIAL_BOND is only ever the purpose of the very first connection of a
  // boot; every connection made with this purpose moves on to
  // SERVICE_SESSION afterward.
  if (self->connect_purpose_ == ConnectPurpose::INITIAL_BOND) {
    if (self->passkey_seen_this_connection_) {
      ESP_LOGI(TAG, "Fresh pairing completed.");
    } else {
      ESP_LOGI(TAG, "Bond already existed (survived reboot).");
      self->bond_verify_done_ = true;
    }
    self->connect_purpose_ = ConnectPurpose::SERVICE_SESSION;
  }

  ESP_LOGD(TAG, "CCCD written — subscribed to notifications.");
  {
    struct ble_gap_conn_desc desc {};
    if (ble_gap_conn_find(self->conn_handle_, &desc) == 0) {
      ESP_LOGD(TAG, "Conn params at CCCD-write-success: itvl=%u (%.2fms) latency=%u timeout=%u (%ums)",
               desc.conn_itvl, desc.conn_itvl * 1.25, desc.conn_latency, desc.supervision_timeout,
               desc.supervision_timeout * 10);
    }
  }
  // Force the connection parameters the BM71 itself requests right after
  // CCCD-write-ack (itvl 8-24, i.e. 10-30ms, latency 0, timeout 512, i.e.
  // 5.12s), applied on every CCCD-write success including the priming
  // connection.
  {
    struct ble_gap_upd_params params {};
    params.itvl_min = 8;
    params.itvl_max = 24;
    params.latency = 0;
    params.supervision_timeout = 512;
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    int rc = ble_gap_update_params(self->conn_handle_, &params);
    if (rc != 0)
      ESP_LOGW(TAG, "ble_gap_update_params failed: rc=%d", rc);
  }

  if (!self->session_primed_) {
    self->session_primed_ = true;
    // A priming connection (encrypt, discover, CCCD write, hold 5s,
    // disconnect) always precedes the real session, exactly once per boot,
    // regardless of whether this connection did fresh pairing or reused an
    // existing bond.
    ESP_LOGD(TAG, "Priming connection complete — holding 5s before reconnecting for the real session.");
    self->set_timeout("uart_prime_hold", 5000, [self]() { self->begin_bond_verify_disconnect_(); });
    return 0;
  }

  ESP_LOGI(TAG, "Connected and subscribed — starting poll cycle.");
  self->set_state_(GwState::WAIT_BEFORE_POLL);
  self->set_gw_status_text_("Connected");
  self->set_timeout("subscribe_delay", 800, [self]() {
    self->set_state_(GwState::POLL_RUNNING);
    self->set_gw_status_text_("Polling");
    self->enqueue_full_poll_cycle_();
  });
  return 0;
}

// ── Sequential command queue (GET polling + SET writes) ─────────────────────

void EbaraHydrostationGateway::enqueue_full_poll_cycle_() {
  // gm-0061 and gm-0063 are aggregate GET commands that return multiple
  // values in a single round trip: gm-0061 replies "Dato:
  // actPress,freqMotor,currMotor" and gm-0063 replies "Dato:
  // targetPress,deltaPress", each replacing what would otherwise be 3 and 2
  // individual gm-000x GETs. The pump protocol has no way to combine
  // arbitrary distinct commands into one write, so these pre-defined
  // aggregates are the only way to reduce round trips — the individual
  // sensors are unaffected, only the wire query changes (see
  // publish_from_response_() for how the multi-value responses are fanned
  // back out). gm-0002 (start pressure) has no aggregate equivalent and
  // stays individual.
  //
  // gm-0032 (lot number) is left out of the active poll cycle since this
  // unit reports it as never-programmed; the entity/parsing stay wired up
  // (see the /* "gm-0032" */ line below) in case a future unit has it set.
  static const char *const kCmds[] = {
      "gm-0005", "gm-0063", "gm-0002", "gm-0061", "gm-0006",
      "gm-0008", "gm-0009", "gm-0011",
  };
  for (const auto *c : kCmds)
    this->cmd_queue_.emplace_back(c);

  // gm-0051 (firmware version), gm-0052 (hardware version), and gm-0031
  // (serial number) are hardware/firmware constants that never change for
  // the life of a pairing — queried once (see on_target_mac_set() for when
  // that cache is invalidated) instead of every cycle. Only the
  // still-missing ones are queued, so a fetch that only partially completed
  // (e.g. a disconnect mid-cycle) resumes without repeating what it already
  // got.
  if (!this->static_data_valid_) {
    PersistedStaticData sd{};
    this->static_data_pref_.load(&sd);
    if (!sd.have_firmware)
      this->cmd_queue_.emplace_back("gm-0051");
    if (!sd.have_hardware)
      this->cmd_queue_.emplace_back("gm-0052");
    if (!sd.have_serial)
      this->cmd_queue_.emplace_back("gm-0031");
  }

  this->pump_command_queue_();
}

void EbaraHydrostationGateway::pump_command_queue_() {
  if (this->state_ != GwState::POLL_RUNNING)
    return;
  if (this->cmd_pending_ || this->cmd_queue_.empty())
    return;
  std::string cmd = this->cmd_queue_.front();
  this->cmd_queue_.erase(this->cmd_queue_.begin());
  this->send_command_now_(cmd);
}

void EbaraHydrostationGateway::send_command_now_(const std::string &cmd) {
  this->cmd_pending_str_ = cmd;
  this->cmd_pending_ = true;
  this->cmd_sent_at_ms_ = millis();
  // Write Command: no ATT response is expected, the pump replies via a
  // notification instead.
  int rc = ble_gattc_write_no_rsp_flat(this->conn_handle_, this->rx_val_handle_, cmd.data(), cmd.size());
  if (rc != 0) {
    ESP_LOGW(TAG, "write_no_rsp_flat failed for '%s': rc=%d", cmd.c_str(), rc);
    this->cmd_pending_ = false;
    this->set_timeout("intercmd_delay", 300, [this]() { this->pump_command_queue_(); });
    return;
  }
  ESP_LOGD(TAG, ">>> %s", cmd.c_str());
  this->set_timeout("cmd_timeout", 8000, [this]() { this->handle_command_timeout_(); });
}

static constexpr uint8_t kMaxConsecutiveTimeouts = 3;

void EbaraHydrostationGateway::handle_command_timeout_() {
  ESP_LOGW(TAG, "TIMEOUT waiting for response to '%s'", this->cmd_pending_str_.c_str());
  this->cmd_pending_ = false;
  this->consecutive_timeouts_++;

  if (this->consecutive_timeouts_ < kMaxConsecutiveTimeouts) {
    this->set_timeout("intercmd_delay", 300, [this]() { this->pump_command_queue_(); });
    return;
  }

  // The pump has stopped answering entirely, but the BLE link itself never
  // reported a disconnect — force one so the normal reconnect path (and its
  // "Disconnected" -> "Connecting..." status updates) actually runs, instead
  // of retrying silently forever with the status stuck on "Polling".
  ESP_LOGE(TAG, "%u consecutive command timeouts — treating link as dead, reconnecting.",
           this->consecutive_timeouts_);
  this->consecutive_timeouts_ = 0;
  this->cmd_queue_.clear();
  this->cancel_timeout("intercmd_delay");
  this->set_state_(GwState::COOLDOWN);
  this->set_gw_status_text_("Unresponsive — reconnecting...");
  if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
  }
}

void EbaraHydrostationGateway::on_notification_(const uint8_t *data, size_t len) {
  std::string raw(reinterpret_cast<const char *>(data), len);
  while (!raw.empty()) {
    char c = raw.back();
    if (c == '\0' || c == '\r' || c == '\n' || c == ';' || c == ' ')
      raw.pop_back();
    else
      break;
  }
  if (!this->cmd_pending_) {
    ESP_LOGW(TAG, "Unexpected notification (no pending command): %s", raw.c_str());
    return;
  }
  this->cancel_timeout("cmd_timeout");
  this->consecutive_timeouts_ = 0;
  std::string cmd = this->cmd_pending_str_;
  this->cmd_pending_ = false;
  this->publish_from_response_(cmd, raw);
  this->set_timeout("intercmd_delay", 300, [this]() { this->pump_command_queue_(); });
}

void EbaraHydrostationGateway::publish_from_response_(const std::string &cmd, const std::string &raw) {
  ESP_LOGI(TAG, "<<< %s : %s", cmd.c_str(), raw.c_str());
  if (raw.find("SET DATO OK") != std::string::npos)
    return;  // SET acknowledgement — nothing to publish.

  auto nums = parse_dato(raw);

  if (nums.empty()) {
    ESP_LOGW(TAG, "Unparsed response for %s: %s", cmd.c_str(), raw.c_str());
    return;
  }
  int32_t v = nums[0];

  if (cmd == "gm-0063") {
    // "Dato: targetPress,deltaPress" — same fields/scale as the individual
    // gm-0001 (target pressure) and gm-0012 (delta pressure) GETs.
    if (nums.size() < 2) {
      ESP_LOGW(TAG, "gm-0063 response missing values: %s", raw.c_str());
    } else {
      float target_press = nums[0] / 10.0f;
      float delta_press = nums[1] / 10.0f;
      if (this->target_pressure_sensor_ != nullptr)
        this->target_pressure_sensor_->publish_state(target_press);
      // Keep the writable setpoint number in sync with the actual value
      // read back from the pump, so it reflects the real state even before
      // the user writes it themselves.
      if (this->target_pressure_number_ != nullptr)
        this->target_pressure_number_->publish_state(target_press);
      if (this->delta_pressure_sensor_ != nullptr)
        this->delta_pressure_sensor_->publish_state(delta_press);
      if (this->delta_pressure_number_ != nullptr)
        this->delta_pressure_number_->publish_state(delta_press);
    }
  } else if (cmd == "gm-0002") {
    if (this->start_pressure_sensor_ != nullptr)
      this->start_pressure_sensor_->publish_state(v / 10.0f);
    if (this->start_pressure_number_ != nullptr)
      this->start_pressure_number_->publish_state(v / 10.0f);
  } else if (cmd == "gm-0061") {
    // "Dato: actPress,freqMotor,currMotor" — same fields/scales as the
    // individual gm-0003 (actual pressure), gm-0007 (motor frequency) and
    // gm-0004 (motor current) GETs.
    if (nums.size() < 3) {
      ESP_LOGW(TAG, "gm-0061 response missing values: %s", raw.c_str());
    } else {
      float actual_press = nums[0] / 10.0f;
      int32_t freq_motor = nums[1];
      float amps = nums[2] / 10.0f;
      if (this->actual_pressure_sensor_ != nullptr)
        this->actual_pressure_sensor_->publish_state(actual_press);
      if (this->motor_frequency_sensor_ != nullptr)
        this->motor_frequency_sensor_->publish_state(freq_motor);
      if (this->motor_current_sensor_ != nullptr)
        this->motor_current_sensor_->publish_state(amps);
      if (this->water_level_sensor_ != nullptr) {
        // Estimated water level, derived from motor current.
        float pct;
        if (amps < 1.0f)
          pct = 0;
        else if (amps < 1.5f)
          pct = 20;
        else if (amps < 2.0f)
          pct = 40;
        else if (amps < 2.4f)
          pct = 60;
        else if (amps < 3.0f)
          pct = 80;
        else
          pct = 100;
        this->water_level_sensor_->publish_state(pct);
      }
    }
  } else if (cmd == "gm-0005") {
    if (this->status_word_sensor_ != nullptr)
      this->status_word_sensor_->publish_state(v);
    // Status-word bitmask: bit14 is inverted (0 = enabled). bit11 is an
    // external-stop condition (e.g. a float switch or dry-run protection)
    // that overrides everything else — the official app's own "Running"
    // status checks it before motor_run for exactly that reason.
    bool motor_error = (v & 0x8000) != 0;
    bool motor_enabled = (v & 0x4000) == 0;
    bool motor_running_raw = (v & 0x2000) != 0;
    bool external_stop_active = (v & 0x0800) == 0;
    bool motor_running = motor_running_raw && !external_stop_active;
    if (this->motor_error_bs_ != nullptr)
      this->motor_error_bs_->publish_state(motor_error);
    if (this->motor_enabled_bs_ != nullptr)
      this->motor_enabled_bs_->publish_state(motor_enabled);
    if (this->motor_running_bs_ != nullptr)
      this->motor_running_bs_->publish_state(motor_running);
    // The "Motor" switch reflects and controls whether the pump's automatic
    // control system is enabled (sm-0005:1/0), not whether the motor
    // happens to be spinning right now — those are independent concepts:
    // the motor only actually runs on water demand even while enabled.
    if (this->motor_switch_ != nullptr)
      this->motor_switch_->publish_state(motor_enabled);
  } else if (cmd == "gm-0006") {
    if (this->working_hours_sensor_ != nullptr)
      this->working_hours_sensor_->publish_state(v);
  } else if (cmd == "gm-0008") {
    if (this->module_temperature_sensor_ != nullptr)
      this->module_temperature_sensor_->publish_state(v);
  } else if (cmd == "gm-0009") {
    if (this->dc_bus_voltage_sensor_ != nullptr)
      this->dc_bus_voltage_sensor_->publish_state(v);
  } else if (cmd == "gm-0011") {
    if (this->error_word_sensor_ != nullptr)
      this->error_word_sensor_->publish_state(v);
    if (this->errors_text_ != nullptr)
      this->errors_text_->publish_state(decode_errors(v));
  } else if (cmd == "gm-0051") {
    if (this->firmware_version_sensor_ != nullptr)
      this->firmware_version_sensor_->publish_state(v / 100.0f);
    PersistedStaticData sd{};
    this->static_data_pref_.load(&sd);
    sd.firmware_version_raw = static_cast<uint16_t>(v);
    sd.have_firmware = true;
    sd.valid = sd.have_serial && sd.have_hardware && sd.have_firmware;
    this->static_data_pref_.save(&sd);
    this->static_data_valid_ = sd.valid;
  } else if (cmd == "gm-0052") {
    if (this->hardware_version_sensor_ != nullptr)
      this->hardware_version_sensor_->publish_state(v);
    PersistedStaticData sd{};
    this->static_data_pref_.load(&sd);
    sd.hardware_version = static_cast<uint16_t>(v);
    sd.have_hardware = true;
    sd.valid = sd.have_serial && sd.have_hardware && sd.have_firmware;
    this->static_data_pref_.save(&sd);
    this->static_data_valid_ = sd.valid;
  } else if (cmd == "gm-0031") {
    // Serial number: parsed unsigned separately (see parse_dato_uint32) and
    // published as text — see the set_serial_number_text_sensor() comment
    // in the header for why.
    uint32_t serial = parse_dato_uint32(raw);
    if (this->serial_number_text_ != nullptr)
      this->serial_number_text_->publish_state(std::to_string(serial));
    PersistedStaticData sd{};
    this->static_data_pref_.load(&sd);
    sd.serial_number = serial;
    sd.have_serial = true;
    sd.valid = sd.have_serial && sd.have_hardware && sd.have_firmware;
    this->static_data_pref_.save(&sd);
    this->static_data_valid_ = sd.valid;
  } else if (cmd == "gm-0032") {
    // Lot number, formatted as a 4-digit zero-padded "XX YYYY" string.
    if (this->lot_number_text_ != nullptr) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%04d", static_cast<int>(v));
      std::string s(buf);
      this->lot_number_text_->publish_state(s.substr(0, 2) + " " + s.substr(2));
    }
  }
}

void EbaraHydrostationGateway::set_state_(GwState s) {
  if (s != this->state_) {
    ESP_LOGD(TAG, "State: %s -> %s", state_name_(this->state_), state_name_(s));
  }
  this->state_ = s;
}

void EbaraHydrostationGateway::set_gw_status_text_(const std::string &s) {
  if (this->gw_status_ != nullptr)
    this->gw_status_->publish_state(s);
}

const char *EbaraHydrostationGateway::state_name_(GwState s) {
  switch (s) {
    case GwState::IDLE:
      return "IDLE";
    case GwState::CONNECTING:
      return "CONNECTING";
    case GwState::ENCRYPTING:
      return "ENCRYPTING";
    case GwState::BOND_VERIFY_DISCONNECTING:
      return "BOND_VERIFY_DISCONNECTING";
    case GwState::BOND_CONFIRMED:
      return "BOND_CONFIRMED";
    case GwState::DISCOVER_SVC:
      return "DISCOVER_SVC";
    case GwState::DISCOVER_CHR:
      return "DISCOVER_CHR";
    case GwState::DISCOVER_CCCD:
      return "DISCOVER_CCCD";
    case GwState::WRITE_CCCD:
      return "WRITE_CCCD";
    case GwState::WAIT_BEFORE_POLL:
      return "WAIT_BEFORE_POLL";
    case GwState::POLL_RUNNING:
      return "POLL_RUNNING";
    case GwState::COOLDOWN:
      return "COOLDOWN";
  }
  return "?";
}

}  // namespace ebara_hydrostation
}  // namespace esphome
