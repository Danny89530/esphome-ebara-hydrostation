// Ebara Hydrostation Gateway — NimBLE implementation.
//
// Owns the entire NimBLE BLE host stack and speaks directly to the pump's
// Microchip BM70/BM71 module over its Transparent UART GATT service: pairs and
// bonds with it, discovers its GATT database, verifies the product family,
// subscribes to notifications, and runs a sequential GET/SET command queue.
//
// Every data mapping below follows the official Android app "Hydrostation"
// 2.0.10015 (com.ebara.hydrostation). Only the HYDRO family (legacy gm-/sm-
// protocol) is supported: HYSTA pumps use a different push-based protocol and
// are reported as unsupported.

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
// Present and linkable in ble_store_config.c (NVS-backed bond storage with
// CONFIG_BT_NIMBLE_NVS_PERSIST=y) but not declared in its public header.
void ble_store_config_init(void);
}

namespace esphome {
namespace ebara_hydrostation {

static const char *const TAG = "ebara_hydrostation";
// Component version (keep in sync with CHANGELOG.md / the GitHub release).
static const char *const COMPONENT_VERSION = "2.0.0";

static constexpr float kUpdateIntervalMinS = 2.0f;
static constexpr float kUpdateIntervalMaxS = 300.0f;
// Slow-changing registers (working hours, temperature, DC bus voltage,
// start pressure) are polled at most this often.
static constexpr uint32_t kSlowPollIntervalMs = 60000;

// Command timing (the official app uses a 3 s per-command timeout).
static constexpr uint32_t kCmdTimeoutMs = 3000;
static constexpr uint32_t kInterCmdDelayMs = 300;
// After a timeout, wait longer before the next command so that a late reply
// to the timed-out command cannot be attributed to the next one (legacy
// replies do not echo the register they answer).
static constexpr uint32_t kPostTimeoutGuardMs = 800;
// Quiet period after which a partial reply without terminator is processed
// (the official app flushes its buffer after 500 ms).
static constexpr uint32_t kRxFlushMs = 300;
static constexpr uint8_t kMaxConsecutiveTimeouts = 3;
// Delay between the CCCD write and the first command (official app: >= 2 s).
static constexpr uint32_t kSubscribeDelayMs = 2000;
// First connection after boot not before this uptime: after an unclean reset
// (crash, flash, power loss) the module keeps the previous link alive until
// its 5.12 s supervision timeout and forwards no UART data to a session opened
// earlier.
static constexpr uint32_t kBootConnectDelayMs = 6500;
// Whole session setup (connect → encrypt → discover → subscribe) must finish
// within this time, otherwise the attempt is aborted and retried.
static constexpr uint32_t kSessionSetupTimeoutMs = 30000;
// Minimum spacing between a disconnect and the next connection attempt.
static constexpr uint32_t kReconnectSpacingMs = 3000;

// psi <-> bar (the app uses 1.45038 on ×10 values, i.e. 14.5038 psi/bar).
static constexpr float kPsiPerBar = 14.5038f;

// ── BM70/BM71 GATT constants (UUID bytes little-endian) ──────────────────────
//   Service:   49535343-fe7d-4ae5-8fa9-9fafd205e455
//   TX/notify: 49535343-1e4d-4bd9-ba61-23c647249616  (handle 0x0052, CCCD 0x0053)
//   RX/write:  49535343-8841-43f4-a8d4-ecbe34729bb3  (handle 0x0055)
static const ble_uuid128_t kServiceUuid = BLE_UUID128_INIT(
    0x55, 0xe4, 0x05, 0xd2, 0xaf, 0x9f, 0xa9, 0x8f, 0xe5, 0x4a, 0x7d, 0xfe, 0x43, 0x53, 0x53, 0x49);
static const ble_uuid128_t kTxNotifyUuid = BLE_UUID128_INIT(
    0x16, 0x96, 0x24, 0x47, 0xc6, 0x23, 0x61, 0xba, 0xd9, 0x4b, 0x4d, 0x1e, 0x43, 0x53, 0x53, 0x49);
static const ble_uuid128_t kRxWriteUuid = BLE_UUID128_INIT(
    0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8, 0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49);
// Named objects instead of BLE_UUID16_DECLARE() (compound-literal address is
// not an lvalue in C++).
static const ble_uuid16_t kCccdUuid = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);
static const ble_uuid16_t kDeviceNameUuid = BLE_UUID16_INIT(0x2A00);

// NimBLE reports HCI errors as BLE_HS_ERR_HCI_BASE (0x200) + HCI code.
static constexpr int kHciErrMicFailure = 0x200 + 0x3D;

static EbaraHydrostationGateway *global_ebara_instance = nullptr;

namespace {

// Parses "AA:BB:CC:DD:EE:FF" into a PUBLIC ble_addr_t (the pump uses a public
// address; NimBLE stores address bytes little-endian).
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

// Strips "Dato:" prefix and trailing terminators; returns the value list part.
bool dato_payload(const std::string &raw, std::string *rest) {
  size_t idx = raw.find("Dato:");
  if (idx == std::string::npos)
    return false;
  std::string r = raw.substr(idx + 5);
  size_t start = r.find_first_not_of(' ');
  if (start == std::string::npos)
    return false;
  r = r.substr(start);
  while (!r.empty()) {
    char c = r.back();
    if (c == ';' || c == ' ' || c == '\r' || c == '\n' || c == '\0')
      r.pop_back();
    else
      break;
  }
  *rest = r;
  return !r.empty();
}

// Parses the first "Dato:" token as unsigned 32-bit (serial number range).
bool parse_dato_uint32(const std::string &raw, uint32_t *out) {
  std::string rest;
  if (!dato_payload(raw, &rest))
    return false;
  size_t comma = rest.find(',');
  std::string tok = (comma == std::string::npos) ? rest : rest.substr(0, comma);
  char *endptr = nullptr;
  unsigned long val = std::strtoul(tok.c_str(), &endptr, 10);
  if (endptr == tok.c_str())
    return false;
  *out = static_cast<uint32_t>(val);
  return true;
}

std::vector<int32_t> parse_dato(const std::string &raw) {
  std::vector<int32_t> out;
  std::string rest;
  if (!dato_payload(raw, &rest))
    return out;
  size_t pos = 0;
  while (pos <= rest.size()) {
    size_t comma = rest.find(',', pos);
    std::string tok = (comma == std::string::npos) ? rest.substr(pos) : rest.substr(pos, comma - pos);
    if (!tok.empty()) {
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

// Number of values each GET reply must carry (shape validation).
size_t expected_value_count(const std::string &cmd) {
  if (cmd == "gm-0061")
    return 3;
  if (cmd == "gm-0063" || cmd == "gm-0062")
    return 2;
  return 1;
}

// Alarm table — codes, masks and texts exactly as in the official app
// (CtrlHydroFragment.decodeStatusWordError + values/strings.xml).
struct AlarmBit {
  uint16_t mask;
  const char *code;
  const char *text;
};
const AlarmBit kAlarmBits[] = {
    {0x0001, "E01", "Overtemperature alarm"},
    {0x0002, "E02", "Low Voltage alarm"},
    {0x0004, "E02", "Overvoltage alarm"},
    {0x0008, "E03", "Overcurrent alarm"},
    {0x0020, "E04", "Short circuit between phases"},
    {0x0100, "E06", "Pressure sensor fault"},
    {0x0400, "H01", "Dry march or working pressure <= 0.5 bar"},  // psi variant below
    {0x0800, "H01+", "Lack of water on the suction side"},
    {0x1000, "H02", "Hourly restarts over the limit"},
    {0x2000, "C01", "Communication error between devices"},
};

std::string decode_alarms(uint16_t word, bool psi_mode) {
  std::string out;
  for (const auto &b : kAlarmBits) {
    if ((word & b.mask) == 0)
      continue;
    if (!out.empty())
      out += ", ";
    out += b.code;
    out += " ";
    if (b.mask == 0x0400 && psi_mode)
      out += "Dry march or working pressure <= 8 psi";
    else
      out += b.text;
  }
  return out;
}

// App 2.0 (InfoDeviceFragment): "FW : %02d.%02d" of the high/low byte.
std::string format_firmware(uint16_t fw) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%02u.%02u", static_cast<unsigned>((fw >> 8) & 0xFF), static_cast<unsigned>(fw & 0xFF));
  return buf;
}

std::string format_serial(uint32_t serial) {
  return serial == 0xFFFFFFFFu ? std::string("not programmed") : std::to_string(serial);
}

// Minimal JSON string escaping (names come from over-the-air advertising).
std::string json_escape(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      char buf[8];
      snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
      out += buf;
    } else {
      out += c;
    }
  }
  return out;
}

// True if the raw advertising data contains `tag` (cheap pre-filter run in
// the NimBLE task: only Ebara adverts are forwarded to the main loop).
bool adv_contains(const uint8_t *data, size_t len, const char *tag) {
  const size_t tlen = strlen(tag);
  if (len < tlen)
    return false;
  for (size_t i = 0; i + tlen <= len; i++) {
    if (memcmp(data + i, tag, tlen) == 0)
      return true;
  }
  return false;
}

// Signed wrap-safe "now >= deadline" on millis() values.
bool time_reached(uint32_t now, uint32_t deadline) { return static_cast<int32_t>(now - deadline) >= 0; }

// "FREQUENCY SPEED" percentage shown by the app (DeviceInfo.getPercentAcquaMotor),
// computed from the RAW motor current (A×10).
float pump_load_from_raw_current(int32_t raw) {
  if (raw >= 30)
    return 100;
  if (raw >= 24)
    return 80;
  if (raw >= 20)
    return 60;
  if (raw >= 15)
    return 40;
  if (raw >= 1)
    return 20;
  return 0;
}

// ── NimBLE C-linkage trampolines ─────────────────────────────────────────────
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
  nimble_port_run();
  nimble_port_freertos_deinit();
}

}  // namespace

// ── Entities ─────────────────────────────────────────────────────────────────
void EbaraTargetMacText::control(const std::string &value) {
  this->publish_state(value);
  if (this->parent_ != nullptr)
    this->parent_->on_target_mac_set(value);
}

void EbaraMotorSwitch::write_state(bool state) {
  if (this->parent_ != nullptr && this->parent_->request_motor(state)) {
    this->publish_state(state);
  } else {
    // Not sent: re-publish the unchanged state so the UI does not show an
    // action that never happened.
    this->publish_state(this->state);
  }
}

void EbaraEnableSwitch::setup() {
  bool initial = this->get_initial_state_with_restore_mode().value_or(true);
  this->write_state(initial);
}

void EbaraEnableSwitch::write_state(bool state) {
  this->publish_state(state);
  if (this->parent_ != nullptr)
    this->parent_->set_gateway_enabled(state);
}

void EbaraSetpointNumber::control(float value) {
  if (this->parent_ != nullptr && this->parent_->request_setpoint(this->kind_, value)) {
    this->publish_state(value);
  } else if (this->has_state()) {
    this->publish_state(this->state);  // not sent: keep the real value
  }
}

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
  float initial = this->parent_ != nullptr ? this->parent_->get_default_update_interval_s() : 10.0f;
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

// ── Gateway lifecycle ────────────────────────────────────────────────────────
void EbaraHydrostationGateway::setup() {
  global_ebara_instance = this;
  this->mac_pref_ = global_preferences->make_preference<PersistedMac>(fnv1_hash("ebara_hydrostation_target_mac"));
  this->static_data_pref_ =
      global_preferences->make_preference<PersistedStaticData>(fnv1_hash("ebara_hydrostation_static_data"));
  if (!this->static_data_pref_.load(&this->static_data_))
    this->static_data_ = PersistedStaticData{};
  // Whatever is already known is published at boot without any BLE traffic.
  const auto &sd = this->static_data_;
  if (sd.have_firmware && this->firmware_version_text_ != nullptr)
    this->firmware_version_text_->publish_state(format_firmware(sd.firmware_version_raw));
  if (sd.have_hardware && this->hardware_version_sensor_ != nullptr)
    this->hardware_version_sensor_->publish_state(sd.hardware_version);
  if (sd.have_serial && this->serial_number_text_ != nullptr)
    this->serial_number_text_->publish_state(format_serial(sd.serial_number));

  // Same idiom as ESPHome's own esp32_ble component; no erase-on-failure
  // fallback (that would wipe ESPHome's persisted preferences).
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

  // LE Secure Connections, Numeric Comparison (the pump is DisplayYesNo and
  // initiates security with AuthReq 0x0D).
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_YESNO;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 1;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  ble_store_config_init();
  nimble_port_freertos_init(&host_task_trampoline);
  this->set_gw_status_text_("Starting BLE stack...");
  // PollingComponent starts the poller right after setup(): from now on an
  // interval change must restart it.
  this->poller_ready_ = true;
}

void EbaraHydrostationGateway::loop() {
  // All NimBLE events are handled here, in the ESPHome main task.
  this->run_deferred_();
}

void EbaraHydrostationGateway::defer_(std::function<void()> &&fn) {
  std::lock_guard<std::mutex> lock(this->deferred_mutex_);
  this->deferred_.push_back(std::move(fn));
}

void EbaraHydrostationGateway::run_deferred_() {
  std::vector<std::function<void()>> batch;
  {
    std::lock_guard<std::mutex> lock(this->deferred_mutex_);
    if (this->deferred_.empty())
      return;
    batch.swap(this->deferred_);
  }
  for (auto &fn : batch)
    fn();
}

bool EbaraHydrostationGateway::can_connect_now_() const {
  if (!this->gateway_enabled_ || !this->have_target_addr_ || this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE)
    return false;
  if (this->state_ != GwState::IDLE && this->state_ != GwState::COOLDOWN)
    return false;
  return !this->ever_disconnected_ || time_reached(millis(), this->last_disconnect_ms_ + kReconnectSpacingMs);
}

void EbaraHydrostationGateway::update() {
  if (!this->gateway_enabled_ || this->state_ == GwState::UNSUPPORTED)
    return;
  if (this->state_ == GwState::POLL_RUNNING) {
    // One cycle at a time: if the previous one is still running (slow
    // replies, SETs in between) this tick is skipped.
    if (this->cmd_queue_.empty() && this->set_queue_.empty() && !this->cmd_pending_)
      this->enqueue_poll_cycle_();
    return;
  }
  if (this->can_connect_now_())
    this->start_connect_();
}

void EbaraHydrostationGateway::dump_config() {
  ESP_LOGCONFIG(TAG, "Ebara Hydrostation Gateway %s (NimBLE, HYDRO family only):", COMPONENT_VERSION);
  ESP_LOGCONFIG(TAG, "  State: %s", state_name_(this->state_));
  ESP_LOGCONFIG(TAG, "  Gateway enabled: %s", YESNO(this->gateway_enabled_));
  if (this->have_target_addr_) {
    const uint8_t *v = this->target_addr_.val;
    ESP_LOGCONFIG(TAG, "  Target MAC: %02X:%02X:%02X:%02X:%02X:%02X", v[5], v[4], v[3], v[2], v[1], v[0]);
  } else {
    ESP_LOGCONFIG(TAG, "  Target MAC: (not set — scanning)");
  }
  ESP_LOGCONFIG(TAG, "  Live poll interval: %" PRIu32 " ms, slow values every %" PRIu32 " ms",
                this->get_update_interval(), kSlowPollIntervalMs);
  ESP_LOGCONFIG(TAG, "  Static data cached: fw=%s hw=%s serial=%s", YESNO(this->static_data_.have_firmware),
                YESNO(this->static_data_.have_hardware), YESNO(this->static_data_.have_serial));
}

void EbaraHydrostationGateway::on_shutdown() {
  if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG, "Shutting down — closing the BLE link to the pump.");
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
    // Give the controller a few connection events to send LL_TERMINATE_IND.
    delay(150);
  }
}

void EbaraHydrostationGateway::handle_host_reset(int reason) { ESP_LOGW(TAG, "NimBLE host reset; reason=%d", reason); }

void EbaraHydrostationGateway::handle_host_sync() {
  this->defer_([this]() { this->host_synced_(); });
}

void EbaraHydrostationGateway::host_synced_() {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: rc=%d", rc);
    return;
  }
  // Own address: a STATIC random address generated once and kept in NVS. The
  // module forwards UART data only to a central using a random own address,
  // and the address must be stable across reboots because the pump keys its
  // bond on it.
  this->own_addr_pref_ =
      global_preferences->make_preference<PersistedOwnAddr>(fnv1_hash("ebara_hydrostation_own_static_addr"));
  PersistedOwnAddr oa{};
  if (!(this->own_addr_pref_.load(&oa) && oa.valid)) {
    oa = PersistedOwnAddr{};
    ble_addr_t gen{};
    rc = ble_hs_id_gen_rnd(0 /* static random, not NRPA */, &gen);
    if (rc == 0) {
      memcpy(oa.addr, gen.val, sizeof(oa.addr));
      oa.valid = true;
      this->own_addr_pref_.save(&oa);
      // Commit now: losing it to a power cut before the periodic flush would
      // orphan the bond the pump is about to store for this address.
      global_preferences->sync();
      ESP_LOGI(TAG, "Generated a new static random own address (persisted).");
    } else {
      ESP_LOGW(TAG, "ble_hs_id_gen_rnd failed: rc=%d", rc);
    }
  }
  if (oa.valid && ble_hs_id_set_rnd(oa.addr) == 0) {
    this->use_random_own_addr_ = true;
    ESP_LOGI(TAG, "Own BLE address: %02X:%02X:%02X:%02X:%02X:%02X (static random, stable across reboots)",
             oa.addr[5], oa.addr[4], oa.addr[3], oa.addr[2], oa.addr[1], oa.addr[0]);
  } else {
    ESP_LOGW(TAG, "Could not set the static random address — falling back to the public address "
                  "(the pump will not forward data to it).");
  }
  ESP_LOGI(TAG, "NimBLE host synced and ready.");
  this->set_gw_status_text_("BLE ready");

  if (!this->have_target_addr_) {
    // The MAC chosen at runtime (persisted) wins over the YAML default, which
    // only seeds a device that has never been configured.
    PersistedMac pm{};
    if (this->mac_pref_.load(&pm) && pm.valid) {
      char mac_str[18];
      snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", pm.mac[5], pm.mac[4], pm.mac[3],
               pm.mac[2], pm.mac[1], pm.mac[0]);
      ESP_LOGI(TAG, "Restored target MAC from flash: %s", mac_str);
      if (this->target_mac_text_ != nullptr)
        this->target_mac_text_->publish_state(mac_str);
      this->on_target_mac_set(mac_str);
      return;
    }
    if (!this->default_target_mac_.empty()) {
      ESP_LOGI(TAG, "Applying default_target_mac from YAML: %s", this->default_target_mac_.c_str());
      if (this->target_mac_text_ != nullptr)
        this->target_mac_text_->publish_state(this->default_target_mac_);
      this->on_target_mac_set(this->default_target_mac_);
      return;
    }
  }
  if (this->can_connect_now_()) {
    this->start_connect_();
  } else if (!this->have_target_addr_) {
    this->start_scan_();
  }
}

void EbaraHydrostationGateway::clear_session_state_() {
  this->cmd_pending_ = false;
  this->cmd_queue_.clear();
  this->set_queue_.clear();
  this->rx_buf_.clear();
  this->consecutive_timeouts_ = 0;
  this->cccd_retry_pending_ = false;
  this->cancel_timeout("cmd_timeout");
  this->cancel_timeout("intercmd_delay");
  this->cancel_timeout("subscribe_delay");
  this->cancel_timeout("rx_flush");
  this->cancel_timeout("session_watchdog");
  this->cancel_timeout("boot_connect_delay");
}

void EbaraHydrostationGateway::abort_session_(const char *reason) {
  ESP_LOGI(TAG, "%s", reason);
  const bool was_live = this->state_ == GwState::POLL_RUNNING || this->state_ == GwState::WAIT_BEFORE_POLL;
  this->clear_session_state_();
  if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
  } else if (this->state_ == GwState::CONNECTING) {
    // Connection not established yet: cancel it (no-op if none is pending,
    // e.g. while waiting for the boot delay).
    ble_gap_conn_cancel();
  }
  if (this->state_ != GwState::UNSUPPORTED)
    this->set_state_(GwState::COOLDOWN);
  if (was_live)
    this->invalidate_live_values_();
}

void EbaraHydrostationGateway::on_target_mac_set(const std::string &mac_str) {
  if (mac_str.empty()) {
    ESP_LOGI(TAG, "Target MAC cleared — forgetting persisted MAC and resuming scan.");
    this->abort_session_("Closing the session with the previous pump.");
    this->invalidate_live_values_();
    this->have_target_addr_ = false;
    this->target_addr_ = ble_addr_t{};
    this->bond_confirmed_ = false;
    PersistedMac pm{};
    pm.valid = false;
    this->mac_pref_.save(&pm);
    this->static_data_ = PersistedStaticData{};
    this->save_static_data_();
    if (this->state_ == GwState::UNSUPPORTED)
      this->set_state_(GwState::COOLDOWN);
    if (this->gateway_enabled_ && this->conn_handle_ == BLE_HS_CONN_HANDLE_NONE)
      this->start_scan_();
    // Otherwise the scan starts on DISCONNECT.
    return;
  }

  ble_addr_t addr{};
  if (!parse_mac_address(mac_str, &addr)) {
    ESP_LOGE(TAG, "Invalid MAC address: '%s' (expected AA:BB:CC:DD:EE:FF)", mac_str.c_str());
    return;
  }

  PersistedMac old_pm{};
  const bool had_valid_mac = this->mac_pref_.load(&old_pm) && old_pm.valid;
  const bool mac_changed = !had_valid_mac || memcmp(addr.val, old_pm.mac, sizeof(addr.val)) != 0;
  // Same MAC re-entered while already configured: nothing to do.
  if (!mac_changed && this->have_target_addr_)
    return;

  this->stop_scan_();
  if (mac_changed && this->have_target_addr_) {
    // A session with the previous pump (or an attempt) must not continue.
    this->abort_session_("Target MAC changed — closing the session with the previous pump.");
  }
  this->target_addr_ = addr;
  this->have_target_addr_ = true;
  PersistedMac pm{};
  memcpy(pm.mac, addr.val, sizeof(pm.mac));
  pm.valid = true;
  this->mac_pref_.save(&pm);
  if (mac_changed) {
    this->static_data_ = PersistedStaticData{};
    this->save_static_data_();
    this->status_known_ = false;
    this->bond_confirmed_ = false;
    // A different pump: a previous "unsupported device" verdict no longer applies.
    if (this->state_ == GwState::UNSUPPORTED)
      this->set_state_(GwState::COOLDOWN);
  }
  ESP_LOGI(TAG, "Target MAC set: %s (saved to flash)", mac_str.c_str());
  if (this->can_connect_now_())
    this->start_connect_();
  // Otherwise update() connects once the previous link is gone.
}

void EbaraHydrostationGateway::set_gateway_enabled(bool enabled) {
  if (enabled == this->gateway_enabled_)
    return;
  this->gateway_enabled_ = enabled;

  if (!enabled) {
    this->stop_scan_();
    this->abort_session_("Gateway disabled — disconnecting.");
    this->set_gw_status_text_("Disabled");
    this->invalidate_live_values_();
    return;
  }

  ESP_LOGI(TAG, "Gateway enabled — resuming.");
  if (this->can_connect_now_()) {
    this->start_connect_();
  } else if (!this->have_target_addr_ && this->use_random_own_addr_) {
    this->start_scan_();
  }
}

void EbaraHydrostationGateway::set_update_interval_seconds(float seconds) {
  if (seconds < kUpdateIntervalMinS)
    seconds = kUpdateIntervalMinS;
  else if (seconds > kUpdateIntervalMaxS)
    seconds = kUpdateIntervalMaxS;
  this->set_update_interval(static_cast<uint32_t>(seconds * 1000));
  // set_update_interval() only stores the value: restart the running poller
  // so that a change made from Home Assistant takes effect immediately.
  if (this->poller_ready_) {
    this->stop_poller();
    this->start_poller();
  }
  ESP_LOGI(TAG, "Update interval set to %.0fs", seconds);
}

// ── User commands ────────────────────────────────────────────────────────────
bool EbaraHydrostationGateway::request_motor(bool enable) {
  if (this->state_ != GwState::POLL_RUNNING || !this->status_known_) {
    ESP_LOGW(TAG, "Motor command ignored: pump not connected or status not yet known.");
    return false;
  }
  if (this->status_word_ & 0x8000) {
    // Official app (legacy/HYDRO mode): the power button sends nothing while
    // an alarm is active.
    ESP_LOGW(TAG, "Motor command ignored: an alarm is active on the pump (status bit 15).");
    return false;
  }
  // SET commands end with ';' (app 2.0.10015, CtrlHydroFragment).
  return this->enqueue_set_command_(enable ? "sm-0005:1;" : "sm-0005:0;");
}

bool EbaraHydrostationGateway::request_setpoint(EbaraSetpointKind kind, float value_bar) {
  if (this->state_ != GwState::POLL_RUNNING || !this->status_known_) {
    ESP_LOGW(TAG, "Setpoint ignored: pump not connected or status (unit) not yet known.");
    return false;
  }
  if (std::isnan(value_bar)) {
    ESP_LOGW(TAG, "Setpoint ignored: value is not a number.");
    return false;
  }
  int raw;
  const char *prefix = (kind == EbaraSetpointKind::TARGET_PRESSURE) ? "sm-0001:" : "sm-0012:";
  if (!this->psi_mode_) {
    // bar×10, step 1 (0.1 bar). HYDRO limits from the app: target 20..55,
    // delta 5..20.
    raw = static_cast<int>(std::lround(value_bar * 10.0f));
    if (kind == EbaraSetpointKind::TARGET_PRESSURE)
      raw = std::max(20, std::min(55, raw));
    else
      raw = std::max(5, std::min(20, raw));
  } else {
    // psi×10, step 20 (2 psi). HYDRO limits from the app: target 300..800,
    // delta 80..300.
    float psi10 = value_bar * kPsiPerBar * 10.0f;
    raw = static_cast<int>(std::lround(psi10 / 20.0f)) * 20;
    if (kind == EbaraSetpointKind::TARGET_PRESSURE)
      raw = std::max(300, std::min(800, raw));
    else
      raw = std::max(80, std::min(300, raw));
  }
  return this->enqueue_set_command_(prefix + std::to_string(raw) + ";");
}

bool EbaraHydrostationGateway::enqueue_set_command_(const std::string &cmd) {
  if (this->state_ != GwState::POLL_RUNNING) {
    ESP_LOGW(TAG, "Cannot send '%s': gateway is not polling (state=%s)", cmd.c_str(), state_name_(this->state_));
    return false;
  }
  // FIFO among SETs, one entry per register ("sm-NNNN:"): a newer value for
  // the same register replaces the queued one instead of sending both.
  const std::string reg = cmd.substr(0, cmd.find(':') + 1);
  for (auto &queued : this->set_queue_) {
    if (queued.compare(0, reg.size(), reg) == 0) {
      ESP_LOGI(TAG, "Replacing queued SET '%s' with '%s'", queued.c_str(), cmd.c_str());
      queued = cmd;
      return true;
    }
  }
  ESP_LOGI(TAG, "Queueing SET command '%s'", cmd.c_str());
  this->set_queue_.push_back(cmd);
  this->pump_command_queue_();
  return true;
}

// ── Connection ───────────────────────────────────────────────────────────────
void EbaraHydrostationGateway::start_connect_() {
  if (!this->have_target_addr_ || !this->gateway_enabled_) {
    ESP_LOGW(TAG, "start_connect_: gateway disabled or no target MAC configured yet");
    return;
  }
  if (!this->use_random_own_addr_) {
    // Without the static random own address the module accepts the link but
    // never forwards UART data: do not even try.
    ESP_LOGE(TAG, "No static random own address available — not connecting.");
    this->set_gw_status_text_("BLE address error");
    return;
  }
  this->clear_session_state_();
  this->connect_purpose_ = this->bond_confirmed_ ? ConnectPurpose::SERVICE_SESSION : ConnectPurpose::INITIAL_BOND;
  this->set_state_(GwState::CONNECTING);
  const uint32_t now = millis();
  if (now < kBootConnectDelayMs) {
    const uint32_t wait = kBootConnectDelayMs - now;
    ESP_LOGI(TAG, "Waiting %" PRIu32 " ms before the first connection (module supervision timeout).", wait);
    this->set_gw_status_text_("Waiting before connecting...");
    this->set_timeout("boot_connect_delay", wait, [this]() {
      if (this->state_ != GwState::CONNECTING || this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE)
        return;
      this->set_state_(GwState::IDLE);
      if (this->can_connect_now_())
        this->start_connect_();
    });
    return;
  }
  this->passkey_seen_this_connection_ = false;
  this->cccd_retry_done_ = false;
  this->set_gw_status_text_("Connecting...");

  // Tight connection parameters from the first connection event (the module
  // starts its own GATT client queries immediately after connecting).
  struct ble_gap_conn_params conn_params {};
  conn_params.scan_itvl = 96;
  conn_params.scan_window = 96;
  conn_params.itvl_min = 12;
  conn_params.itvl_max = 24;
  conn_params.latency = 0;
  conn_params.supervision_timeout = 72;
  conn_params.min_ce_len = 0;
  conn_params.max_ce_len = 0;
  int rc = ble_gap_connect(BLE_OWN_ADDR_RANDOM, &this->target_addr_, 15000, &conn_params, &gap_event_trampoline, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_connect failed: rc=%d", rc);
    this->set_state_(GwState::COOLDOWN);
    return;
  }
  // Session setup watchdog: cancelled when polling starts.
  this->set_timeout("session_watchdog", kSessionSetupTimeoutMs, [this]() {
    ESP_LOGW(TAG, "Session setup not completed within %" PRIu32 " ms (state=%s) — aborting and retrying.",
             kSessionSetupTimeoutMs, state_name_(this->state_));
    this->abort_session_("Session setup timeout.");
    this->set_gw_status_text_("Setup timeout — retrying...");
  });
}

// ── BLE scan (pre-configuration only) ────────────────────────────────────────
void EbaraHydrostationGateway::start_scan_() {
  if (this->scanning_ || !this->gateway_enabled_ || this->have_target_addr_)
    return;
  if (!this->use_random_own_addr_) {
    ESP_LOGE(TAG, "No static random own address available — not scanning.");
    return;
  }
  struct ble_gap_disc_params disc_params {};
  disc_params.passive = 0;
  disc_params.filter_duplicates = 1;
  int rc = ble_gap_disc(BLE_OWN_ADDR_RANDOM, BLE_HS_FOREVER, &disc_params, &gap_event_trampoline, this);
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
  ESP_LOGI(TAG, "Still scanning for Hydrostations... (%u found so far)",
           static_cast<unsigned>(this->discovered_hydros_.size()));
  this->set_timeout("scan_heartbeat", 10000, [this]() { this->scan_heartbeat_(); });
}

void EbaraHydrostationGateway::stop_scan_() {
  if (!this->scanning_)
    return;
  ble_gap_disc_cancel();
  this->scanning_ = false;
  this->cancel_timeout("scan_heartbeat");
  ESP_LOGI(TAG, "Stopped scanning for Hydrostations (%u found).",
           static_cast<unsigned>(this->discovered_hydros_.size()));
}

void EbaraHydrostationGateway::on_scan_complete_() {
  // The controller ended the scan on its own (not via stop_scan_()).
  if (!this->scanning_)
    return;
  this->scanning_ = false;
  this->cancel_timeout("scan_heartbeat");
  ESP_LOGW(TAG, "Scan ended unexpectedly — restarting it.");
  this->start_scan_();
}

void EbaraHydrostationGateway::publish_discovered_() {
  if (this->discovered_text_ == nullptr)
    return;
  std::string json = "[";
  for (size_t i = 0; i < this->discovered_hydros_.size(); i++) {
    const auto &h = this->discovered_hydros_[i];
    if (i > 0)
      json += ",";
    json += "{\"mac\":\"" + h.mac + "\",\"name\":\"" + json_escape(h.name) + "\",\"rssi\":" + std::to_string(h.rssi) +
            ",\"supported\":" + (h.supported ? "true" : "false") + "}";
  }
  json += "]";
  ESP_LOGI(TAG, "Discovered Hydrostations entity updated: %s", json.c_str());
  this->discovered_text_->publish_state(json);
}

void EbaraHydrostationGateway::fail_discovery_(const char *reason) {
  ESP_LOGE(TAG, "%s", reason);
  this->clear_session_state_();
  if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE)
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
  this->set_state_(GwState::COOLDOWN);
}

void EbaraHydrostationGateway::mark_unsupported_(const std::string &name) {
  ESP_LOGE(TAG, "Unsupported device '%s': only Ebara HYDRO pumps are supported (HYSTA uses a different protocol).",
           name.c_str());
  this->clear_session_state_();
  this->set_state_(GwState::UNSUPPORTED);
  this->set_gw_status_text_("Unsupported device: " + name + " (only HYDRO supported)");
  this->invalidate_live_values_();
  if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE)
    ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
}

// NimBLE host task: copy what is needed and defer to the main loop. Only the
// events that need a synchronous return value are handled inline, and those
// make no ESPHome calls.
int EbaraHydrostationGateway::handle_gap_event(struct ble_gap_event *event) {
  switch (event->type) {
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0)
        ble_store_util_delete_peer(&desc.peer_id_addr);
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
      const uint16_t conn = event->notify_rx.conn_handle;
      const uint16_t handle = event->notify_rx.attr_handle;
      uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      if (len == 0 || len > 256)
        return 0;
      std::string data(len, '\0');
      os_mbuf_copydata(event->notify_rx.om, 0, len, &data[0]);
      this->defer_([this, conn, handle, data]() {
        if (this->is_current_conn_(conn) && this->tx_val_handle_ != 0 && handle == this->tx_val_handle_)
          this->on_notification_(reinterpret_cast<const uint8_t *>(data.data()), data.size());
      });
      return 0;
    }
    case BLE_GAP_EVENT_DISC: {
      // Pre-filter here: only Ebara adverts reach the main loop, and never
      // more than kMaxDeferredAdv pending events.
      const uint8_t *d = event->disc.data;
      const size_t n = event->disc.length_data;
      if (!adv_contains(d, n, "HYDRO") && !adv_contains(d, n, "HYSTA"))
        return 0;
      {
        std::lock_guard<std::mutex> lock(this->deferred_mutex_);
        if (this->deferred_.size() >= kMaxDeferredAdv)
          return 0;
      }
      ble_addr_t addr = event->disc.addr;
      int8_t rssi = event->disc.rssi;
      std::vector<uint8_t> adv(d, d + n);
      this->defer_([this, addr, rssi, adv]() { this->process_disc_(addr, rssi, adv); });
      return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
      this->defer_([this]() { this->on_scan_complete_(); });
      return 0;
    case BLE_GAP_EVENT_L2CAP_UPDATE_REQ: {
      // Accept (NimBLE applies the peer's request itself); log from the loop.
      struct ble_gap_upd_params p = *event->conn_update_req.peer_params;
      this->defer_([p]() {
        ESP_LOGD(TAG, "L2CAP conn-param update requested by peer: itvl_min=%u itvl_max=%u latency=%u timeout=%u",
                 p.itvl_min, p.itvl_max, p.latency, p.supervision_timeout);
      });
      return 0;
    }
    case BLE_GAP_EVENT_CONNECT:
    case BLE_GAP_EVENT_DISCONNECT:
    case BLE_GAP_EVENT_ENC_CHANGE:
    case BLE_GAP_EVENT_PASSKEY_ACTION:
    case BLE_GAP_EVENT_MTU:
    case BLE_GAP_EVENT_CONN_UPDATE: {
      // These union members hold plain values only: a by-value copy is safe.
      struct ble_gap_event copy = *event;
      this->defer_([this, copy]() mutable { this->process_gap_event_(&copy); });
      return 0;
    }
    default:
      return 0;
  }
}

void EbaraHydrostationGateway::process_gap_event_(struct ble_gap_event *event) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
      if (event->connect.status != 0) {
        ESP_LOGW(TAG, "Connection attempt failed: status=%d", event->connect.status);
        this->cancel_timeout("session_watchdog");
        if (this->state_ == GwState::CONNECTING)
          this->set_state_(GwState::COOLDOWN);
        if (this->gateway_enabled_)
          this->set_gw_status_text_("Connect failed");
        return;
      }
      this->conn_handle_ = event->connect.conn_handle;
      if (!this->gateway_enabled_ || this->state_ != GwState::CONNECTING) {
        // The attempt was aborted (disable, MAC change, watchdog) but the link
        // came up before the cancel took effect: close it.
        ESP_LOGI(TAG, "Connected after the attempt was aborted (state=%s) — closing the link.",
                 state_name_(this->state_));
        ble_gap_terminate(this->conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
        return;
      }
      ESP_LOGI(TAG, "Connected (conn_handle=%u), purpose=%s", this->conn_handle_,
               this->connect_purpose_ == ConnectPurpose::INITIAL_BOND ? "initial" : "bonded session");
      if (this->connect_purpose_ == ConnectPurpose::INITIAL_BOND) {
        // No bond confirmed yet: discover first; pairing (or encryption with a
        // stored LTK) is triggered by the pump's own Security Request / the
        // rejected unencrypted CCCD write.
        this->begin_service_discovery_();
        return;
      }
      // A bond exists. The pump sends its own Security Request right after
      // connecting, to which NimBLE may already have responded by starting
      // encryption with the stored LTK. A second encryption start on the same
      // link ends in an LL MIC failure (0x3D): only initiate if nothing is in
      // progress yet.
      this->set_state_(GwState::ENCRYPTING);
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(this->conn_handle_, &desc) == 0 && desc.sec_state.encrypted) {
        ESP_LOGD(TAG, "Link already encrypted — not re-initiating security.");
        this->on_link_encrypted_();
        return;
      }
      int rc = ble_gap_security_initiate(this->conn_handle_);
      if (rc == BLE_HS_EALREADY) {
        ESP_LOGD(TAG, "Security procedure already in progress — waiting for it.");
      } else if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_security_initiate failed: rc=%d", rc);
        this->fail_discovery_("Could not start encryption");
      }
      return;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
      const uint16_t handle = event->disconnect.conn.conn_handle;
      if (this->conn_handle_ != BLE_HS_CONN_HANDLE_NONE && handle != this->conn_handle_) {
        ESP_LOGD(TAG, "Ignoring disconnect of a stale connection (handle=%u).", handle);
        return;
      }
      ESP_LOGI(TAG, "Disconnected; reason=%d (state=%s)", event->disconnect.reason, state_name_(this->state_));
      const GwState prev = this->state_;
      this->conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
      this->last_disconnect_ms_ = millis();
      this->ever_disconnected_ = true;
      // Same teardown on every path (expected or not).
      this->clear_session_state_();
      if (prev == GwState::UNSUPPORTED)
        return;  // stay idle; the status text already explains why
      if (prev != GwState::COOLDOWN) {
        ESP_LOGW(TAG, "Unexpected disconnect while in state %s", state_name_(prev));
        if (prev == GwState::POLL_RUNNING || prev == GwState::WAIT_BEFORE_POLL)
          this->invalidate_live_values_();
        this->set_gw_status_text_("Disconnected");
      }
      this->set_state_(GwState::COOLDOWN);
      if (!this->gateway_enabled_)
        this->set_gw_status_text_("Disabled");
      else if (!this->have_target_addr_)
        this->start_scan_();
      return;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
      if (!this->is_current_conn_(event->enc_change.conn_handle))
        return;
      if (event->enc_change.status != 0) {
        ESP_LOGE(TAG, "Encryption failed: status=%d", event->enc_change.status);
        if (event->enc_change.status == kHciErrMicFailure) {
          // Link-layer integrity error (e.g. a duplicated encryption start):
          // says nothing about the validity of the stored keys — keep them.
          ESP_LOGW(TAG, "MIC failure — keeping the stored bond and retrying.");
        } else {
          // The pump rejected/ignored our stored LTK: it no longer knows this
          // bond. Forget it locally and pair again next time. The pump uses
          // its public address as identity, i.e. the target MAC.
          struct ble_gap_conn_desc desc {};
          ble_addr_t peer = this->target_addr_;
          if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0)
            peer = desc.peer_id_addr;
          ble_store_util_delete_peer(&peer);
          ESP_LOGW(TAG, "Deleted local bond for this peer — will retry as a fresh pairing.");
          this->bond_confirmed_ = false;
        }
        this->fail_discovery_("Encryption failed");
        return;
      }
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0)
        ESP_LOGI(TAG, "Encrypted: authenticated=%d bonded=%d", desc.sec_state.authenticated, desc.sec_state.bonded);
      if (this->connect_purpose_ == ConnectPurpose::INITIAL_BOND) {
        if (this->state_ == GwState::WRITE_CCCD && this->cccd_retry_pending_) {
          ESP_LOGD(TAG, "Encrypted after reactive pairing — retrying CCCD write.");
          this->cccd_retry_pending_ = false;
          this->cccd_retry_done_ = true;
          this->write_cccd_();
        }
        // Otherwise discovery is in flight and the CCCD write will succeed.
        return;
      }
      if (this->state_ == GwState::ENCRYPTING)
        this->on_link_encrypted_();
      return;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
      if (!this->is_current_conn_(event->passkey.conn_handle))
        return;
      this->passkey_seen_this_connection_ = true;
      if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
        ESP_LOGI(TAG, "Numeric comparison passkey: %06" PRIu32 " — auto-confirming.", event->passkey.params.numcmp);
        struct ble_sm_io pkey {};
        pkey.action = BLE_SM_IOACT_NUMCMP;
        pkey.numcmp_accept = 1;
        int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        if (rc != 0)
          ESP_LOGE(TAG, "ble_sm_inject_io failed: rc=%d", rc);
      } else {
        ESP_LOGW(TAG, "Unexpected passkey action=%d", event->passkey.params.action);
      }
      return;
    }

    case BLE_GAP_EVENT_MTU:
      ESP_LOGD(TAG, "MTU update: conn_handle=%u mtu=%u", event->mtu.conn_handle, event->mtu.value);
      return;

    case BLE_GAP_EVENT_CONN_UPDATE: {
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
        ESP_LOGD(TAG, "Conn params updated: status=%d itvl=%u (%.2fms) latency=%u timeout=%u (%ums)",
                 event->conn_update.status, desc.conn_itvl, desc.conn_itvl * 1.25, desc.conn_latency,
                 desc.supervision_timeout, desc.supervision_timeout * 10);
      }
      return;
    }

    default:
      return;
  }
}

void EbaraHydrostationGateway::process_disc_(ble_addr_t addr, int8_t rssi, const std::vector<uint8_t> &adv) {
  if (!this->scanning_)
    return;
  struct ble_hs_adv_fields fields {};
  if (ble_hs_adv_parse_fields(&fields, adv.data(), adv.size()) != 0)
    return;
  if (fields.name == nullptr || fields.name_len == 0)
    return;
  char name_buf[24];
  size_t name_len = std::min<size_t>(fields.name_len, sizeof(name_buf) - 1);
  memcpy(name_buf, fields.name, name_len);
  name_buf[name_len] = '\0';
  // HYDRO_xxxx: supported. HYSTA_xxxx: Ebara Hydrostation of the other
  // family — listed, but flagged as not supported.
  bool is_hydro = strncmp(name_buf, "HYDRO", 5) == 0;
  bool is_hysta = strncmp(name_buf, "HYSTA", 5) == 0;
  if (!is_hydro && !is_hysta)
    return;

  char mac_buf[18];
  const uint8_t *v = addr.val;
  snprintf(mac_buf, sizeof(mac_buf), "%02X:%02X:%02X:%02X:%02X:%02X", v[5], v[4], v[3], v[2], v[1], v[0]);

  for (auto &h : this->discovered_hydros_) {
    if (h.mac == mac_buf) {
      h.rssi = rssi;
      this->publish_discovered_();
      return;
    }
  }
  if (this->discovered_hydros_.size() >= 10)
    return;
  this->discovered_hydros_.push_back({mac_buf, name_buf, rssi, is_hydro});
  if (is_hydro)
    ESP_LOGI(TAG, "Discovered Hydrostation: %s (%s, %d dBm)", name_buf, mac_buf, rssi);
  else
    ESP_LOGW(TAG, "Discovered %s (%s): HYSTA family is not supported by this gateway.", name_buf, mac_buf);
  this->publish_discovered_();
}

void EbaraHydrostationGateway::on_link_encrypted_() {
  if (this->state_ != GwState::ENCRYPTING)
    return;  // already handled
  if (this->connect_purpose_ != ConnectPurpose::SERVICE_SESSION)
    return;
  if (this->passkey_seen_this_connection_)
    ESP_LOGW(TAG, "A new pairing was needed — the previous bond was not reused.");
  this->begin_service_discovery_();
}

// ── GATT discovery: all services → all chars → all descriptors → device name
//    check → CCCD write ─────────────────────────────────────────────────────
void EbaraHydrostationGateway::begin_service_discovery_() {
  this->set_state_(GwState::DISCOVER_SVC);
  this->tx_val_handle_ = this->cccd_handle_ = this->rx_val_handle_ = 0;
  this->name_val_handle_ = 0;
  this->disc_services_.clear();
  this->disc_svc_idx_ = 0;
  this->disc_chrs_.clear();
  this->disc_chr_idx_ = 0;
  int rc = ble_gattc_disc_all_svcs(this->conn_handle_, &disc_svc_cb_, this);
  if (rc != 0)
    this->fail_discovery_("ble_gattc_disc_all_svcs failed");
}

int EbaraHydrostationGateway::disc_svc_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                            const struct ble_gatt_svc *service, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  struct ble_gatt_svc copy {};
  if (service != nullptr)
    copy = *service;
  const int st = error->status;
  self->defer_([self, conn_handle, st, copy]() { self->on_disc_svc_(conn_handle, st, copy); });
  return 0;
}

void EbaraHydrostationGateway::on_disc_svc_(uint16_t conn, int status, struct ble_gatt_svc svc) {
  if (!this->is_current_conn_(conn) || this->state_ != GwState::DISCOVER_SVC)
    return;
  switch (status) {
    case 0:
      this->disc_services_.push_back({svc.start_handle, svc.end_handle});
      return;
    case BLE_HS_EDONE: {
      if (this->disc_services_.empty()) {
        this->fail_discovery_("No services found on peer at all");
        return;
      }
      ESP_LOGD(TAG, "Discovered %u services — discovering characteristics for each",
               static_cast<unsigned>(this->disc_services_.size()));
      this->set_state_(GwState::DISCOVER_CHR);
      this->disc_svc_idx_ = 0;
      this->disc_chrs_svc_start_idx_ = this->disc_chrs_.size();
      const auto &svc = this->disc_services_[0];
      int rc = ble_gattc_disc_all_chrs(this->conn_handle_, svc.start_handle, svc.end_handle, &disc_chr_cb_, this);
      if (rc != 0)
        this->fail_discovery_("ble_gattc_disc_all_chrs failed");
      return;
    }
    default:
      this->fail_discovery_("Service discovery error");
      return;
  }
}

int EbaraHydrostationGateway::disc_chr_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                            const struct ble_gatt_chr *chr, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  struct ble_gatt_chr copy {};
  if (chr != nullptr)
    copy = *chr;
  const int st = error->status;
  self->defer_([self, conn_handle, st, copy]() { self->on_disc_chr_(conn_handle, st, copy); });
  return 0;
}

void EbaraHydrostationGateway::on_disc_chr_(uint16_t conn, int status, struct ble_gatt_chr chr) {
  if (!this->is_current_conn_(conn) || this->state_ != GwState::DISCOVER_CHR)
    return;
  switch (status) {
    case 0: {
      if (ble_uuid_cmp(&chr.uuid.u, (const ble_uuid_t *) &kTxNotifyUuid) == 0) {
        this->tx_val_handle_ = chr.val_handle;
      } else if (ble_uuid_cmp(&chr.uuid.u, (const ble_uuid_t *) &kRxWriteUuid) == 0) {
        this->rx_val_handle_ = chr.val_handle;
      } else if (ble_uuid_cmp(&chr.uuid.u, (const ble_uuid_t *) &kDeviceNameUuid) == 0) {
        this->name_val_handle_ = chr.val_handle;
      }
      if (this->disc_chrs_.size() > this->disc_chrs_svc_start_idx_)
        this->disc_chrs_.back().dsc_range_end = chr.def_handle - 1;
      const auto &svc = this->disc_services_[this->disc_svc_idx_];
      this->disc_chrs_.push_back({chr.val_handle, svc.end_handle});
      return;
    }
    case BLE_HS_EDONE: {
      this->disc_svc_idx_++;
      if (this->disc_svc_idx_ < this->disc_services_.size()) {
        this->disc_chrs_svc_start_idx_ = this->disc_chrs_.size();
        const auto &svc = this->disc_services_[this->disc_svc_idx_];
        int rc = ble_gattc_disc_all_chrs(this->conn_handle_, svc.start_handle, svc.end_handle, &disc_chr_cb_, this);
        if (rc != 0)
          this->fail_discovery_("ble_gattc_disc_all_chrs failed");
        return;
      }
      if (this->tx_val_handle_ == 0 || this->rx_val_handle_ == 0) {
        this->fail_discovery_("TX/RX characteristic not found");
        return;
      }
      ESP_LOGD(TAG, "Discovered %u characteristics across all services — discovering descriptors for each",
               static_cast<unsigned>(this->disc_chrs_.size()));
      this->set_state_(GwState::DISCOVER_CCCD);
      this->disc_chr_idx_ = 0;
      while (this->disc_chr_idx_ < this->disc_chrs_.size() &&
             this->disc_chrs_[this->disc_chr_idx_].dsc_range_end <
                 static_cast<uint16_t>(this->disc_chrs_[this->disc_chr_idx_].val_handle + 1)) {
        this->disc_chr_idx_++;
      }
      if (this->disc_chr_idx_ >= this->disc_chrs_.size()) {
        this->fail_discovery_("CCCD for TX characteristic not found");
        return;
      }
      // NimBLE applies its own +1 to start_handle in ble_gattc_disc_all_dscs():
      // pass val_handle unshifted.
      const auto &c = this->disc_chrs_[this->disc_chr_idx_];
      int rc = ble_gattc_disc_all_dscs(this->conn_handle_, c.val_handle, c.dsc_range_end, &disc_cccd_cb_, this);
      if (rc != 0)
        this->fail_discovery_("ble_gattc_disc_all_dscs failed");
      return;
    }
    default:
      this->fail_discovery_("Characteristic discovery error");
      return;
  }
}

int EbaraHydrostationGateway::disc_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                             uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  struct ble_gatt_dsc copy {};
  if (dsc != nullptr)
    copy = *dsc;
  const int st = error->status;
  self->defer_([self, conn_handle, st, chr_val_handle, copy]() {
    self->on_disc_dsc_(conn_handle, st, chr_val_handle, copy);
  });
  return 0;
}

void EbaraHydrostationGateway::on_disc_dsc_(uint16_t conn, int status, uint16_t chr_val_handle,
                                            struct ble_gatt_dsc dsc) {
  if (!this->is_current_conn_(conn) || this->state_ != GwState::DISCOVER_CCCD)
    return;
  switch (status) {
    case 0:
      // Only the TX characteristic's CCCD (0x0053) — not the CCCD of the
      // third, unused 4c8a characteristic (0x0058), which is writable without
      // encryption and so says nothing about the bond.
      if (chr_val_handle == this->tx_val_handle_ && ble_uuid_cmp(&dsc.uuid.u, (const ble_uuid_t *) &kCccdUuid) == 0)
        this->cccd_handle_ = dsc.handle;
      return;
    case BLE_HS_EDONE: {
      this->disc_chr_idx_++;
      while (this->disc_chr_idx_ < this->disc_chrs_.size() &&
             this->disc_chrs_[this->disc_chr_idx_].dsc_range_end <
                 static_cast<uint16_t>(this->disc_chrs_[this->disc_chr_idx_].val_handle + 1)) {
        this->disc_chr_idx_++;
      }
      if (this->disc_chr_idx_ < this->disc_chrs_.size()) {
        const auto &c = this->disc_chrs_[this->disc_chr_idx_];
        int rc = ble_gattc_disc_all_dscs(this->conn_handle_, c.val_handle, c.dsc_range_end, &disc_cccd_cb_, this);
        if (rc != 0)
          this->fail_discovery_("ble_gattc_disc_all_dscs failed");
        return;
      }
      if (this->cccd_handle_ == 0) {
        this->fail_discovery_("CCCD for TX characteristic not found");
        return;
      }
      ESP_LOGD(TAG, "Full GATT database discovery complete");
      this->read_device_name_();
      return;
    }
    default:
      this->fail_discovery_("Descriptor discovery error");
      return;
  }
}

void EbaraHydrostationGateway::read_device_name_() {
  if (this->name_val_handle_ == 0) {
    this->fail_discovery_("GAP Device Name characteristic not found — cannot verify product family");
    return;
  }
  this->set_state_(GwState::READ_DEVICE_NAME);
  int rc = ble_gattc_read(this->conn_handle_, this->name_val_handle_, &read_name_cb_, this);
  if (rc != 0)
    this->fail_discovery_("Device name read failed to start");
}

int EbaraHydrostationGateway::read_name_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                             struct ble_gatt_attr *attr, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  const int st = error->status;
  std::string name;
  if (st == 0 && attr != nullptr) {
    uint16_t len = std::min<uint16_t>(OS_MBUF_PKTLEN(attr->om), 32);
    name.assign(len, '\0');
    os_mbuf_copydata(attr->om, 0, len, &name[0]);
    name = name.c_str();  // stop at an embedded NUL
  }
  self->defer_([self, conn_handle, st, name]() { self->on_read_name_(conn_handle, st, name); });
  return 0;
}

void EbaraHydrostationGateway::on_read_name_(uint16_t conn, int status, const std::string &device_name) {
  if (!this->is_current_conn_(conn) || this->state_ != GwState::READ_DEVICE_NAME)
    return;
  if (status != 0) {
    ESP_LOGE(TAG, "Device name read failed: status=%d", status);
    this->fail_discovery_("Device name read failed");
    return;
  }
  ESP_LOGI(TAG, "Pump device name: '%s'", device_name.c_str());
  if (this->device_name_text_ != nullptr)
    this->device_name_text_->publish_state(device_name);
  if (device_name.rfind("HYDRO", 0) != 0) {
    this->mark_unsupported_(device_name);
    return;
  }
  this->write_cccd_();
}

void EbaraHydrostationGateway::write_cccd_() {
  this->set_state_(GwState::WRITE_CCCD);
  struct ble_gap_conn_desc desc {};
  if (ble_gap_conn_find(this->conn_handle_, &desc) == 0) {
    ESP_LOGD(TAG, "Pre-CCCD-write link state: encrypted=%d authenticated=%d bonded=%d", desc.sec_state.encrypted,
             desc.sec_state.authenticated, desc.sec_state.bonded);
  }
  // 0x0001 enables notifications on the TX characteristic (ATT Write Request).
  uint8_t value[2] = {0x01, 0x00};
  int rc = ble_gattc_write_flat(this->conn_handle_, this->cccd_handle_, value, sizeof(value), &write_cccd_cb_, this);
  if (rc != 0)
    this->fail_discovery_("CCCD write failed to start");
}

int EbaraHydrostationGateway::write_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                                              struct ble_gatt_attr *attr, void *arg) {
  auto *self = static_cast<EbaraHydrostationGateway *>(arg);
  const int st = error->status;
  self->defer_([self, conn_handle, st]() { self->on_write_cccd_(conn_handle, st); });
  return 0;
}

void EbaraHydrostationGateway::on_write_cccd_(uint16_t conn, int status) {
  if (!this->is_current_conn_(conn) || this->state_ != GwState::WRITE_CCCD)
    return;
  if (status != 0) {
    if (this->connect_purpose_ == ConnectPurpose::INITIAL_BOND && !this->cccd_retry_done_) {
      // Expected pre-bond: the rejected unencrypted CCCD write is the
      // reactive pairing trigger and ENC_CHANGE retries the write. If the
      // link got encrypted before this rejection was processed, ENC_CHANGE
      // has already passed: retry right away.
      struct ble_gap_conn_desc desc {};
      if (ble_gap_conn_find(this->conn_handle_, &desc) == 0 && desc.sec_state.encrypted) {
        ESP_LOGD(TAG, "CCCD write rejected (status=%d) but the link is already encrypted — retrying now.", status);
        this->cccd_retry_done_ = true;
        this->write_cccd_();
        return;
      }
      ESP_LOGD(TAG, "CCCD write rejected (status=%d) as expected pre-bond — waiting for reactive pairing.",
               status);
      this->cccd_retry_pending_ = true;
      return;
    }
    this->fail_discovery_("CCCD write failed");
    return;
  }
  this->cccd_retry_pending_ = false;

  if (this->connect_purpose_ == ConnectPurpose::INITIAL_BOND)
    ESP_LOGI(TAG, this->passkey_seen_this_connection_ ? "Fresh pairing completed." : "Bond already existed (survived reboot).");
  // The link is encrypted and bonded either way: later reconnects reuse the bond.
  this->bond_confirmed_ = true;
  ESP_LOGD(TAG, "CCCD written — subscribed to notifications.");
  // No ble_gap_update_params(): the module requests its connection parameters
  // itself right after the CCCD write and NimBLE applies them. Like the
  // official app, the first command is sent ~2 s after subscribing.
  ESP_LOGI(TAG, "Connected and subscribed — starting poll cycle.");
  this->set_state_(GwState::WAIT_BEFORE_POLL);
  this->set_gw_status_text_("Connected");
  this->set_timeout("subscribe_delay", kSubscribeDelayMs, [this]() {
    if (this->state_ != GwState::WAIT_BEFORE_POLL)
      return;
    this->cancel_timeout("session_watchdog");
    this->next_cmd_allowed_ms_ = millis();
    this->set_state_(GwState::POLL_RUNNING);
    this->set_gw_status_text_("Polling");
    this->enqueue_poll_cycle_();
  });
}

// ── Sequential command queue ─────────────────────────────────────────────────
void EbaraHydrostationGateway::enqueue_poll_cycle_() {
  // Live values, every update interval: status word first (alarm bit and
  // pressure unit are needed to interpret the rest), then the aggregate
  // replies the official app polls and the error word.
  static const char *const kFastCmds[] = {"gm-0005", "gm-0063", "gm-0061", "gm-0011"};
  // Slow-changing values (start pressure, working hours, module temperature,
  // DC bus voltage): at most every kSlowPollIntervalMs.
  static const char *const kSlowCmds[] = {"gm-0002", "gm-0006", "gm-0008", "gm-0009"};
  for (const auto *cmd : kFastCmds)
    this->cmd_queue_.emplace_back(cmd);
  const uint32_t now = millis();
  if (!this->slow_poll_done_ || now - this->last_slow_poll_ms_ >= kSlowPollIntervalMs) {
    for (const auto *cmd : kSlowCmds)
      this->cmd_queue_.emplace_back(cmd);
    this->slow_poll_done_ = true;
    this->last_slow_poll_ms_ = now;
  }
  // Hardware/firmware constants: fetched once per pump (only the missing ones).
  if (!this->static_data_.have_firmware)
    this->cmd_queue_.emplace_back("gm-0051");
  if (!this->static_data_.have_hardware)
    this->cmd_queue_.emplace_back("gm-0052");
  if (!this->static_data_.have_serial)
    this->cmd_queue_.emplace_back("gm-0031");
  ESP_LOGD(TAG, "Poll cycle: %u commands queued", static_cast<unsigned>(this->cmd_queue_.size()));
  this->pump_command_queue_();
}

void EbaraHydrostationGateway::enqueue_readback_front_(const char *cmd) {
  auto it = std::find(this->cmd_queue_.begin(), this->cmd_queue_.end(), cmd);
  if (it != this->cmd_queue_.end())
    this->cmd_queue_.erase(it);
  this->cmd_queue_.insert(this->cmd_queue_.begin(), cmd);
}

void EbaraHydrostationGateway::arm_command_guard_(uint32_t ms) {
  this->next_cmd_allowed_ms_ = millis() + ms;
  this->set_timeout("intercmd_delay", ms, [this]() { this->pump_command_queue_(); });
}

void EbaraHydrostationGateway::pump_command_queue_() {
  if (this->state_ != GwState::POLL_RUNNING || this->cmd_pending_)
    return;
  if (this->set_queue_.empty() && this->cmd_queue_.empty())
    return;
  const uint32_t now = millis();
  if (!time_reached(now, this->next_cmd_allowed_ms_)) {
    // Inter-command guard still running (e.g. a SET requested right after a
    // reply): the intercmd_delay timer resumes the queue.
    const uint32_t remaining = this->next_cmd_allowed_ms_ - now;
    this->set_timeout("intercmd_delay", remaining, [this]() { this->pump_command_queue_(); });
    return;
  }
  // User SETs (FIFO) first, then the periodic GETs.
  auto &queue = this->set_queue_.empty() ? this->cmd_queue_ : this->set_queue_;
  std::string cmd = queue.front();
  queue.erase(queue.begin());
  this->send_command_now_(cmd);
}

void EbaraHydrostationGateway::send_command_now_(const std::string &cmd) {
  this->rx_buf_.clear();
  this->cmd_pending_str_ = cmd;
  this->cmd_pending_ = true;
  // ATT Write Command (no ATT response): the pump answers with a notification.
  int rc = ble_gattc_write_no_rsp_flat(this->conn_handle_, this->rx_val_handle_, cmd.data(), cmd.size());
  if (rc != 0) {
    ESP_LOGW(TAG, "write_no_rsp_flat failed for '%s': rc=%d", cmd.c_str(), rc);
    this->cmd_pending_ = false;
    this->arm_command_guard_(kInterCmdDelayMs);
    return;
  }
  ESP_LOGD(TAG, ">>> %s", cmd.c_str());
  this->set_timeout("cmd_timeout", kCmdTimeoutMs, [this]() { this->handle_command_timeout_(); });
}

void EbaraHydrostationGateway::handle_command_timeout_() {
  if (!this->cmd_pending_)
    return;
  ESP_LOGW(TAG, "TIMEOUT waiting for response to '%s'", this->cmd_pending_str_.c_str());
  this->cmd_pending_ = false;
  this->rx_buf_.clear();
  this->cancel_timeout("rx_flush");
  this->consecutive_timeouts_++;

  if (this->consecutive_timeouts_ < kMaxConsecutiveTimeouts) {
    this->arm_command_guard_(kPostTimeoutGuardMs);
    return;
  }
  ESP_LOGE(TAG, "%u consecutive command timeouts — treating link as dead, reconnecting.",
           static_cast<unsigned>(this->consecutive_timeouts_));
  this->abort_session_("Pump unresponsive — closing the link.");
  this->set_gw_status_text_("Unresponsive — reconnecting...");
}

void EbaraHydrostationGateway::on_notification_(const uint8_t *data, size_t len) {
  if (!this->cmd_pending_) {
    // Includes late replies to a command that already timed out.
    std::string raw(reinterpret_cast<const char *>(data), len);
    ESP_LOGW(TAG, "Discarding notification with no pending command: %s", raw.c_str());
    return;
  }
  this->rx_buf_.append(reinterpret_cast<const char *>(data), len);
  // A reply is complete at '\r' (single value) or ';' (multi value, followed
  // by "\r\n\0"); otherwise wait for more fragments up to kRxFlushMs.
  if (this->rx_buf_.find('\r') != std::string::npos || this->rx_buf_.find(';') != std::string::npos) {
    this->cancel_timeout("rx_flush");
    this->process_rx_buffer_();
  } else {
    this->set_timeout("rx_flush", kRxFlushMs, [this]() { this->process_rx_buffer_(); });
  }
}

void EbaraHydrostationGateway::process_rx_buffer_() {
  if (!this->cmd_pending_ || this->rx_buf_.empty())
    return;
  std::string raw = this->rx_buf_;
  this->rx_buf_.clear();
  while (!raw.empty()) {
    char c = raw.back();
    if (c == '\0' || c == '\r' || c == '\n' || c == ';' || c == ' ')
      raw.pop_back();
    else
      break;
  }
  this->cancel_timeout("cmd_timeout");
  this->consecutive_timeouts_ = 0;
  std::string cmd = this->cmd_pending_str_;
  this->cmd_pending_ = false;
  this->publish_from_response_(cmd, raw);
  this->arm_command_guard_(kInterCmdDelayMs);
}

float EbaraHydrostationGateway::pressure_bar_from_raw_(int32_t raw) const {
  // Raw pressures are bar×10, or psi×10 when status bit 9 is set.
  return this->psi_mode_ ? (raw / 10.0f) / kPsiPerBar : raw / 10.0f;
}

void EbaraHydrostationGateway::invalidate_live_values_() {
  this->status_known_ = false;
  this->slow_poll_done_ = false;  // refresh slow values right after reconnecting
  for (auto *s : {this->target_pressure_sensor_, this->start_pressure_sensor_, this->actual_pressure_sensor_,
                  this->motor_current_sensor_, this->working_hours_sensor_, this->motor_frequency_sensor_,
                  this->module_temperature_sensor_, this->dc_bus_voltage_sensor_, this->delta_pressure_sensor_,
                  this->pump_load_sensor_}) {
    if (s != nullptr && s->has_state() && !std::isnan(s->state))
      s->publish_state(NAN);
  }
  for (auto *n : {this->target_pressure_number_, this->delta_pressure_number_}) {
    if (n != nullptr && n->has_state() && !std::isnan(n->state))
      n->publish_state(NAN);
  }
  for (auto *b : {this->motor_running_bs_, this->motor_enabled_bs_, this->motor_error_bs_, this->external_stop_bs_,
                  this->status_bit10_bs_}) {
    if (b != nullptr)
      b->invalidate_state();
  }
  if (this->pump_status_text_ != nullptr)
    this->pump_status_text_->publish_state("Offline");
  if (this->errors_text_ != nullptr)
    this->errors_text_->publish_state("Unknown (pump offline)");
}

void EbaraHydrostationGateway::publish_status_word_(uint16_t sw) {
  this->status_word_ = sw;
  this->status_known_ = true;
  this->psi_mode_ = (sw & 0x0200) != 0;

  const bool alarm = (sw & 0x8000) != 0;
  const bool enabled = (sw & 0x4000) == 0;      // bit 14 inverted
  const bool running_raw = (sw & 0x2000) != 0;
  const bool no_external_stop = (sw & 0x0800) != 0;  // bit 11 = 1 → no external stop
  const bool assist = (sw & 0x0020) != 0;

  if (this->motor_error_bs_ != nullptr)
    this->motor_error_bs_->publish_state(alarm);
  if (this->motor_enabled_bs_ != nullptr)
    this->motor_enabled_bs_->publish_state(enabled);
  if (this->motor_running_bs_ != nullptr)
    this->motor_running_bs_->publish_state(running_raw && no_external_stop);
  if (this->external_stop_bs_ != nullptr)
    this->external_stop_bs_->publish_state(!no_external_stop);
  if (this->status_bit10_bs_ != nullptr)
    this->status_bit10_bs_->publish_state((sw & 0x0400) != 0);
  // The "Motor" switch reflects whether automatic control is enabled
  // (sm-0005), not whether the motor is spinning right now.
  if (this->motor_switch_ != nullptr)
    this->motor_switch_->publish_state(enabled);
  if (this->pressure_unit_text_ != nullptr)
    this->pressure_unit_text_->publish_state(this->psi_mode_ ? "psi" : "bar");

  // Pump status text, same priority as the official app (updateTextViews).
  if (this->pump_status_text_ != nullptr) {
    const char *st;
    if (!no_external_stop)
      st = "External Stop";
    else if (running_raw)
      st = "Running";
    else if (assist)
      st = "Assist Mode";
    else if (enabled && !alarm)
      st = "Standby";
    else if (alarm)
      st = "Alarm";
    else
      st = "Pump Off";
    this->pump_status_text_->publish_state(st);
  }
}

void EbaraHydrostationGateway::save_static_data_() {
  auto &sd = this->static_data_;
  sd.valid = sd.have_serial && sd.have_hardware && sd.have_firmware;
  this->static_data_pref_.save(&sd);
}

void EbaraHydrostationGateway::publish_from_response_(const std::string &cmd, const std::string &raw) {
  ESP_LOGI(TAG, "<<< %s : %s", cmd.c_str(), raw.c_str());

  if (cmd.rfind("sm-", 0) == 0) {
    // SET acknowledgement: "SET DATO OK" (app 1.2.7) or "DATOOK;" (app 2.0).
    if (raw.find("SET DATO OK") != std::string::npos || raw.find("DATOOK") != std::string::npos)
      ESP_LOGI(TAG, "SET '%s' acknowledged by the pump.", cmd.c_str());
    else
      ESP_LOGW(TAG, "Unexpected reply to SET '%s': %s", cmd.c_str(), raw.c_str());
    // Read back the new values right away (once, even after several SETs).
    this->enqueue_readback_front_("gm-0002");
    this->enqueue_readback_front_("gm-0063");
    this->enqueue_readback_front_("gm-0005");
    return;
  }

  auto nums = parse_dato(raw);
  if (nums.size() != expected_value_count(cmd)) {
    ESP_LOGW(TAG, "Discarding reply to %s with unexpected shape (%u values): %s", cmd.c_str(),
             static_cast<unsigned>(nums.size()), raw.c_str());
    return;
  }
  const int32_t v = nums[0];
  const bool is_pressure_reply = cmd == "gm-0063" || cmd == "gm-0002" || cmd == "gm-0061";
  if (is_pressure_reply && !this->status_known_) {
    // The pressure unit (bar/psi, status bit 9) is not known in this session
    // yet: do not publish values that could be off by a factor 14.5.
    ESP_LOGW(TAG, "Pressure reply to %s ignored: pressure unit not known yet (status word missing).", cmd.c_str());
    return;
  }

  if (cmd == "gm-0005") {
    this->publish_status_word_(static_cast<uint16_t>(v & 0xFFFF));
  } else if (cmd == "gm-0063") {
    // "Dato: targetPress,deltaPress"
    float target = this->pressure_bar_from_raw_(nums[0]);
    float delta = this->pressure_bar_from_raw_(nums[1]);
    if (this->target_pressure_sensor_ != nullptr)
      this->target_pressure_sensor_->publish_state(target);
    if (this->target_pressure_number_ != nullptr)
      this->target_pressure_number_->publish_state(target);
    if (this->delta_pressure_sensor_ != nullptr)
      this->delta_pressure_sensor_->publish_state(delta);
    if (this->delta_pressure_number_ != nullptr)
      this->delta_pressure_number_->publish_state(delta);
  } else if (cmd == "gm-0002") {
    // Start pressure: read-only, equals target − delta.
    if (this->start_pressure_sensor_ != nullptr)
      this->start_pressure_sensor_->publish_state(this->pressure_bar_from_raw_(v));
  } else if (cmd == "gm-0061") {
    // "Dato: actPress,freqMotor,currMotor"
    if (this->actual_pressure_sensor_ != nullptr)
      this->actual_pressure_sensor_->publish_state(this->pressure_bar_from_raw_(nums[0]));
    // Unit not confirmed (never displayed by the app): published raw.
    if (this->motor_frequency_sensor_ != nullptr)
      this->motor_frequency_sensor_->publish_state(nums[1]);
    if (this->motor_current_sensor_ != nullptr)
      this->motor_current_sensor_->publish_state(nums[2] / 10.0f);
    if (this->pump_load_sensor_ != nullptr)
      this->pump_load_sensor_->publish_state(pump_load_from_raw_current(nums[2]));
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
    const uint16_t word = static_cast<uint16_t>(v & 0xFFFF);
    if (this->errors_text_ != nullptr) {
      // Like the app, alarm texts are shown only while status bit 15 is set.
      const bool alarm_active = this->status_known_ && (this->status_word_ & 0x8000);
      std::string text;
      if (alarm_active)
        text = word != 0 ? decode_alarms(word, this->psi_mode_) : std::string("Alarm (no code)");
      else if (this->status_known_)
        text = "No active alarms";
      else
        text = "Unknown (status not read)";
      this->errors_text_->publish_state(text);
    }
  } else if (cmd == "gm-0051") {
    const uint16_t fw = static_cast<uint16_t>(v & 0xFFFF);
    if (this->firmware_version_text_ != nullptr)
      this->firmware_version_text_->publish_state(format_firmware(fw));
    this->static_data_.firmware_version_raw = fw;
    this->static_data_.have_firmware = true;
    this->save_static_data_();
  } else if (cmd == "gm-0052") {
    if (this->hardware_version_sensor_ != nullptr)
      this->hardware_version_sensor_->publish_state(v);
    this->static_data_.hardware_version = static_cast<uint16_t>(v);
    this->static_data_.have_hardware = true;
    this->save_static_data_();
  } else if (cmd == "gm-0031") {
    uint32_t serial = 0;
    if (!parse_dato_uint32(raw, &serial))
      return;
    if (this->serial_number_text_ != nullptr)
      this->serial_number_text_->publish_state(format_serial(serial));
    this->static_data_.serial_number = serial;
    this->static_data_.have_serial = true;
    this->save_static_data_();
  }
}

void EbaraHydrostationGateway::set_state_(GwState s) {
  if (s != this->state_)
    ESP_LOGD(TAG, "State: %s -> %s", state_name_(this->state_), state_name_(s));
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
    case GwState::DISCOVER_SVC:
      return "DISCOVER_SVC";
    case GwState::DISCOVER_CHR:
      return "DISCOVER_CHR";
    case GwState::DISCOVER_CCCD:
      return "DISCOVER_CCCD";
    case GwState::READ_DEVICE_NAME:
      return "READ_DEVICE_NAME";
    case GwState::WRITE_CCCD:
      return "WRITE_CCCD";
    case GwState::WAIT_BEFORE_POLL:
      return "WAIT_BEFORE_POLL";
    case GwState::POLL_RUNNING:
      return "POLL_RUNNING";
    case GwState::COOLDOWN:
      return "COOLDOWN";
    case GwState::UNSUPPORTED:
      return "UNSUPPORTED";
  }
  return "?";
}

}  // namespace ebara_hydrostation
}  // namespace esphome
