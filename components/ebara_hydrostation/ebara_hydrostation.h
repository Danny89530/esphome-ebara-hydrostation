#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/text/text.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"

#include <string>
#include <vector>
#include <cstdint>

extern "C" {
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "host/ble_uuid.h"
}

namespace esphome {
namespace ebara_hydrostation {

class EbaraHydrostationGateway;

// Text entity used to receive the BM71 target MAC address from Home Assistant.
class EbaraTargetMacText : public text::Text, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }
  void control(const std::string &value) override;

 protected:
  EbaraHydrostationGateway *parent_{nullptr};
};

// Switch entity for motor start/stop (sm-0005).
class EbaraMotorSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;
  EbaraHydrostationGateway *parent_{nullptr};
};

// Master enable/disable switch for the whole gateway. OFF disconnects
// immediately and suppresses all auto-reconnect/poll activity; ON
// (re)starts the connection right away. Purely local — no BLE traffic.
class EbaraEnableSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }
  // Applies the restore_mode-computed initial state (persisted preference,
  // or the configured default on first boot) before the gateway itself
  // starts connecting — without this, the entity's displayed state stays at
  // its raw default (OFF) regardless of what the gateway is actually doing.
  void setup() override;

 protected:
  void write_state(bool state) override;
  EbaraHydrostationGateway *parent_{nullptr};
};

// Which SET parameter a given EbaraSetpointNumber controls.
enum class EbaraSetpointKind : uint8_t {
  TARGET_PRESSURE,
  START_PRESSURE,
  DELTA_PRESSURE,
};

// Number entity for the SET parameters (sm-0001, sm-0002, sm-0012).
class EbaraSetpointNumber : public number::Number, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }
  void set_kind(EbaraSetpointKind kind) { this->kind_ = kind; }

 protected:
  void control(float value) override;
  EbaraHydrostationGateway *parent_{nullptr};
  EbaraSetpointKind kind_{EbaraSetpointKind::TARGET_PRESSURE};
};

// Number entity controlling the poll update_interval itself (seconds).
// Min/max are enforced both by the entity's own config (min_value/max_value,
// set in __init__.py) and defensively again in
// EbaraHydrostationGateway::set_update_interval_seconds().
class EbaraUpdateIntervalNumber : public number::Number, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }
  // Restores the persisted interval (or a sensible default) before polling
  // starts — see the EbaraEnableSwitch registration gotcha this mirrors.
  void setup() override;

 protected:
  void control(float value) override;
  EbaraHydrostationGateway *parent_{nullptr};
  ESPPreferenceObject pref_;
};

// High-level connection/session state. The BM71 only supports a single
// outstanding ATT operation at a time, so the whole session is modeled as a
// strict sequential state machine rather than allowing concurrent GATT
// operations.
enum class GwState : uint8_t {
  IDLE = 0,
  CONNECTING,
  ENCRYPTING,
  // Verifies that a fresh pairing's bond was actually persisted: disconnect
  // right after the first successful encryption, then reconnect and
  // re-encrypt without a new pairing dialog. Skipped if the bond already
  // existed before this connection (it survived an ESP reboot).
  BOND_VERIFY_DISCONNECTING,
  BOND_CONFIRMED,
  DISCOVER_SVC,
  DISCOVER_CHR,
  // Pairing is triggered reactively: the CCCD write below is issued before
  // the link is encrypted, so it fails with insufficient-authentication —
  // that rejection is what makes the BM71 initiate pairing.
  DISCOVER_CCCD,
  WRITE_CCCD,
  WAIT_BEFORE_POLL,
  POLL_RUNNING,
  COOLDOWN,
};

// Distinguishes why a connection was initiated, since BLE_GAP_EVENT_CONNECT /
// BLE_GAP_EVENT_ENC_CHANGE are shared entry points for all three flows.
enum class ConnectPurpose : uint8_t {
  INITIAL_BOND,
  BOND_VERIFY,
  SERVICE_SESSION,
};

class EbaraHydrostationGateway : public PollingComponent {
 public:
  EbaraHydrostationGateway() : PollingComponent(15000) {}

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // If true, the firmware stops right after BOND_CONFIRMED and never
  // proceeds to service discovery / polling.
  void set_stop_after_bond_verify(bool value) { this->stop_after_bond_verify_ = value; }

  // Applied once at boot if set (YAML `default_target_mac`), letting a
  // target MAC be preconfigured instead of entered through the text entity.
  void set_default_target_mac(const std::string &mac) { this->default_target_mac_ = mac; }
  void set_target_mac_entity(EbaraTargetMacText *t) { this->target_mac_text_ = t; }
  void set_gw_status(text_sensor::TextSensor *s) { this->gw_status_ = s; }
  void set_errors_text_sensor(text_sensor::TextSensor *s) { this->errors_text_ = s; }

  void set_target_pressure_sensor(sensor::Sensor *s) { this->target_pressure_sensor_ = s; }
  void set_start_pressure_sensor(sensor::Sensor *s) { this->start_pressure_sensor_ = s; }
  void set_actual_pressure_sensor(sensor::Sensor *s) { this->actual_pressure_sensor_ = s; }
  void set_motor_current_sensor(sensor::Sensor *s) { this->motor_current_sensor_ = s; }
  void set_working_hours_sensor(sensor::Sensor *s) { this->working_hours_sensor_ = s; }
  void set_motor_frequency_sensor(sensor::Sensor *s) { this->motor_frequency_sensor_ = s; }
  void set_module_temperature_sensor(sensor::Sensor *s) { this->module_temperature_sensor_ = s; }
  void set_dc_bus_voltage_sensor(sensor::Sensor *s) { this->dc_bus_voltage_sensor_ = s; }
  void set_delta_pressure_sensor(sensor::Sensor *s) { this->delta_pressure_sensor_ = s; }
  void set_firmware_version_sensor(sensor::Sensor *s) { this->firmware_version_sensor_ = s; }
  void set_hardware_version_sensor(sensor::Sensor *s) { this->hardware_version_sensor_ = s; }
  void set_water_level_sensor(sensor::Sensor *s) { this->water_level_sensor_ = s; }
  void set_error_word_sensor(sensor::Sensor *s) { this->error_word_sensor_ = s; }
  void set_status_word_sensor(sensor::Sensor *s) { this->status_word_sensor_ = s; }
  void set_discovered_text_sensor(text_sensor::TextSensor *s) { this->discovered_text_ = s; }

  // Serial number is an unsigned 32-bit field (an unprogrammed unit reports
  // the sentinel value 0xFFFFFFFF) — exposed as text rather than a numeric
  // sensor, since both signed 32-bit parsing and ESPHome's float32 Sensor
  // (24-bit mantissa) lose precision above 2^31/2^24 respectively.
  void set_serial_number_text_sensor(text_sensor::TextSensor *s) { this->serial_number_text_ = s; }

  void set_motor_running_binary_sensor(binary_sensor::BinarySensor *s) { this->motor_running_bs_ = s; }
  void set_motor_enabled_binary_sensor(binary_sensor::BinarySensor *s) { this->motor_enabled_bs_ = s; }
  void set_motor_error_binary_sensor(binary_sensor::BinarySensor *s) { this->motor_error_bs_ = s; }

  void set_motor_switch(EbaraMotorSwitch *s) { this->motor_switch_ = s; }
  // Called by EbaraEnableSwitch when the user flips the master enable switch.
  void set_gateway_enabled(bool enabled);
  void set_target_pressure_number(EbaraSetpointNumber *n) { this->target_pressure_number_ = n; }
  void set_start_pressure_number(EbaraSetpointNumber *n) { this->start_pressure_number_ = n; }
  void set_delta_pressure_number(EbaraSetpointNumber *n) { this->delta_pressure_number_ = n; }
  // Called by EbaraUpdateIntervalNumber; clamps to [kUpdateIntervalMinS,
  // kUpdateIntervalMaxS] before applying.
  void set_update_interval_seconds(float seconds);

  // Called by EbaraTargetMacText when the user enters/restores a MAC address.
  void on_target_mac_set(const std::string &mac_str);
  // Called by EbaraMotorSwitch / EbaraSetpointNumber to enqueue a SET command.
  void enqueue_set_command(const std::string &cmd);

  // Entry point for the NimBLE GAP event callback trampoline.
  int handle_gap_event(struct ble_gap_event *event);
  void handle_host_reset(int reason);
  void handle_host_sync();

 protected:
  static const char *state_name_(GwState s);
  void set_state_(GwState s);
  void set_gw_status_text_(const std::string &s);

  void start_connect_(ConnectPurpose purpose);
  // Terminates the active connection (if any) before moving to COOLDOWN, so
  // every discovery/CCCD-write failure path always tears the link down
  // first instead of leaving a stale connection for the next reconnect
  // attempt to collide with.
  void fail_discovery_(const char *reason);
  void begin_bond_verify_disconnect_();
  void after_bond_confirmed_();
  void begin_service_discovery_();
  void enqueue_full_poll_cycle_();
  void pump_command_queue_();
  void send_command_now_(const std::string &cmd);
  void on_notification_(const uint8_t *data, size_t len);
  void handle_command_timeout_();
  void publish_from_response_(const std::string &cmd, const std::string &raw);

  // BLE scan for nearby Hydrostations, filtered on their advertised
  // "HYDRO_"/"HYSTA_" name prefix. Only runs while no target MAC is known
  // yet: start_scan_() only fires pre-configuration, and on_target_mac_set()
  // cancels it unconditionally the moment a MAC arrives.
  void start_scan_();
  void stop_scan_();
  void scan_heartbeat_();
  void publish_discovered_();

  // GATT discovery callbacks (NimBLE requires plain function pointers).
  static int disc_svc_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_svc *service, void *arg);
  static int disc_chr_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg);
  static int disc_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                            uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
  static int write_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg);

  GwState state_{GwState::IDLE};
  bool stop_after_bond_verify_{true};
  // Master enable/disable, controlled by EbaraEnableSwitch. When false,
  // update() and every auto-connect entry point stay inert and any active
  // connection has already been torn down by set_gateway_enabled().
  bool gateway_enabled_{true};
  // True once a random (non-resolvable-private) own-address has been
  // configured in handle_host_sync(), used instead of the PUBLIC type
  // ble_hs_id_infer_auto() picks by default.
  bool use_random_own_addr_{false};

  struct DiscoveredHydro {
    std::string mac;
    std::string name;
    int8_t rssi;
  };
  std::vector<DiscoveredHydro> discovered_hydros_;
  bool scanning_{false};
  text_sensor::TextSensor *discovered_text_{nullptr};

  // Flash-persisted target MAC (NVS-backed via ESPHome's own preference API)
  // so the gateway remembers which Hydrostation to connect to across
  // reboots independent of Home Assistant re-sending it — the discovery
  // flow only runs once, at first-time setup.
  struct PersistedMac {
    uint8_t mac[6];
    bool valid;
  };
  ESPPreferenceObject mac_pref_;

  // Hardware/firmware constants that never change for the life of a given
  // pairing (serial number, hardware version, firmware version): fetched
  // once and cached here instead of being re-queried on every poll cycle.
  // have_serial/have_hardware/have_firmware track partial progress
  // individually (in case a fetch cycle only completes some of them before
  // a disconnect), so only the still-missing ones are retried; valid is
  // true once all three are known. Invalidated (all fields reset) whenever
  // on_target_mac_set() sees an actual change of target — a different MAC
  // may be a different physical pump — but survives a reboot that restores
  // the same, already-known MAC.
  struct PersistedStaticData {
    uint32_t serial_number;
    uint16_t hardware_version;
    uint16_t firmware_version_raw;
    bool have_serial;
    bool have_hardware;
    bool have_firmware;
    bool valid;
  };
  ESPPreferenceObject static_data_pref_;
  bool static_data_valid_{false};

  std::string default_target_mac_;
  bool have_target_addr_{false};
  ble_addr_t target_addr_{};
  uint16_t conn_handle_{BLE_HS_CONN_HANDLE_NONE};
  ConnectPurpose connect_purpose_{ConnectPurpose::INITIAL_BOND};
  bool passkey_seen_this_connection_{false};
  bool bond_verify_done_{false};
  // True after the CCCD write is rejected pre-bond (see DISCOVER_CCCD/
  // WRITE_CCCD comment) and until the reactive-pairing retry of that same
  // write completes.
  bool cccd_retry_pending_{false};
  // True once the single priming connection (CCCD write success, held 5s,
  // then disconnected) has completed this boot. Set permanently for the rest
  // of the boot — the next CCCD-write success (on the reconnect) is treated
  // as the real session and proceeds straight to polling.
  bool session_primed_{false};

  uint16_t tx_val_handle_{0};
  uint16_t tx_def_handle_{0};
  uint16_t cccd_handle_{0};
  uint16_t rx_val_handle_{0};

  // Full-database discovery: every primary service on the BM71 (Generic
  // Access, Generic Attribute, Device Information, then the custom UART
  // service) is walked in turn — all services, then all characteristics per
  // service, then all descriptors per characteristic — rather than
  // discovering only the custom service by UUID.
  struct DiscSvc {
    uint16_t start_handle;
    uint16_t end_handle;
  };
  struct DiscChr {
    uint16_t val_handle;
    uint16_t dsc_range_end;  // upper bound to search for this char's descriptors
  };
  std::vector<DiscSvc> disc_services_;
  size_t disc_svc_idx_{0};
  std::vector<DiscChr> disc_chrs_;
  size_t disc_chr_idx_{0};
  // disc_chrs_.size() at the moment the current service's characteristic
  // discovery started — lets disc_chr_cb_ confirm that disc_chrs_.back()
  // belongs to the service currently being scanned before correcting its
  // dsc_range_end, so a correction never lands on the previous service's
  // last characteristic.
  size_t disc_chrs_svc_start_idx_{0};

  // Sequential command queue: only one outstanding ATT operation at a time.
  std::vector<std::string> cmd_queue_;
  bool cmd_pending_{false};
  std::string cmd_pending_str_;
  uint32_t cmd_sent_at_ms_{0};
  // Counts timeouts back-to-back, with no successful response in between.
  // The BLE link can stay nominally connected while the pump stops
  // answering (its supervision timeout not yet elapsed) — this detects that
  // case and forces a reconnect instead of retrying forever with no visible
  // status change.
  uint8_t consecutive_timeouts_{0};

  EbaraTargetMacText *target_mac_text_{nullptr};
  text_sensor::TextSensor *gw_status_{nullptr};
  text_sensor::TextSensor *errors_text_{nullptr};

  sensor::Sensor *target_pressure_sensor_{nullptr};
  sensor::Sensor *start_pressure_sensor_{nullptr};
  sensor::Sensor *actual_pressure_sensor_{nullptr};
  sensor::Sensor *motor_current_sensor_{nullptr};
  sensor::Sensor *working_hours_sensor_{nullptr};
  sensor::Sensor *motor_frequency_sensor_{nullptr};
  sensor::Sensor *module_temperature_sensor_{nullptr};
  sensor::Sensor *dc_bus_voltage_sensor_{nullptr};
  sensor::Sensor *delta_pressure_sensor_{nullptr};
  sensor::Sensor *firmware_version_sensor_{nullptr};
  sensor::Sensor *hardware_version_sensor_{nullptr};
  sensor::Sensor *water_level_sensor_{nullptr};
  sensor::Sensor *error_word_sensor_{nullptr};
  sensor::Sensor *status_word_sensor_{nullptr};

  binary_sensor::BinarySensor *motor_running_bs_{nullptr};
  binary_sensor::BinarySensor *motor_enabled_bs_{nullptr};
  binary_sensor::BinarySensor *motor_error_bs_{nullptr};

  text_sensor::TextSensor *serial_number_text_{nullptr};

  EbaraMotorSwitch *motor_switch_{nullptr};
  EbaraSetpointNumber *target_pressure_number_{nullptr};
  EbaraSetpointNumber *start_pressure_number_{nullptr};
  EbaraSetpointNumber *delta_pressure_number_{nullptr};
};

}  // namespace ebara_hydrostation
}  // namespace esphome
