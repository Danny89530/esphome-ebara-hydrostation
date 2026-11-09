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
#include <functional>
#include <mutex>

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

// Text entity used to receive the pump's target MAC address from Home Assistant.
class EbaraTargetMacText : public text::Text, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }
  void control(const std::string &value) override;

 protected:
  EbaraHydrostationGateway *parent_{nullptr};
};

// Switch entity for enabling/disabling the pump's automatic control (sm-0005).
class EbaraMotorSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;
  EbaraHydrostationGateway *parent_{nullptr};
};

// Master enable/disable switch for the whole gateway. OFF closes the BLE link
// (or cancels a pending connection/scan) and suppresses all reconnect/poll
// activity; ON (re)starts the connection right away. Sends no command to the
// pump.
class EbaraEnableSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }
  void setup() override;

 protected:
  void write_state(bool state) override;
  EbaraHydrostationGateway *parent_{nullptr};
};

// Which SET parameter a given EbaraSetpointNumber controls: the two setpoints
// the official app writes for HYDRO pumps, sm-0001 (target pressure) and
// sm-0012 (restart delta). The start pressure is read-only (target - delta).
enum class EbaraSetpointKind : uint8_t {
  TARGET_PRESSURE,
  DELTA_PRESSURE,
};

// Number entity for the SET parameters. Always expressed in bar on the HA
// side; converted to the pump's own unit (bar×10 or psi×10, status bit 9)
// before being sent.
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
class EbaraUpdateIntervalNumber : public number::Number, public Component {
 public:
  void set_parent(EbaraHydrostationGateway *parent) { this->parent_ = parent; }
  void setup() override;

 protected:
  void control(float value) override;
  EbaraHydrostationGateway *parent_{nullptr};
  ESPPreferenceObject pref_;
};

// High-level connection/session state. The pump's BM70/BM71 module only
// supports a single outstanding ATT operation at a time, so the whole session
// is modeled as a strict sequential state machine.
enum class GwState : uint8_t {
  IDLE = 0,
  CONNECTING,
  ENCRYPTING,
  DISCOVER_SVC,
  DISCOVER_CHR,
  DISCOVER_CCCD,
  // Reads the GAP Device Name (0x2A00) to verify the product family before
  // subscribing: only "HYDRO*" pumps (legacy gm-/sm- protocol) are supported.
  READ_DEVICE_NAME,
  WRITE_CCCD,
  WAIT_BEFORE_POLL,
  POLL_RUNNING,
  COOLDOWN,
  // A non-HYDRO device (e.g. HYSTA, which uses a different push-based
  // protocol) was detected: stay idle until the target MAC changes.
  UNSUPPORTED,
};

// INITIAL_BOND: no bond confirmed since boot — discover first, pairing (or
// encryption with a stored LTK) is triggered by the pump's Security Request /
// the rejected unencrypted CCCD write. SERVICE_SESSION: bond confirmed in this
// boot — encrypt first, then discover.
enum class ConnectPurpose : uint8_t {
  INITIAL_BOND,
  SERVICE_SESSION,
};

class EbaraHydrostationGateway : public PollingComponent {
 public:
  EbaraHydrostationGateway() : PollingComponent(10000) {}

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  // Terminates the BLE link cleanly before a controlled reboot: the module
  // keeps an unterminated link alive until its 5.12 s supervision timeout and
  // forwards no UART data to a new session opened in the meantime.
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

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
  void set_hardware_version_sensor(sensor::Sensor *s) { this->hardware_version_sensor_ = s; }
  void set_pump_load_sensor(sensor::Sensor *s) { this->pump_load_sensor_ = s; }
  void set_discovered_text_sensor(text_sensor::TextSensor *s) { this->discovered_text_ = s; }

  // Text sensors.
  void set_serial_number_text_sensor(text_sensor::TextSensor *s) { this->serial_number_text_ = s; }
  void set_firmware_version_text_sensor(text_sensor::TextSensor *s) { this->firmware_version_text_ = s; }
  void set_pump_status_text_sensor(text_sensor::TextSensor *s) { this->pump_status_text_ = s; }
  void set_pressure_unit_text_sensor(text_sensor::TextSensor *s) { this->pressure_unit_text_ = s; }
  void set_device_name_text_sensor(text_sensor::TextSensor *s) { this->device_name_text_ = s; }

  void set_motor_running_binary_sensor(binary_sensor::BinarySensor *s) { this->motor_running_bs_ = s; }
  void set_motor_enabled_binary_sensor(binary_sensor::BinarySensor *s) { this->motor_enabled_bs_ = s; }
  void set_motor_error_binary_sensor(binary_sensor::BinarySensor *s) { this->motor_error_bs_ = s; }
  void set_external_stop_binary_sensor(binary_sensor::BinarySensor *s) { this->external_stop_bs_ = s; }
  // Status word bit 10: not decoded by the official app nor documented in the
  // pump manual; exposed raw.
  void set_status_bit10_binary_sensor(binary_sensor::BinarySensor *s) { this->status_bit10_bs_ = s; }

  void set_motor_switch(EbaraMotorSwitch *s) { this->motor_switch_ = s; }
  void set_gateway_enabled(bool enabled);
  void set_target_pressure_number(EbaraSetpointNumber *n) { this->target_pressure_number_ = n; }
  void set_delta_pressure_number(EbaraSetpointNumber *n) { this->delta_pressure_number_ = n; }
  void set_update_interval_seconds(float seconds);
  // Default for the "Update Interval" entity when no value is persisted yet:
  // the YAML update_interval.
  void set_default_update_interval_s(float s) { this->default_interval_s_ = s; }
  float get_default_update_interval_s() const { return this->default_interval_s_; }

  void on_target_mac_set(const std::string &mac_str);

  // Called by EbaraMotorSwitch. Returns false (command not sent) when the
  // gateway is not polling, the status word is still unknown, or an alarm is
  // active (status bit 15): the official app sends nothing in that case.
  bool request_motor(bool enable);
  // Called by EbaraSetpointNumber with a value in bar. Converts it to the
  // pump's unit/range/step and enqueues the SET. Returns false if not sent.
  bool request_setpoint(EbaraSetpointKind kind, float value_bar);

  // Entry points called from the NimBLE host task. They only copy the event
  // data and defer the actual handling to the ESPHome main loop: entity
  // publishing, preferences and scheduler timeouts are not thread-safe and too
  // heavy for the small nimble_host task stack.
  int handle_gap_event(struct ble_gap_event *event);
  void handle_host_reset(int reason);
  void handle_host_sync();

 protected:
  void defer_(std::function<void()> &&fn);
  void run_deferred_();
  void host_synced_();
  void process_gap_event_(struct ble_gap_event *event);
  void process_disc_(ble_addr_t addr, int8_t rssi, const std::vector<uint8_t> &adv);
  // GATT procedure results. `conn` is the connection the procedure ran on:
  // results of a link that is already gone are ignored.
  void on_disc_svc_(uint16_t conn, int status, struct ble_gatt_svc svc);
  void on_disc_chr_(uint16_t conn, int status, struct ble_gatt_chr chr);
  void on_disc_dsc_(uint16_t conn, int status, uint16_t chr_val_handle, struct ble_gatt_dsc dsc);
  void on_read_name_(uint16_t conn, int status, const std::string &name);
  void on_write_cccd_(uint16_t conn, int status);
  bool is_current_conn_(uint16_t conn) const {
    return conn != BLE_HS_CONN_HANDLE_NONE && conn == this->conn_handle_;
  }

  std::mutex deferred_mutex_;
  std::vector<std::function<void()>> deferred_;
  // Upper bound on queued events: advertising reports are dropped beyond it
  // (connection events never are).
  static constexpr size_t kMaxDeferredAdv = 32;

  static const char *state_name_(GwState s);
  void set_state_(GwState s);
  void set_gw_status_text_(const std::string &s);

  bool can_connect_now_() const;
  void start_connect_();
  // Aborts the current connection attempt or link (if any), clears the
  // session and goes to COOLDOWN. Used by disable / MAC change / watchdog.
  void abort_session_(const char *reason);
  // Session bookkeeping shared by every disconnect path: queues, pending
  // command, reassembly buffer and all session timers.
  void clear_session_state_();
  void fail_discovery_(const char *reason);
  void begin_service_discovery_();
  // Continues a SERVICE_SESSION connection once the link is encrypted. Called
  // from whichever of CONNECT / ENC_CHANGE observes the encrypted link last,
  // so event ordering does not matter.
  void on_link_encrypted_();
  void read_device_name_();
  void write_cccd_();
  void mark_unsupported_(const std::string &name);
  bool enqueue_set_command_(const std::string &cmd);
  // Two-speed polling: live values every update interval, slow-changing
  // values every kSlowPollIntervalMs.
  void enqueue_poll_cycle_();
  // Queues a GET at the front of the queue unless already queued.
  void enqueue_readback_front_(const char *cmd);
  // Sends the next command (SETs first) once the previous one is answered
  // and the inter-command guard has elapsed.
  void pump_command_queue_();
  void arm_command_guard_(uint32_t ms);
  void save_static_data_();
  void send_command_now_(const std::string &cmd);
  void on_notification_(const uint8_t *data, size_t len);
  void process_rx_buffer_();
  void handle_command_timeout_();
  void publish_from_response_(const std::string &cmd, const std::string &raw);
  void publish_status_word_(uint16_t sw);
  // Marks every live value as unknown (pump offline/unreachable) so that Home
  // Assistant never shows stale readings as if they were current. Static data (firmware, hardware, serial, device name) is kept.
  void invalidate_live_values_();
  float pressure_bar_from_raw_(int32_t raw) const;

  void start_scan_();
  void stop_scan_();
  void scan_heartbeat_();
  void publish_discovered_();

  void on_scan_complete_();

  static int disc_svc_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_svc *service, void *arg);
  static int disc_chr_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg);
  static int disc_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                            uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg);
  static int read_name_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                            struct ble_gatt_attr *attr, void *arg);
  static int write_cccd_cb_(uint16_t conn_handle, const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg);

  GwState state_{GwState::IDLE};
  bool gateway_enabled_{true};
  // Own address: a STATIC random address generated once and persisted in NVS.
  // The module forwards UART data only to a central using a random address,
  // and the address must be stable across reboots because the pump keys its
  // bond on it.
  bool use_random_own_addr_{false};
  struct PersistedOwnAddr {
    uint8_t addr[6];
    bool valid;
  };
  ESPPreferenceObject own_addr_pref_;

  struct DiscoveredHydro {
    std::string mac;
    std::string name;
    int8_t rssi;
    bool supported;
  };
  std::vector<DiscoveredHydro> discovered_hydros_;
  bool scanning_{false};
  text_sensor::TextSensor *discovered_text_{nullptr};

  struct PersistedMac {
    uint8_t mac[6];
    bool valid;
  };
  ESPPreferenceObject mac_pref_;

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
  // RAM copy of the persisted static data (loaded once in setup()).
  PersistedStaticData static_data_{};

  std::string default_target_mac_;
  bool have_target_addr_{false};
  ble_addr_t target_addr_{};
  uint16_t conn_handle_{BLE_HS_CONN_HANDLE_NONE};
  ConnectPurpose connect_purpose_{ConnectPurpose::INITIAL_BOND};
  bool passkey_seen_this_connection_{false};
  // A bonded, encrypted session was established in this boot: reconnects
  // encrypt first (SERVICE_SESSION).
  bool bond_confirmed_{false};
  bool cccd_retry_pending_{false};
  bool cccd_retry_done_{false};
  // millis() of the last disconnect, to space out reconnection attempts.
  uint32_t last_disconnect_ms_{0};
  bool ever_disconnected_{false};

  uint16_t tx_val_handle_{0};
  uint16_t cccd_handle_{0};
  uint16_t rx_val_handle_{0};
  uint16_t name_val_handle_{0};  // GAP Device Name (0x2A00)

  struct DiscSvc {
    uint16_t start_handle;
    uint16_t end_handle;
  };
  struct DiscChr {
    uint16_t val_handle;
    uint16_t dsc_range_end;
  };
  std::vector<DiscSvc> disc_services_;
  size_t disc_svc_idx_{0};
  std::vector<DiscChr> disc_chrs_;
  size_t disc_chr_idx_{0};
  size_t disc_chrs_svc_start_idx_{0};

  // Sequential command queue: only one outstanding command at a time. User
  // SETs have their own FIFO (one entry per register, the latest value wins)
  // served before the periodic GETs.
  std::vector<std::string> set_queue_;
  std::vector<std::string> cmd_queue_;
  bool cmd_pending_{false};
  std::string cmd_pending_str_;
  // No command is sent before this millis() value (inter-command guard).
  uint32_t next_cmd_allowed_ms_{0};
  uint8_t consecutive_timeouts_{0};
  // Notification reassembly buffer (official app: accumulate, split on ';',
  // flush after a quiet period). Single-value replies end with '\r',
  // multi-value replies with ";\r\n\0".
  std::string rx_buf_;

  // Last decoded status word (gm-0005) and the pressure unit it implies.
  bool status_known_{false};
  float default_interval_s_{10.0f};
  bool poller_ready_{false};
  bool slow_poll_done_{false};
  uint32_t last_slow_poll_ms_{0};
  uint16_t status_word_{0};
  bool psi_mode_{false};

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
  sensor::Sensor *hardware_version_sensor_{nullptr};
  sensor::Sensor *pump_load_sensor_{nullptr};

  binary_sensor::BinarySensor *motor_running_bs_{nullptr};
  binary_sensor::BinarySensor *motor_enabled_bs_{nullptr};
  binary_sensor::BinarySensor *motor_error_bs_{nullptr};
  binary_sensor::BinarySensor *external_stop_bs_{nullptr};
  binary_sensor::BinarySensor *status_bit10_bs_{nullptr};

  text_sensor::TextSensor *serial_number_text_{nullptr};
  text_sensor::TextSensor *firmware_version_text_{nullptr};
  text_sensor::TextSensor *pump_status_text_{nullptr};
  text_sensor::TextSensor *pressure_unit_text_{nullptr};
  text_sensor::TextSensor *device_name_text_{nullptr};

  EbaraMotorSwitch *motor_switch_{nullptr};
  EbaraSetpointNumber *target_pressure_number_{nullptr};
  EbaraSetpointNumber *delta_pressure_number_{nullptr};
};

}  // namespace ebara_hydrostation
}  // namespace esphome
