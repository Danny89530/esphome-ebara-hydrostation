# Changelog

## 2.0.0

Use together with the Home Assistant integration
[ebara_hydrostation](https://github.com/Danny89530/ebara_hydrostation) **2.0.0** or newer.

### Updating your YAML (from 1.0.x)

If your YAML pulls this repository with `type: git` (and `refresh` has
expired, or is `0s`), the new component is downloaded automatically when you
compile. **Your existing YAML still compiles unchanged**, with warnings for the
keys listed below — but edit it as follows to get every feature.

1. **Remove** these keys from `ebara_hydrostation:` (they are now ignored):

   ```yaml
     stop_after_bond_verify: false   # remove (the comment lines above it too)
     water_level:                    # remove: replaced by pump_load (step 2)
       name: "Estimated Water Level"
     error_word:                     # remove: decoded alarms are in errors_text
       name: "Error Word"
     status_word:                    # remove: decoded into the entities of step 2
       name: "Status Word"
     start_pressure_setpoint:        # remove: start pressure is read-only
       name: "Start Pressure Setpoint"
   ```

2. **Add** these keys under `ebara_hydrostation:` — keep the names exactly as
   written, the Home Assistant integration looks the entities up by name:

   ```yaml
     pump_status:
       name: "Pump Status"
     external_stop:
       name: "External Stop"
     status_bit10:
       name: "Status Bit 10 (undocumented)"
     pressure_unit:
       name: "Pressure Unit"
     device_name:
       name: "Pump Device Name"
     pump_load:
       name: "Estimated Pump Load"
   ```

3. Optional: `update_interval` is now the period of the live values only
   (default and suggested value `10s`, allowed 2–300 s), and it is the
   default of the "Update Interval" entity. If you already changed that
   entity from Home Assistant, the stored value keeps winning.

Nothing else changes: `esp32` / `sdkconfig_options` stay as they are (the
NimBLE host task stack is now enlarged by the component itself), and the pump
bond and target MAC stored on the ESP32 are kept. After the update the
gateway reconnects to the pump by itself; the first connection after the
flash may need a second attempt a few seconds later.

### Changed — behaviour aligned with the official app (2.0.10015)

- **Pump family check**: the pump's Bluetooth device name is read before
  subscribing; only HYDRO pumps are polled. HYSTA pumps (a different,
  push-based protocol) are reported as `Unsupported device` and flagged
  `"supported": false` in the discovered list.
- **psi pumps**: status bit 9 (pressure unit) is now honoured. Pressures are
  converted to bar and setpoints are sent in psi×10 with the app's 2-psi step
  and limits. Before, a pump configured in psi showed values 14.5 times too
  high and setpoints were written in the wrong unit.
- **Setpoint limits** are the app's HYDRO limits: target 2.0–5.5 bar, restart
  delta 0.5–2.0 bar.
- **Start pressure setpoint removed**: the app never writes `sm-0002`; start
  pressure stays available read-only (it equals target − delta).
- **Firmware version** is shown as the app does: `01.04` (high/low byte)
  instead of `2.60`. It is now a text sensor.
- **Estimated Water Level → Estimated Pump Load**: it is the app's
  "FREQUENCY SPEED" estimate from the motor current, not a water level; the
  first threshold is fixed (0.1–0.9 A now reads 20 % as in the app).
- **Motor frequency** is published raw, without the unconfirmed "Hz" unit.
- **Alarms**: `Errors` lists the app's codes and texts (E01…E06, H01, H01+,
  H02, C01; H01 text in psi on psi pumps) and only while the alarm bit of the
  status word is set, like the app. Codes are no longer reported while no
  alarm is active.
- **Motor command** is refused while an alarm is active (the app sends
  nothing in that case).
- **Serial number** `4294967295` (not programmed) is shown as `not programmed`.
- SET acknowledgements of both app generations (`SET DATO OK`, `DATOOK`) are
  recognised.

### Added

- Entities `pump_status` (Running, Standby, Assist Mode, External Stop, Alarm,
  Pump Off, Offline, with the app's priority), `external_stop`,
  `status_bit10` (raw, not decoded by the app), `pressure_unit`,
  `device_name`, `pump_load`.
- **Two-speed polling**: live values (status, pressures, motor current,
  alarms) every update interval — minimum lowered from 5 s to 2 s — and
  slow-changing values (working hours, module temperature, DC bus voltage,
  start pressure) every 60 s. After a SET the new values are read back at once.
- Live values become **unknown** while the pump is unreachable or the gateway
  is disabled, instead of keeping the last reading.
- Component version shown in the ESPHome log at boot.

### Fixed

- **Bond lost at every ESP32 reboot**: the gateway used a new random address
  at each boot, so the pump rejected the stored keys and a new pairing was
  made every time. It now uses a static random address generated once and
  kept in flash (the pump only forwards data to a random-type address).
- Valid bond deleted after a duplicated encryption start (MIC failure): the
  gateway no longer starts encryption on a link that is already encrypted and
  keeps the bond on MIC failures.
- **Crashes** (stack overflow in the NimBLE task, out-of-memory resets): all
  Bluetooth events are now handled in the ESPHome main loop.
- **Build failure with ESPHome 2026.10+** (`host/ble_gap.h` not found).
- First session after a reboot receiving no data: the link is now closed
  cleanly on reboot and the first connection waits for the pump to release
  the previous one; the extra "priming" connection is gone.
- The `update_interval` in the YAML was ignored (a fixed 15 s was used), and
  changes to the "Update Interval" entity only applied after a reboot: both
  now take effect immediately.
- `default_target_mac` no longer overrides, at every boot, the pump chosen
  from Home Assistant.
- Reply handling: notifications are reassembled until the terminator, the
  shape of each reply is checked, the command timeout is 3 s like the app and
  a late reply after a timeout can no longer be attributed to the next
  command.
- Setpoints and the motor switch no longer show a value that was not sent.
- Gateway disable / target-MAC changes now also stop a connection in progress;
  a stalled session setup is aborted and retried after 30 s; SETs are sent in
  the order requested (the latest value wins for the same setpoint).

### Removed

- `stop_after_bond_verify` (bond-test mode), the `water_level`, `error_word`,
  `status_word` and `start_pressure_setpoint` entities. The keys are still
  accepted with a warning so that existing configurations keep compiling.
