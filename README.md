# Ebara Hydrostation ESPHome Gateway

An [ESPHome](https://esphome.io) external component that turns an ESP32 into
a Bluetooth gateway for the **Ebara Hydrostation**, a constant-pressure water
pump that's only ever been controllable through its manufacturer's Android
app. There's no official local protocol, so I reverse-engineered it from a
real pump (BLE captures, decompiling the app, testing against the actual
hardware) and built this component to bond with it, read its sensors, and
control it — all without touching the app.

If you're looking for the Home Assistant side of this (the nice dashboard
with proper entities, not just raw ESPHome sensors), that's a separate,
companion project:
**[ebara_hydrostation](https://github.com/Danny89530/ebara_hydrostation)**.
This repository is just the gateway — it's meant to run standalone on an
ESP32 and expose its data over ESPHome's native API, which the Home
Assistant integration then consumes.

## What it does

The pump's Bluetooth module (a Microchip BM70/BM71, "Transparent UART" mode)
requires a proper LE Secure Connections bond before it'll answer any command,
and has its own quirks around connection timing and command sequencing that
most generic BLE integrations won't get right on the first try. This
component owns the whole thing end to end:

- Bonds with the pump (Numeric Comparison pairing — confirmed automatically
  on the ESP32 side, no button to press).
- Persists the bond and the pump's MAC address to flash, so it survives ESP
  reboots without needing Home Assistant to re-send anything.
- Checks the pump family by its Bluetooth device name: only **HYDRO** pumps
  are supported. HYSTA pumps speak a different protocol and are reported as
  unsupported instead of being polled.
- Discovers the pump's GATT database, subscribes to its notifications, and
  runs a sequential command queue (one command in flight at a time):
  - live values (status, pressures, motor current, alarms) every update
    interval;
  - slow-changing values (working hours, module temperature, DC bus voltage,
    start pressure) every 60 s;
  - firmware/hardware version and serial number once per pump.
- Decodes the status and alarm words exactly like the official app: pump
  status (Running, Standby, Assist Mode, External Stop, Alarm, Pump Off),
  motor flags, pressure unit and the active alarm codes (E01…E06, H01, H01+,
  H02, C01).
- Lets you set the target pressure and the restart delta (sent in the pump's
  own unit, bar or psi, with the app's limits) and enable/disable the pump's
  automatic control. Like the app, the motor command is refused while an
  alarm is active.
- Reports every live value as unknown while the pump is unreachable, so stale
  readings are never shown as current.
- Exposes a BLE scan mode (used during first-time setup) that lists nearby
  Hydrostation pumps by name/MAC/signal strength, so you don't need to know
  the MAC address up front.

Everything is exposed as normal ESPHome entities (sensors, binary sensors,
switches, numbers, text sensors) — you can use them directly if you want,
but they're really meant to be picked up by the companion Home Assistant
integration linked above, which wraps them in something much friendlier.

## Hardware requirements

- Any ESP32 board (this was built and tested against a plain `esp32dev`).
- ESPHome 2026.10 or newer is what this is tested with.
- Built on **esp-idf**, not Arduino — this component needs esp-idf's NimBLE
  Bluetooth stack, not the Bluedroid stack ESPHome's own Bluetooth
  components use. Bluedroid must be explicitly disabled in favor of NimBLE
  (see the sdkconfig options below). The component also enlarges the NimBLE
  host task stack by itself (`CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE`), unless
  you set that option yourself.
- Don't add ESPHome's own `esp32_ble_tracker` / `bluetooth_proxy` to the same
  ESP32: this component owns the Bluetooth stack.
- WiFi in range of the ESP32, and the pump in Bluetooth range of it.

## Installation

Add this repository as an external component in your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Danny89530/esphome-ebara-hydrostation
      path: components
    refresh: 1d
```

`refresh` controls how often ESPHome checks this repository for a newer
version when you compile (use `0s` to always fetch the latest).

Then set up the ESP32 with esp-idf and NimBLE:

```yaml
esphome:
  name: my-hydro-gw

esp32:
  board: esp32dev
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_BT_ENABLED: "y"
      CONFIG_BT_NIMBLE_ENABLED: "y"
      CONFIG_BT_BLUEDROID_ENABLED: "n"
      CONFIG_BT_NIMBLE_NVS_PERSIST: "y"

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
  encryption:
    key: !secret api_key

ota:
  - platform: esphome
    password: !secret ota_password
```

You'll need a `secrets.yaml` with `wifi_ssid`, `wifi_password`, `api_key`,
and `ota_password`.

## Configuring the component itself

```yaml
ebara_hydrostation:
  update_interval: 10s
  target_mac:
    name: "Target MAC"
  gw_status:
    name: "GW Status"
  actual_pressure:
    name: "Actual Pressure"
  # ... one entry per entity you want exposed — see the full example
  # in ebara_hydro_gw.yaml in this repo for every available entity.
```

Every sensor, switch, and number the component can expose is configured the
same way: a key under `ebara_hydrostation:` with a `name:`. You don't have
to configure all of them — anything you leave out just won't be created.

`update_interval` controls how often the live values are read (default 10s,
allowed 2–300 s); slow-changing values are read every 60 s regardless. It is
only the default of the "Update Interval" number entity: if you expose that
entity, the value you set there is stored on the ESP32 and takes effect
immediately, without reflashing.

The Home Assistant integration looks the gateway entities up **by name**, so
keep the names used in `ebara_hydro_gw.yaml`.

### Available entities

| Key | Type | Notes |
|---|---|---|
| `target_mac`, `gw_status` | text / text sensor | Required. Pump MAC (set by the HA integration) and gateway state |
| `discovered` | text sensor | Pumps found by the BLE scan (JSON, HYSTA flagged `supported: false`) |
| `pump_status` | text sensor | Running, Standby, Assist Mode, External Stop, Alarm, Pump Off, Offline |
| `errors_text` | text sensor | Active alarms with code and text, `No active alarms` otherwise |
| `motor_running`, `motor_enabled`, `motor_error`, `external_stop` | binary sensor | Decoded status word |
| `status_bit10` | binary sensor | Status bit not decoded by the app; exposed raw (diagnostic) |
| `pressure_unit`, `device_name` | text sensor | Unit configured on the pump (bar/psi) and its Bluetooth name |
| `actual_pressure`, `target_pressure`, `delta_pressure`, `start_pressure` | sensor | Always in bar (converted when the pump uses psi); start = target − delta |
| `motor_current`, `pump_load` | sensor | Current in A; load estimate in % (the app's "FREQUENCY SPEED") |
| `motor_frequency` | sensor | Raw value: its unit is not confirmed by the app |
| `module_temperature`, `dc_bus_voltage`, `working_hours` | sensor | °C, V, h |
| `firmware_version`, `hardware_version`, `serial_number` | text sensor / sensor | Firmware shown as `HH.LL` like the app |
| `motor` | switch | Automatic control of the pump on/off (not "motor spinning now") |
| `enable` | switch | Gateway on/off: OFF closes the Bluetooth link and stops polling |
| `target_pressure_setpoint`, `delta_pressure_setpoint` | number | 2.0–5.5 bar and 0.5–2.0 bar (app limits) |
| `poll_interval` | number | Live-values update interval, 2–300 s |

## Upgrading

See [CHANGELOG.md](CHANGELOG.md): it lists, for each version, exactly what to
change in your YAML.

You don't need to set a target MAC in the YAML at all — on first boot with
no MAC configured, the component starts scanning and publishes discovered
pumps to a text entity, and you pick one either straight from that entity's
state or, more conveniently, through the companion Home Assistant
integration's setup wizard. Once picked, the MAC is written to the ESP32's
own flash and it'll reconnect to that pump on every boot from then on,
independent of Home Assistant.

## A note on scope

Everything shipped here is a command I could actually confirm works against
real hardware — verified against the pump's own official app (decompiled)
and, where possible, a real Bluetooth capture, then tested live. I've
deliberately left out a couple of things the app supports (twin/dual-pump
mode, a couple of extra diagnostic registers) because I could never get a
reliable way to tell "not supported by this firmware" apart from "wrong
command" — rather than ship a guess, I left them out. If you're digging
into this yourself, don't try random command variants against a real pump;
figure out the format from the app first.

This project isn't affiliated with, endorsed by, or supported by Ebara in
any way. Use it at your own risk — this is unofficial, reverse-engineered,
and comes with zero guarantees.
