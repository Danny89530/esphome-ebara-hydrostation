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
- Discovers the pump's GATT database, subscribes to its notifications, and
  runs a sequential poll loop reading pressure, motor status, temperature,
  voltage, working hours, firmware/hardware version, and more.
- Lets you write pressure setpoints and start/stop the motor.
- Exposes a BLE scan mode (used during first-time setup) that lists nearby
  Hydrostation pumps by name/MAC/signal strength, so you don't need to know
  the MAC address up front.

Everything is exposed as normal ESPHome entities (sensors, binary sensors,
switches, numbers, text sensors) — you can use them directly if you want,
but they're really meant to be picked up by the companion Home Assistant
integration linked above, which wraps them in something much friendlier.

## Hardware requirements

- Any ESP32 board (this was built and tested against a plain `esp32dev`).
- Built on **esp-idf**, not Arduino — this component needs esp-idf's NimBLE
  Bluetooth stack, not the Bluedroid stack ESPHome's own Bluetooth
  components use, since Bluedroid could never establish a reliable GATT
  session with this pump's Bluetooth module. Bluedroid must be explicitly
  disabled in favor of NimBLE (see the sdkconfig options below).
- WiFi in range of the ESP32, and the pump in Bluetooth range of it.

## Installation

Add this repository as an external component in your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Danny89530/esphome-ebara-hydrostation
      path: components
```

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
  update_interval: 15s
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

`update_interval` controls how often the whole poll cycle runs (default
15s); it can also be changed at runtime through the "Update Interval" number
entity if you expose it, without reflashing.

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
