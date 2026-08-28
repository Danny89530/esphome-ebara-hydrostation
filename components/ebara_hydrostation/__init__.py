# Ebara Hydrostation Gateway — ESPHome component configuration (NimBLE).
#
# This component does not depend on esp32_ble_tracker / esp32_ble_client: it
# owns the entire NimBLE stack itself (see setup() in ebara_hydrostation.cpp).
# It requires the esp-idf framework with Bluedroid disabled and NimBLE enabled
# (see ebara_hydro_gw.yaml's esp32.framework.sdkconfig_options).

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    number,
    sensor,
    switch,
    text,
    text_sensor,
)
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["text", "text_sensor", "sensor", "binary_sensor", "switch", "number"]

ebara_hydro_ns = cg.esphome_ns.namespace("ebara_hydrostation")
EbaraHydrostationGateway = ebara_hydro_ns.class_(
    "EbaraHydrostationGateway", cg.PollingComponent
)
EbaraTargetMacText = ebara_hydro_ns.class_(
    "EbaraTargetMacText", text.Text, cg.Component
)
EbaraMotorSwitch = ebara_hydro_ns.class_(
    "EbaraMotorSwitch", switch.Switch, cg.Component
)
EbaraEnableSwitch = ebara_hydro_ns.class_(
    "EbaraEnableSwitch", switch.Switch, cg.Component
)
EbaraSetpointNumber = ebara_hydro_ns.class_(
    "EbaraSetpointNumber", number.Number, cg.Component
)
EbaraUpdateIntervalNumber = ebara_hydro_ns.class_(
    "EbaraUpdateIntervalNumber", number.Number, cg.Component
)
EbaraSetpointKind = ebara_hydro_ns.enum("EbaraSetpointKind", is_class=True)

# Must match kUpdateIntervalMinS/kUpdateIntervalMaxS in ebara_hydrostation.cpp.
UPDATE_INTERVAL_MIN_S = 5
UPDATE_INTERVAL_MAX_S = 300

CONF_STOP_AFTER_BOND_VERIFY = "stop_after_bond_verify"
CONF_DEFAULT_TARGET_MAC = "default_target_mac"

CONF_TARGET_MAC = "target_mac"
CONF_GW_STATUS = "gw_status"
CONF_ERRORS_TEXT = "errors_text"
CONF_DISCOVERED = "discovered"

CONF_TARGET_PRESSURE = "target_pressure"
CONF_START_PRESSURE = "start_pressure"
CONF_ACTUAL_PRESSURE = "actual_pressure"
CONF_MOTOR_CURRENT = "motor_current"
CONF_WORKING_HOURS = "working_hours"
CONF_MOTOR_FREQUENCY = "motor_frequency"
CONF_MODULE_TEMPERATURE = "module_temperature"
CONF_DC_BUS_VOLTAGE = "dc_bus_voltage"
CONF_DELTA_PRESSURE = "delta_pressure"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_HARDWARE_VERSION = "hardware_version"
CONF_WATER_LEVEL = "water_level"
CONF_ERROR_WORD = "error_word"
CONF_STATUS_WORD = "status_word"

CONF_SERIAL_NUMBER = "serial_number"

CONF_MOTOR_RUNNING = "motor_running"
CONF_MOTOR_ENABLED = "motor_enabled"
CONF_MOTOR_ERROR = "motor_error"

CONF_MOTOR = "motor"
CONF_ENABLE = "enable"

CONF_TARGET_PRESSURE_SETPOINT = "target_pressure_setpoint"
CONF_START_PRESSURE_SETPOINT = "start_pressure_setpoint"
CONF_DELTA_PRESSURE_SETPOINT = "delta_pressure_setpoint"

CONF_POLL_INTERVAL = "poll_interval"

TARGET_MAC_SCHEMA = text.text_schema(EbaraTargetMacText, mode="text").extend(
    cv.COMPONENT_SCHEMA
)
MOTOR_SWITCH_SCHEMA = switch.switch_schema(EbaraMotorSwitch).extend(
    cv.COMPONENT_SCHEMA
)
ENABLE_SWITCH_SCHEMA = switch.switch_schema(
    EbaraEnableSwitch, default_restore_mode="RESTORE_DEFAULT_ON"
).extend(cv.COMPONENT_SCHEMA)

# Allowed (min_value, max_value, step) per setpoint, in bar.
SETPOINT_RANGES = {
    CONF_TARGET_PRESSURE_SETPOINT: (2.0, 5.5, 0.1),
    CONF_START_PRESSURE_SETPOINT: (1.0, 5.5, 0.1),
    CONF_DELTA_PRESSURE_SETPOINT: (0.5, 2.0, 0.1),
}


def setpoint_number_schema():
    return number.number_schema(EbaraSetpointNumber).extend(cv.COMPONENT_SCHEMA)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EbaraHydrostationGateway),
        cv.Optional(CONF_STOP_AFTER_BOND_VERIFY, default=True): cv.boolean,
        cv.Optional(CONF_DEFAULT_TARGET_MAC): cv.string_strict,
        cv.Required(CONF_TARGET_MAC): TARGET_MAC_SCHEMA,
        cv.Required(CONF_GW_STATUS): text_sensor.text_sensor_schema().extend(
            cv.COMPONENT_SCHEMA
        ),
        cv.Optional(CONF_ERRORS_TEXT): text_sensor.text_sensor_schema().extend(
            cv.COMPONENT_SCHEMA
        ),
        cv.Optional(CONF_DISCOVERED): text_sensor.text_sensor_schema().extend(
            cv.COMPONENT_SCHEMA
        ),
        cv.Optional(CONF_TARGET_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1
        ),
        cv.Optional(CONF_START_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1
        ),
        cv.Optional(CONF_ACTUAL_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1
        ),
        cv.Optional(CONF_MOTOR_CURRENT): sensor.sensor_schema(
            unit_of_measurement="A", accuracy_decimals=1
        ),
        cv.Optional(CONF_WORKING_HOURS): sensor.sensor_schema(
            unit_of_measurement="h", accuracy_decimals=0
        ),
        cv.Optional(CONF_MOTOR_FREQUENCY): sensor.sensor_schema(
            unit_of_measurement="Hz", accuracy_decimals=0
        ),
        cv.Optional(CONF_MODULE_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement="°C", accuracy_decimals=0
        ),
        cv.Optional(CONF_DC_BUS_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement="V", accuracy_decimals=0
        ),
        cv.Optional(CONF_DELTA_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1
        ),
        cv.Optional(CONF_FIRMWARE_VERSION): sensor.sensor_schema(
            accuracy_decimals=2
        ),
        cv.Optional(CONF_HARDWARE_VERSION): sensor.sensor_schema(
            accuracy_decimals=0
        ),
        cv.Optional(CONF_WATER_LEVEL): sensor.sensor_schema(
            unit_of_measurement="%", accuracy_decimals=0
        ),
        cv.Optional(CONF_ERROR_WORD): sensor.sensor_schema(accuracy_decimals=0),
        cv.Optional(CONF_STATUS_WORD): sensor.sensor_schema(accuracy_decimals=0),
        cv.Optional(CONF_SERIAL_NUMBER): text_sensor.text_sensor_schema().extend(
            cv.COMPONENT_SCHEMA
        ),
        cv.Optional(CONF_MOTOR_RUNNING): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_MOTOR_ENABLED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_MOTOR_ERROR): binary_sensor.binary_sensor_schema(
            device_class="problem"
        ),
        cv.Optional(CONF_MOTOR): MOTOR_SWITCH_SCHEMA,
        cv.Optional(CONF_ENABLE): ENABLE_SWITCH_SCHEMA,
        cv.Optional(CONF_TARGET_PRESSURE_SETPOINT): setpoint_number_schema(),
        cv.Optional(CONF_START_PRESSURE_SETPOINT): setpoint_number_schema(),
        cv.Optional(CONF_DELTA_PRESSURE_SETPOINT): setpoint_number_schema(),
        cv.Optional(CONF_POLL_INTERVAL): number.number_schema(
            EbaraUpdateIntervalNumber
        ).extend(cv.COMPONENT_SCHEMA),
    }
).extend(cv.polling_component_schema("15s"))

SETPOINT_KIND_MAP = {
    CONF_TARGET_PRESSURE_SETPOINT: EbaraSetpointKind.TARGET_PRESSURE,
    CONF_START_PRESSURE_SETPOINT: EbaraSetpointKind.START_PRESSURE,
    CONF_DELTA_PRESSURE_SETPOINT: EbaraSetpointKind.DELTA_PRESSURE,
}

SENSOR_SETTERS = {
    CONF_TARGET_PRESSURE: "set_target_pressure_sensor",
    CONF_START_PRESSURE: "set_start_pressure_sensor",
    CONF_ACTUAL_PRESSURE: "set_actual_pressure_sensor",
    CONF_MOTOR_CURRENT: "set_motor_current_sensor",
    CONF_WORKING_HOURS: "set_working_hours_sensor",
    CONF_MOTOR_FREQUENCY: "set_motor_frequency_sensor",
    CONF_MODULE_TEMPERATURE: "set_module_temperature_sensor",
    CONF_DC_BUS_VOLTAGE: "set_dc_bus_voltage_sensor",
    CONF_DELTA_PRESSURE: "set_delta_pressure_sensor",
    CONF_FIRMWARE_VERSION: "set_firmware_version_sensor",
    CONF_HARDWARE_VERSION: "set_hardware_version_sensor",
    CONF_WATER_LEVEL: "set_water_level_sensor",
    CONF_ERROR_WORD: "set_error_word_sensor",
    CONF_STATUS_WORD: "set_status_word_sensor",
}

BINARY_SENSOR_SETTERS = {
    CONF_MOTOR_RUNNING: "set_motor_running_binary_sensor",
    CONF_MOTOR_ENABLED: "set_motor_enabled_binary_sensor",
    CONF_MOTOR_ERROR: "set_motor_error_binary_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)

    cg.add(var.set_stop_after_bond_verify(config[CONF_STOP_AFTER_BOND_VERIFY]))
    if (default_mac := config.get(CONF_DEFAULT_TARGET_MAC)) is not None:
        cg.add(var.set_default_target_mac(default_mac))

    # Target MAC text input.
    mac_conf = config[CONF_TARGET_MAC]
    mac_var = cg.new_Pvariable(mac_conf[cv.GenerateID()])
    await cg.register_component(mac_var, mac_conf)
    await text.register_text(mac_var, mac_conf)
    cg.add(mac_var.set_parent(var))
    cg.add(var.set_target_mac_entity(mac_var))

    # Gateway status text sensor.
    st_var = await text_sensor.new_text_sensor(config[CONF_GW_STATUS])
    cg.add(var.set_gw_status(st_var))

    if errors_conf := config.get(CONF_ERRORS_TEXT):
        errors_var = await text_sensor.new_text_sensor(errors_conf)
        cg.add(var.set_errors_text_sensor(errors_var))

    if discovered_conf := config.get(CONF_DISCOVERED):
        discovered_var = await text_sensor.new_text_sensor(discovered_conf)
        cg.add(var.set_discovered_text_sensor(discovered_var))

    if serial_conf := config.get(CONF_SERIAL_NUMBER):
        serial_var = await text_sensor.new_text_sensor(serial_conf)
        cg.add(var.set_serial_number_text_sensor(serial_var))

    for key, setter in SENSOR_SETTERS.items():
        if (conf := config.get(key)) is not None:
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(var, setter)(sens))

    for key, setter in BINARY_SENSOR_SETTERS.items():
        if (conf := config.get(key)) is not None:
            bsens = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(var, setter)(bsens))

    if motor_conf := config.get(CONF_MOTOR):
        motor_var = await switch.new_switch(motor_conf)
        cg.add(motor_var.set_parent(var))
        cg.add(var.set_motor_switch(motor_var))

    if enable_conf := config.get(CONF_ENABLE):
        enable_var = await switch.new_switch(enable_conf)
        # Also registered as a plain Component (switch.new_switch alone only
        # registers it as a switch entity) so its setup() override — which
        # applies the persisted/default restore-mode state — actually runs.
        await cg.register_component(enable_var, enable_conf)
        cg.add(enable_var.set_parent(var))

    for key, kind in SETPOINT_KIND_MAP.items():
        if (conf := config.get(key)) is None:
            continue
        min_v, max_v, step = SETPOINT_RANGES[key]
        num_var = await number.new_number(
            conf, min_value=min_v, max_value=max_v, step=step
        )
        cg.add(num_var.set_parent(var))
        cg.add(num_var.set_kind(kind))
        setter = {
            CONF_TARGET_PRESSURE_SETPOINT: "set_target_pressure_number",
            CONF_START_PRESSURE_SETPOINT: "set_start_pressure_number",
            CONF_DELTA_PRESSURE_SETPOINT: "set_delta_pressure_number",
        }[key]
        cg.add(getattr(var, setter)(num_var))

    if poll_interval_conf := config.get(CONF_POLL_INTERVAL):
        poll_interval_var = await number.new_number(
            poll_interval_conf,
            min_value=UPDATE_INTERVAL_MIN_S,
            max_value=UPDATE_INTERVAL_MAX_S,
            step=1,
        )
        # setup() applies the persisted/default interval — must be a
        # registered Component for that to actually run (see EbaraEnableSwitch).
        await cg.register_component(poll_interval_var, poll_interval_conf)
        cg.add(poll_interval_var.set_parent(var))
