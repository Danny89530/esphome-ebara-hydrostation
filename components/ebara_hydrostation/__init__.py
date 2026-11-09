# Ebara Hydrostation Gateway — ESPHome component configuration (NimBLE).
#
# Owns the entire NimBLE stack itself (no esp32_ble_tracker/esp32_ble_client).
# Requires the esp-idf framework with Bluedroid disabled and NimBLE enabled.
# Only Ebara HYDRO pumps (legacy gm-/sm- protocol) are supported; HYSTA pumps
# are detected and reported as unsupported.

import logging

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
from esphome.const import CONF_UPDATE_INTERVAL, ENTITY_CATEGORY_DIAGNOSTIC

_LOGGER = logging.getLogger(__name__)

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
UPDATE_INTERVAL_MIN_S = 2
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
CONF_PUMP_LOAD = "pump_load"
CONF_ERROR_WORD = "error_word"
CONF_STATUS_WORD = "status_word"
CONF_SERIAL_NUMBER = "serial_number"
CONF_PUMP_STATUS = "pump_status"
CONF_PRESSURE_UNIT = "pressure_unit"
CONF_DEVICE_NAME = "device_name"

CONF_MOTOR_RUNNING = "motor_running"
CONF_MOTOR_ENABLED = "motor_enabled"
CONF_MOTOR_ERROR = "motor_error"
CONF_EXTERNAL_STOP = "external_stop"
CONF_STATUS_BIT10 = "status_bit10"

CONF_MOTOR = "motor"
CONF_ENABLE = "enable"

CONF_TARGET_PRESSURE_SETPOINT = "target_pressure_setpoint"
CONF_DELTA_PRESSURE_SETPOINT = "delta_pressure_setpoint"
CONF_POLL_INTERVAL = "poll_interval"

# Keys of version 1.x that no longer exist: still accepted (and ignored) so an
# existing configuration keeps compiling, with a warning explaining what to do.
CONF_WATER_LEVEL = "water_level"
CONF_START_PRESSURE_SETPOINT = "start_pressure_setpoint"

TARGET_MAC_SCHEMA = text.text_schema(EbaraTargetMacText, mode="text").extend(
    cv.COMPONENT_SCHEMA
)
MOTOR_SWITCH_SCHEMA = switch.switch_schema(EbaraMotorSwitch).extend(
    cv.COMPONENT_SCHEMA
)
ENABLE_SWITCH_SCHEMA = switch.switch_schema(
    EbaraEnableSwitch, default_restore_mode="RESTORE_DEFAULT_ON"
).extend(cv.COMPONENT_SCHEMA)

# Allowed (min_value, max_value, step) per setpoint, in bar — HYDRO limits of
# the official app (target 20..55, delta 5..20 in bar×10). In psi mode the
# value is converted and snapped to the app's 2-psi step before sending.
SETPOINT_RANGES = {
    CONF_TARGET_PRESSURE_SETPOINT: (2.0, 5.5, 0.1),
    CONF_DELTA_PRESSURE_SETPOINT: (0.5, 2.0, 0.1),
}


def _deprecated_stop_after_bond_verify(value):
    _LOGGER.warning(
        "'%s' is deprecated and ignored: the bond is verified at every connection. "
        "Remove it from the configuration.",
        CONF_STOP_AFTER_BOND_VERIFY,
    )
    return cv.boolean(value)


def _removed_key(key, hint):
    """Accept a removed entity key, ignore it and tell the user what to do."""

    def validator(value):
        _LOGGER.warning("'%s' was removed and is ignored: %s", key, hint)
        return value

    return validator


def _text_sensor(**kwargs):
    return text_sensor.text_sensor_schema(**kwargs).extend(cv.COMPONENT_SCHEMA)


def setpoint_number_schema():
    return number.number_schema(EbaraSetpointNumber).extend(cv.COMPONENT_SCHEMA)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EbaraHydrostationGateway),
        cv.Optional(CONF_STOP_AFTER_BOND_VERIFY): _deprecated_stop_after_bond_verify,
        # Only seeds a gateway that has never been configured: a MAC chosen
        # later from Home Assistant (persisted) takes precedence.
        cv.Optional(CONF_DEFAULT_TARGET_MAC): cv.mac_address,
        cv.Required(CONF_TARGET_MAC): TARGET_MAC_SCHEMA,
        cv.Required(CONF_GW_STATUS): _text_sensor(),
        cv.Optional(CONF_ERRORS_TEXT): _text_sensor(),
        cv.Optional(CONF_DISCOVERED): _text_sensor(),
        # Pressures are always published in bar (converted from psi×10 when
        # the pump is configured in psi, status bit 9).
        cv.Optional(CONF_TARGET_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1, device_class="pressure",
            state_class="measurement",
        ),
        cv.Optional(CONF_START_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1, device_class="pressure",
            state_class="measurement",
        ),
        cv.Optional(CONF_ACTUAL_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1, device_class="pressure",
            state_class="measurement",
        ),
        cv.Optional(CONF_DELTA_PRESSURE): sensor.sensor_schema(
            unit_of_measurement="bar", accuracy_decimals=1, device_class="pressure",
            state_class="measurement",
        ),
        cv.Optional(CONF_MOTOR_CURRENT): sensor.sensor_schema(
            unit_of_measurement="A", accuracy_decimals=1, device_class="current",
            state_class="measurement",
        ),
        cv.Optional(CONF_WORKING_HOURS): sensor.sensor_schema(
            unit_of_measurement="h", accuracy_decimals=0, state_class="total_increasing",
        ),
        # Unit not confirmed by the official app (never displayed): raw value.
        cv.Optional(CONF_MOTOR_FREQUENCY): sensor.sensor_schema(
            accuracy_decimals=0, state_class="measurement",
        ),
        cv.Optional(CONF_MODULE_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement="°C", accuracy_decimals=0, device_class="temperature",
            state_class="measurement",
        ),
        cv.Optional(CONF_DC_BUS_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement="V", accuracy_decimals=0, device_class="voltage",
            state_class="measurement",
        ),
        # App "FREQUENCY SPEED" %: load estimate from the motor current.
        cv.Optional(CONF_PUMP_LOAD): sensor.sensor_schema(
            unit_of_measurement="%", accuracy_decimals=0, state_class="measurement",
        ),
        cv.Optional(CONF_FIRMWARE_VERSION): _text_sensor(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_HARDWARE_VERSION): sensor.sensor_schema(
            accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_SERIAL_NUMBER): _text_sensor(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_PUMP_STATUS): _text_sensor(),
        cv.Optional(CONF_PRESSURE_UNIT): _text_sensor(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_DEVICE_NAME): _text_sensor(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_MOTOR_RUNNING): binary_sensor.binary_sensor_schema(
            device_class="running"
        ),
        cv.Optional(CONF_MOTOR_ENABLED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_MOTOR_ERROR): binary_sensor.binary_sensor_schema(
            device_class="problem"
        ),
        cv.Optional(CONF_EXTERNAL_STOP): binary_sensor.binary_sensor_schema(),
        # Status word bit 10: not decoded by the official app nor documented.
        cv.Optional(CONF_STATUS_BIT10): binary_sensor.binary_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_MOTOR): MOTOR_SWITCH_SCHEMA,
        cv.Optional(CONF_ENABLE): ENABLE_SWITCH_SCHEMA,
        cv.Optional(CONF_TARGET_PRESSURE_SETPOINT): setpoint_number_schema(),
        cv.Optional(CONF_DELTA_PRESSURE_SETPOINT): setpoint_number_schema(),
        cv.Optional(CONF_POLL_INTERVAL): number.number_schema(
            EbaraUpdateIntervalNumber
        ).extend(cv.COMPONENT_SCHEMA),
        cv.Optional(CONF_WATER_LEVEL): _removed_key(
            CONF_WATER_LEVEL,
            "it is the app's 'FREQUENCY SPEED' load estimate: use 'pump_load' instead.",
        ),
        cv.Optional(CONF_ERROR_WORD): _removed_key(
            CONF_ERROR_WORD, "the decoded alarms are in 'errors_text'."
        ),
        cv.Optional(CONF_STATUS_WORD): _removed_key(
            CONF_STATUS_WORD,
            "use 'pump_status', 'motor_running', 'motor_enabled', 'motor_error', "
            "'external_stop' and 'pressure_unit'.",
        ),
        cv.Optional(CONF_START_PRESSURE_SETPOINT): _removed_key(
            CONF_START_PRESSURE_SETPOINT,
            "the start pressure is read-only (target - delta), as in the official app.",
        ),
    }
).extend(cv.polling_component_schema("10s"))

SETPOINT_KIND_MAP = {
    CONF_TARGET_PRESSURE_SETPOINT: (EbaraSetpointKind.TARGET_PRESSURE, "set_target_pressure_number"),
    CONF_DELTA_PRESSURE_SETPOINT: (EbaraSetpointKind.DELTA_PRESSURE, "set_delta_pressure_number"),
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
    CONF_HARDWARE_VERSION: "set_hardware_version_sensor",
    CONF_PUMP_LOAD: "set_pump_load_sensor",
}

TEXT_SENSOR_SETTERS = {
    CONF_ERRORS_TEXT: "set_errors_text_sensor",
    CONF_DISCOVERED: "set_discovered_text_sensor",
    CONF_SERIAL_NUMBER: "set_serial_number_text_sensor",
    CONF_FIRMWARE_VERSION: "set_firmware_version_text_sensor",
    CONF_PUMP_STATUS: "set_pump_status_text_sensor",
    CONF_PRESSURE_UNIT: "set_pressure_unit_text_sensor",
    CONF_DEVICE_NAME: "set_device_name_text_sensor",
}

BINARY_SENSOR_SETTERS = {
    CONF_MOTOR_RUNNING: "set_motor_running_binary_sensor",
    CONF_MOTOR_ENABLED: "set_motor_enabled_binary_sensor",
    CONF_MOTOR_ERROR: "set_motor_error_binary_sensor",
    CONF_EXTERNAL_STOP: "set_external_stop_binary_sensor",
    CONF_STATUS_BIT10: "set_status_bit10_binary_sensor",
}


def _set_nimble_host_stack_size():
    # Margin over the 4096-byte default of the nimble_host task. Skipped if the
    # user already set the option in sdkconfig_options.
    try:
        from esphome.components.esp32 import add_idf_sdkconfig_option
    except ImportError:
        return
    from esphome.core import CORE

    user_opts = (
        CORE.config.get("esp32", {}).get("framework", {}).get("sdkconfig_options", {})
        if CORE.config
        else {}
    )
    if "CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE" not in user_opts:
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE", 6144)


def _request_bluetooth_idf_components():
    # ESPHome >= 2026.10 excludes unused built-in ESP-IDF components; "bt"
    # (NimBLE headers/libs) is only re-included when a component requests it.
    # This component owns NimBLE directly, so it must do what esp32_ble does.
    try:
        from esphome.components.esp32 import (
            request_bluetooth,
            request_software_coexistence,
        )
    except ImportError:  # older ESPHome: nothing is excluded by default
        return
    request_bluetooth()
    request_software_coexistence()


async def to_code(config):
    _request_bluetooth_idf_components()
    _set_nimble_host_stack_size()
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)

    # The YAML update_interval is the default of the "Update Interval" entity
    # (live-values poll period) until the user changes it.
    cg.add(var.set_default_update_interval_s(config[CONF_UPDATE_INTERVAL].total_milliseconds / 1000.0))
    if (default_mac := config.get(CONF_DEFAULT_TARGET_MAC)) is not None:
        cg.add(var.set_default_target_mac(str(default_mac)))

    mac_conf = config[CONF_TARGET_MAC]
    mac_var = cg.new_Pvariable(mac_conf[cv.GenerateID()])
    await cg.register_component(mac_var, mac_conf)
    await text.register_text(mac_var, mac_conf)
    cg.add(mac_var.set_parent(var))
    cg.add(var.set_target_mac_entity(mac_var))

    st_var = await text_sensor.new_text_sensor(config[CONF_GW_STATUS])
    cg.add(var.set_gw_status(st_var))

    for key, setter in TEXT_SENSOR_SETTERS.items():
        if (conf := config.get(key)) is not None:
            ts = await text_sensor.new_text_sensor(conf)
            cg.add(getattr(var, setter)(ts))

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
        # Registered also as a Component so its setup() (restore mode) runs.
        await cg.register_component(enable_var, enable_conf)
        cg.add(enable_var.set_parent(var))

    for key, (kind, setter) in SETPOINT_KIND_MAP.items():
        if (conf := config.get(key)) is None:
            continue
        min_v, max_v, step = SETPOINT_RANGES[key]
        num_var = await number.new_number(
            conf, min_value=min_v, max_value=max_v, step=step
        )
        cg.add(num_var.set_parent(var))
        cg.add(num_var.set_kind(kind))
        cg.add(getattr(var, setter)(num_var))

    if poll_interval_conf := config.get(CONF_POLL_INTERVAL):
        poll_interval_var = await number.new_number(
            poll_interval_conf,
            min_value=UPDATE_INTERVAL_MIN_S,
            max_value=UPDATE_INTERVAL_MAX_S,
            step=1,
        )
        await cg.register_component(poll_interval_var, poll_interval_conf)
        cg.add(poll_interval_var.set_parent(var))
