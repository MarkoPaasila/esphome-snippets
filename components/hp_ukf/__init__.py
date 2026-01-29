"""HP-UKF external component for ESPHome."""

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)
from esphome.components import sensor

DEPENDENCIES = ["sensor"]

hp_ukf_ns = cg.esphome_ns.namespace("hp_ukf")
HpUkfComponent = hp_ukf_ns.class_("HpUkfComponent", cg.PollingComponent)

CONF_HP_UKF = "hp_ukf"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_INLET_TEMPERATURE = "inlet_temperature"
CONF_INLET_HUMIDITY = "inlet_humidity"
CONF_OUTLET_TEMPERATURE = "outlet_temperature"
CONF_OUTLET_HUMIDITY = "outlet_humidity"
CONF_TRACK_TEMPERATURE_DERIVATIVES = "track_temperature_derivatives"
CONF_FILTERED_INLET_TEMPERATURE = "filtered_inlet_temperature"
CONF_FILTERED_INLET_HUMIDITY = "filtered_inlet_humidity"
CONF_FILTERED_OUTLET_TEMPERATURE = "filtered_outlet_temperature"
CONF_FILTERED_OUTLET_HUMIDITY = "filtered_outlet_humidity"
CONF_FILTERED_INLET_TEMPERATURE_DERIVATIVE = "filtered_inlet_temperature_derivative"
CONF_FILTERED_OUTLET_TEMPERATURE_DERIVATIVE = "filtered_outlet_temperature_derivative"
CONF_FILTERED_INLET_HUMIDITY_DERIVATIVE = "filtered_inlet_humidity_derivative"
CONF_FILTERED_OUTLET_HUMIDITY_DERIVATIVE = "filtered_outlet_humidity_derivative"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HpUkfComponent),
        cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
        cv.Optional(CONF_INLET_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_INLET_HUMIDITY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTLET_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTLET_HUMIDITY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TRACK_TEMPERATURE_DERIVATIVES, default=True): cv.boolean,
        cv.Optional(
            CONF_FILTERED_INLET_TEMPERATURE,
            default={CONF_NAME: "Filtered Inlet Temperature"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_INLET_HUMIDITY,
            default={CONF_NAME: "Filtered Inlet Humidity"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_TEMPERATURE,
            default={CONF_NAME: "Filtered Outlet Temperature"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_HUMIDITY,
            default={CONF_NAME: "Filtered Outlet Humidity"},
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_INLET_TEMPERATURE_DERIVATIVE,
            default={CONF_NAME: "Filtered Inlet Temperature Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="°C/s",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_TEMPERATURE_DERIVATIVE,
            default={CONF_NAME: "Filtered Outlet Temperature Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="°C/s",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_INLET_HUMIDITY_DERIVATIVE,
            default={CONF_NAME: "Filtered Inlet Humidity Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="%/s",
            accuracy_decimals=4,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(
            CONF_FILTERED_OUTLET_HUMIDITY_DERIVATIVE,
            default={CONF_NAME: "Filtered Outlet Humidity Derivative"},
        ): sensor.sensor_schema(
            unit_of_measurement="%/s",
            accuracy_decimals=4,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_track_temperature_derivatives(config[CONF_TRACK_TEMPERATURE_DERIVATIVES]))
    if CONF_INLET_TEMPERATURE in config:
        sens = await cg.get_variable(config[CONF_INLET_TEMPERATURE])
        cg.add(var.set_inlet_temperature_sensor(sens))
    if CONF_INLET_HUMIDITY in config:
        sens = await cg.get_variable(config[CONF_INLET_HUMIDITY])
        cg.add(var.set_inlet_humidity_sensor(sens))
    if CONF_OUTLET_TEMPERATURE in config:
        sens = await cg.get_variable(config[CONF_OUTLET_TEMPERATURE])
        cg.add(var.set_outlet_temperature_sensor(sens))
    if CONF_OUTLET_HUMIDITY in config:
        sens = await cg.get_variable(config[CONF_OUTLET_HUMIDITY])
        cg.add(var.set_outlet_humidity_sensor(sens))

    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_TEMPERATURE])
    cg.add(var.set_filtered_inlet_temperature_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_HUMIDITY])
    cg.add(var.set_filtered_inlet_humidity_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_TEMPERATURE])
    cg.add(var.set_filtered_outlet_temperature_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_HUMIDITY])
    cg.add(var.set_filtered_outlet_humidity_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_TEMPERATURE_DERIVATIVE])
    cg.add(var.set_filtered_inlet_temperature_derivative_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_TEMPERATURE_DERIVATIVE])
    cg.add(var.set_filtered_outlet_temperature_derivative_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_INLET_HUMIDITY_DERIVATIVE])
    cg.add(var.set_filtered_inlet_humidity_derivative_sensor(sens))
    sens = await sensor.new_sensor(config[CONF_FILTERED_OUTLET_HUMIDITY_DERIVATIVE])
    cg.add(var.set_filtered_outlet_humidity_derivative_sensor(sens))
