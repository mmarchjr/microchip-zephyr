/*
 * Copyright (c) 2024 Arrow Electronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tmp75

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "tmp75.h"

LOG_MODULE_REGISTER(TMP75, CONFIG_SENSOR_LOG_LEVEL);

#define I2C_REG_ADDR_SIZE   1
#define I2C_REG_SENSOR_SIZE sizeof(uint16_t)
#define I2C_BUFFER_SIZE     I2C_REG_ADDR_SIZE + I2C_REG_SENSOR_SIZE

#define I2C_REG_ADDR_OFFSET   0
#define I2C_WRITE_DATA_OFFSET 1

static int tmp75_reg_read(const struct tmp75_config *cfg, uint8_t reg, uint16_t *val)
{
	if (i2c_burst_read_dt(&cfg->bus, reg, (uint8_t *)val, sizeof(*val)) < 0) {
		return -EIO;
	}
	*val = sys_be16_to_cpu(*val);
	return 0;
}

static int tmp75_reg_write(const struct tmp75_config *cfg, uint8_t reg, uint16_t val)
{
	uint8_t buf[I2C_REG_ADDR_SIZE + I2C_REG_SENSOR_SIZE];

	buf[I2C_REG_ADDR_OFFSET] = reg;
	sys_put_be16(val, &buf[I2C_WRITE_DATA_OFFSET]);

	return i2c_write_dt(&cfg->bus, buf, sizeof(buf));
}

static inline uint32_t tmp75_conv_time_ms(uint8_t resolution)
{
	switch (resolution) {
	case TMP75_RES_9_BIT:
		return 220;
	case TMP75_RES_10_BIT:
		return 110;
	case TMP75_RES_11_BIT:
		return 55;
	case TMP75_RES_12_BIT:
		return 28;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static inline uint32_t tmp75_temp_scale(uint8_t resolution)
{
	switch (resolution) {
	case TMP75_RES_9_BIT:
		return TMP75_TEMP_SCALE_9_BIT;
	case TMP75_RES_10_BIT:
		return TMP75_TEMP_SCALE_10_BIT;
	case TMP75_RES_11_BIT:
		return TMP75_TEMP_SCALE_11_BIT;
	case TMP75_RES_12_BIT:
		return TMP75_TEMP_SCALE_12_BIT;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static inline uint8_t tmp75_data_shift(uint8_t resolution)
{
	switch (resolution) {
	case TMP75_RES_9_BIT:
		return 7;
	case TMP75_RES_10_BIT:
		return 6;
	case TMP75_RES_11_BIT:
		return 5;
	case TMP75_RES_12_BIT:
		return 4;
	default:
		__ASSERT_NO_MSG(false);
		return 4;
	}
}

#if CONFIG_TMP75_ALERT_INTERRUPTS
static int set_threshold_attribute(const struct device *dev, uint8_t reg, int16_t value,
				   const char *error_msg)
{
	if (tmp75_reg_write(dev->config, reg, value) < 0) {
		LOG_ERR("Failed to set %s attribute!", error_msg);
		return -EIO;
	}
	return 0;
}
#endif

static int tmp75_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct tmp75_config *cfg = dev->config;
	struct tmp75_data *drv_data = dev->data;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	switch (attr) {
#if CONFIG_TMP75_ALERT_INTERRUPTS
	case SENSOR_ATTR_LOWER_THRESH: {
		/* Convert sensor_value to TMP75 register format based on resolution */
		uint32_t scale = tmp75_temp_scale(cfg->resolution);
		uint8_t shift = tmp75_data_shift(cfg->resolution);
		int32_t raw_val = (val->val1 * uCELSIUS_IN_CELSIUS + val->val2) / scale;
		uint16_t reg_val = (raw_val << shift) & 0xFFF0;
		return set_threshold_attribute(dev, TMP75_REG_TLOW, reg_val,
					       "SENSOR_ATTR_LOWER_THRESH");
	}

	case SENSOR_ATTR_UPPER_THRESH: {
		/* Convert sensor_value to TMP75 register format based on resolution */
		uint32_t scale = tmp75_temp_scale(cfg->resolution);
		uint8_t shift = tmp75_data_shift(cfg->resolution);
		int32_t raw_val = (val->val1 * uCELSIUS_IN_CELSIUS + val->val2) / scale;
		uint16_t reg_val = (raw_val << shift) & 0xFFF0;
		return set_threshold_attribute(dev, TMP75_REG_THIGH, reg_val,
					       "SENSOR_ATTR_UPPER_THRESH");
	}
#endif

	default:
		return -ENOTSUP;
	}
}

#if CONFIG_TMP75_ALERT_INTERRUPTS
static int get_threshold_attribute(const struct device *dev, uint8_t reg, struct sensor_value *val,
				   const char *error_msg)
{
	const struct tmp75_config *cfg = dev->config;
	uint16_t value;
	uint8_t shift = tmp75_data_shift(cfg->resolution);
	uint32_t scale = tmp75_temp_scale(cfg->resolution);

	if (tmp75_reg_read(dev->config, reg, &value) < 0) {
		LOG_ERR("Failed to get %s attribute!", error_msg);
		return -EIO;
	}

	/* Extract temperature value based on resolution */
	int16_t temp_raw = (value >> shift);
	int32_t temp_udeg = temp_raw * scale;

	val->val1 = temp_udeg / uCELSIUS_IN_CELSIUS;
	val->val2 = temp_udeg % uCELSIUS_IN_CELSIUS;
	return 0;
}
#endif

static int tmp75_attr_get(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	switch (attr) {
#if CONFIG_TMP75_ALERT_INTERRUPTS
	case SENSOR_ATTR_LOWER_THRESH:
		return get_threshold_attribute(dev, TMP75_REG_TLOW, val,
					       "SENSOR_ATTR_LOWER_THRESH");

	case SENSOR_ATTR_UPPER_THRESH:
		return get_threshold_attribute(dev, TMP75_REG_THIGH, val,
					       "SENSOR_ATTR_UPPER_THRESH");
#endif

	default:
		return -ENOTSUP;
	}
}

static int tmp75_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct tmp75_data *drv_data = dev->data;
	const struct tmp75_config *cfg = dev->config;
	uint16_t val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

	if (cfg->shutdown_mode || cfg->one_shot) {
		/* Initiate a single temperature conversion */
		uint16_t config_reg = drv_data->config_reg;

		TMP75_SET_ONE_SHOT(config_reg, 1);

		if (tmp75_reg_write(cfg, TMP75_REG_CONFIG, config_reg) < 0) {
			LOG_ERR("Failed to initiate one-shot conversion");
			return -EIO;
		}

		/* Wait for conversion to complete */
		k_sleep(K_MSEC(tmp75_conv_time_ms(cfg->resolution)));
	}

	if (tmp75_reg_read(cfg, TMP75_REG_TEMPERATURE, &val) < 0) {
		return -EIO;
	}

	/* Shift based on resolution to get the actual temperature value */
	drv_data->sample = arithmetic_shift_right((int16_t)val, tmp75_data_shift(cfg->resolution));
	return 0;
}

static int tmp75_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct tmp75_data *drv_data = dev->data;
	const struct tmp75_config *cfg = dev->config;
	int32_t uval;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	uval = (int32_t)drv_data->sample * tmp75_temp_scale(cfg->resolution);
	val->val1 = uval / uCELSIUS_IN_CELSIUS;
	val->val2 = uval % uCELSIUS_IN_CELSIUS;

	return 0;
}

static int tmp75_init(const struct device *dev)
{
	const struct tmp75_config *cfg = dev->config;
	struct tmp75_data *drv_data = dev->data;
	uint16_t config_reg = 0;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C bus %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	drv_data->tmp75_dev = dev;
	drv_data->resolution = cfg->resolution;

	/* Build configuration register */
	TMP75_SET_CONVERSION_RATE(config_reg, cfg->resolution);  /* Resolution determines conversion rate */
	TMP75_SET_FAULT_QUEUE(config_reg, cfg->fault_queue);
	TMP75_SET_ALERT_POLARITY(config_reg, cfg->alert_pol);
	TMP75_SET_THERMOSTAT_MODE(config_reg, cfg->interrupt_mode);
	TMP75_SET_SHUTDOWN_MODE(config_reg, cfg->shutdown_mode);
	TMP75_SET_RESOLUTION(config_reg, cfg->resolution);

	drv_data->config_reg = config_reg;

	if (tmp75_reg_write(cfg, TMP75_REG_CONFIG, config_reg) < 0) {
		LOG_ERR("Failed to write configuration register");
		return -EIO;
	}

#if CONFIG_TMP75_ALERT_INTERRUPTS
	if (cfg->alert_gpio.port != NULL) {
		if (!device_is_ready(cfg->alert_gpio.port)) {
			LOG_ERR("Alert GPIO device not ready");
			return -ENODEV;
		}

		int ret = gpio_pin_configure_dt(&cfg->alert_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure alert GPIO");
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->alert_gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure alert GPIO interrupt");
			return ret;
		}

		gpio_init_callback(&drv_data->temp_alert_gpio_cb,
				   tmp75_trigger_handle_alert,
				   BIT(cfg->alert_gpio.pin));

		ret = gpio_add_callback(cfg->alert_gpio.port, &drv_data->temp_alert_gpio_cb);
		if (ret < 0) {
			LOG_ERR("Failed to add alert GPIO callback");
			return ret;
		}
	}
#endif

	return 0;
}

static DEVICE_API(sensor, tmp75_driver_api) = {
	.attr_set = tmp75_attr_set,
	.attr_get = tmp75_attr_get,
	.sample_fetch = tmp75_sample_fetch,
	.channel_get = tmp75_channel_get,
#if CONFIG_TMP75_ALERT_INTERRUPTS
	.trigger_set = tmp75_trigger_set,
#endif
};

#define TMP75_RESOLUTION(idx) \
	((DT_INST_PROP(idx, resolution) == 9)  ? TMP75_RES_9_BIT : \
	 (DT_INST_PROP(idx, resolution) == 10) ? TMP75_RES_10_BIT : \
	 (DT_INST_PROP(idx, resolution) == 11) ? TMP75_RES_11_BIT : TMP75_RES_12_BIT)

#define TMP75_FAULT_QUEUE(idx) \
	((DT_INST_PROP(idx, fault_queue) == 1) ? TMP75_FAULT_1 : \
	 (DT_INST_PROP(idx, fault_queue) == 2) ? TMP75_FAULT_2 : \
	 (DT_INST_PROP(idx, fault_queue) == 4) ? TMP75_FAULT_4 : TMP75_FAULT_6)

#define TMP75_INST_INIT(idx) \
	static struct tmp75_data tmp75_data_##idx; \
	static const struct tmp75_config tmp75_config_##idx = { \
		.bus = I2C_DT_SPEC_INST_GET(idx), \
		.alert_gpio = GPIO_DT_SPEC_INST_GET_OR(idx, alert_gpios, {0}), \
		.resolution = TMP75_RESOLUTION(idx), \
		.fault_queue = TMP75_FAULT_QUEUE(idx), \
		.alert_pol = DT_INST_PROP(idx, alert_pin_active_high), \
		.one_shot = DT_INST_PROP(idx, one_shot_conversion), \
		.interrupt_mode = DT_INST_PROP(idx, interrupt_mode), \
		.shutdown_mode = DT_INST_PROP(idx, shutdown_mode), \
	}; \
	SENSOR_DEVICE_DT_INST_DEFINE(idx, tmp75_init, NULL, \
				     &tmp75_data_##idx, &tmp75_config_##idx, \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				     &tmp75_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TMP75_INST_INIT)