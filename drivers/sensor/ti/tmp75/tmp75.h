/*
 * Copyright (c) 2024 Arrow Electronics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EEEBE0B7_8054_4369_8D18_5F1F76A6B39E
#define EEEBE0B7_8054_4369_8D18_5F1F76A6B39E

#ifndef ZEPHYR_DRIVERS_SENSOR_TMP75_TMP75_H_
#define ZEPHYR_DRIVERS_SENSOR_TMP75_TMP75_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* TMP75 Register addresses */
#define TMP75_REG_TEMPERATURE 0x00
#define TMP75_REG_CONFIG      0x01
#define TMP75_REG_TLOW        0x02
#define TMP75_REG_THIGH       0x03

/* TMP75 Configuration Register bit definitions */
#define TMP75_CONFIG_OS       BIT(15)  /* One-shot conversion */
#define TMP75_CONFIG_R1       BIT(13)  /* Conversion rate bit 1 */
#define TMP75_CONFIG_R0       BIT(12)  /* Conversion rate bit 0 */
#define TMP75_CONFIG_F1       BIT(11)  /* Fault queue bit 1 */
#define TMP75_CONFIG_F0       BIT(10)  /* Fault queue bit 0 */
#define TMP75_CONFIG_POL      BIT(9)   /* Alert polarity */
#define TMP75_CONFIG_TM       BIT(8)   /* Thermostat mode */
#define TMP75_CONFIG_SD       BIT(7)   /* Shutdown mode */
#define TMP75_CONFIG_R1_RES   BIT(6)   /* Resolution bit 1 */
#define TMP75_CONFIG_R0_RES   BIT(5)   /* Resolution bit 0 */

/* Resolution settings (bits 6:5) */
#define TMP75_RES_9_BIT   0x00  /* 9-bit resolution, 220ms conversion time */
#define TMP75_RES_10_BIT  0x20  /* 10-bit resolution, 110ms conversion time */
#define TMP75_RES_11_BIT  0x40  /* 11-bit resolution, 55ms conversion time */
#define TMP75_RES_12_BIT  0x60  /* 12-bit resolution, 27.5ms conversion time */
#define TMP75_RES_MASK    0x60  /* Mask for resolution bits */

/* Conversion rate settings (bits 13:12) - same as resolution for TMP75 */
#define TMP75_CR_27_5MS   0x0000  /* 27.5ms (12-bit) */
#define TMP75_CR_55MS     0x2000  /* 55ms (11-bit) */
#define TMP75_CR_110MS    0x4000  /* 110ms (10-bit) */
#define TMP75_CR_220MS    0x6000  /* 220ms (9-bit) */

/* Fault queue settings (bits 11:10) */
#define TMP75_FAULT_1     0x0000  /* 1 fault */
#define TMP75_FAULT_2     0x0400  /* 2 faults */
#define TMP75_FAULT_4     0x0800  /* 4 faults */
#define TMP75_FAULT_6     0x0C00  /* 6 faults */

/* Temperature scale factors for different resolutions (in micro degrees Celsius) */
#define TMP75_TEMP_SCALE_9_BIT   500000   /* 0.5°C per LSB */
#define TMP75_TEMP_SCALE_10_BIT  250000   /* 0.25°C per LSB */
#define TMP75_TEMP_SCALE_11_BIT  125000   /* 0.125°C per LSB */
#define TMP75_TEMP_SCALE_12_BIT  62500    /* 0.0625°C per LSB */

#define uCELSIUS_IN_CELSIUS 1000000

/* Macros for configuration register manipulation */
#define TMP75_SET_ONE_SHOT(reg, enable) \
	((reg) = ((reg) & ~TMP75_CONFIG_OS) | ((enable) << 15))

#define TMP75_SET_CONVERSION_RATE(reg, rate) \
	((reg) = ((reg) & ~(TMP75_CONFIG_R1 | TMP75_CONFIG_R0)) | (rate))

#define TMP75_SET_FAULT_QUEUE(reg, faults) \
	((reg) = ((reg) & ~(TMP75_CONFIG_F1 | TMP75_CONFIG_F0)) | (faults))

#define TMP75_SET_ALERT_POLARITY(reg, active_high) \
	((reg) = ((reg) & ~TMP75_CONFIG_POL) | ((active_high) << 9))

#define TMP75_SET_THERMOSTAT_MODE(reg, interrupt_mode) \
	((reg) = ((reg) & ~TMP75_CONFIG_TM) | ((interrupt_mode) << 8))

#define TMP75_SET_SHUTDOWN_MODE(reg, shutdown) \
	((reg) = ((reg) & ~TMP75_CONFIG_SD) | ((shutdown) << 7))

#define TMP75_SET_RESOLUTION(reg, resolution) \
	((reg) = ((reg) & ~TMP75_RES_MASK) | (resolution))

/* Get resolution from config register */
#define TMP75_GET_RESOLUTION(reg) ((reg) & TMP75_RES_MASK)

/* Get conversion time in ms based on resolution */
#define TMP75_CONV_TIME_MS(res) \
	(((res) == TMP75_RES_9_BIT)  ? 220 : \
	 ((res) == TMP75_RES_10_BIT) ? 110 : \
	 ((res) == TMP75_RES_11_BIT) ? 55  : 28)

/* Get temperature scale based on resolution */
#define TMP75_TEMP_SCALE(res) \
	(((res) == TMP75_RES_9_BIT)  ? TMP75_TEMP_SCALE_9_BIT : \
	 ((res) == TMP75_RES_10_BIT) ? TMP75_TEMP_SCALE_10_BIT : \
	 ((res) == TMP75_RES_11_BIT) ? TMP75_TEMP_SCALE_11_BIT : TMP75_TEMP_SCALE_12_BIT)

/* Get data shift based on resolution (temperature register is left-justified 12-bit) */
#define TMP75_DATA_SHIFT(res) \
	(((res) == TMP75_RES_9_BIT)  ? 7 : \
	 ((res) == TMP75_RES_10_BIT) ? 6 : \
	 ((res) == TMP75_RES_11_BIT) ? 5 : 4)

struct tmp75_data {
	const struct device *tmp75_dev;
	int16_t sample;
	uint16_t config_reg;
	uint8_t resolution;
	const struct sensor_trigger *temp_alert_trigger;
	sensor_trigger_handler_t temp_alert_handler;
	struct gpio_callback temp_alert_gpio_cb;
	bool over_threshold;
};

struct tmp75_config {
	const struct i2c_dt_spec bus;
	const struct gpio_dt_spec alert_gpio;
	uint8_t resolution;
	uint8_t fault_queue;
	bool alert_pol: 1;
	bool one_shot: 1;
	bool interrupt_mode: 1;
	bool shutdown_mode: 1;
};

int tmp75_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		      sensor_trigger_handler_t handler);

void tmp75_trigger_handle_alert(const struct device *port, struct gpio_callback *cb,
				gpio_port_pins_t pins);

#endif /* ZEPHYR_DRIVERS_SENSOR_TMP75_TMP75_H_ */


#endif /* EEEBE0B7_8054_4369_8D18_5F1F76A6B39E */
