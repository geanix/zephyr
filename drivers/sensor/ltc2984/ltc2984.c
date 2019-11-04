/*
 * Copyright (c) 2019 Geanix ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Temperature driver for the LTC2984
 */

#include <drivers/sensor.h>
#include <drivers/spi.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ltc2984, CONFIG_SENSOR_LOG_LEVEL);

/* LTC2984 register addresses */
#define LTC2984_REG_COMMAND_STATUS        0x000U
#define LTC2984_REG_EEPROM_KEY            0x00BU
#define LTC2984_REG_EEPROM_READ_RST       0x0D0U
#define LTC2984_REG_GLOBAL_CONF           0x0F0U
#define LTC2984_REG_MULTIPLE_CH_MASK      0x0F4U
#define LTC2984_REG_EEPROM_STATUS         0x0F9U
#define LTC2984_REG_MUX_DELAY             0x0FFU

/* LTC2984 channel input */
#define LTC2984_REG_RESULT(ch)            (0x010U + (4 * ch - 1))
#define LTC2984_REG_ASSIGNMENT(ch)        (0x200U + (4 * ch - 1))

/* LTC2984 instruction byte */
#define LTC2984_SPI_WRITE                 0x02U
#define LTC2984_SPI_READ                  0x03U

/* Maximum number of ADC channels */
#define LTC2984_MAX_CHANNELS              20

/* Sensor Types Table5 */
#define LTC2984_SENSOR_UNASSIGNED         0x00U
#define LTC2984_SENSOR_THERMOC_J          0x01U
#define LTC2984_SENSOR_THERMOC_K          0x02U
#define LTC2984_SENSOR_THERMOC_E          0x03U
#define LTC2984_SENSOR_THERMOC_N          0x04U
#define LTC2984_SENSOR_THERMOC_R          0x05U
#define LTC2984_SENSOR_THERMOC_S          0x06U
#define LTC2984_SENSOR_THERMOC_T          0x07U
#define LTC2984_SENSOR_THERMOC_B          0x08U
#define LTC2984_SENSOR_RTD_PT10           0x0AU
#define LTC2984_SENSOR_RTD_PT50           0x0BU
#define LTC2984_SENSOR_RTD_PT100          0x0CU
#define LTC2984_SENSOR_RTD_PT200          0x0DU
#define LTC2984_SENSOR_RTD_PT500          0x0EU
#define LTC2984_SENSOR_RTD_PT1000         0x0FU
#define LTC2984_SENSOR_RTD_1000           0x10U
#define LTC2984_SENSOR_THERM_2252         0x13U
#define LTC2984_SENSOR_THERM_3            0x14U
#define LTC2984_SENSOR_THERM_5            0x15U
#define LTC2984_SENSOR_THERM_10           0x16U
#define LTC2984_SENSOR_THERM_30           0x17U
#define LTC2984_SENSOR_DIODE              0x1CU
#define LTC2984_SENSOR_SENSE_RESISTOR     0x1DU

#define LTC2984_SENSOR_TYPE_CONFIG(conf)  (conf << 27)
#define LTC2984_THERMOC_COLD_JUNC_CH(ch)  (ch << 22)
#define LTC2984_THERMOC_THERM_SGL         BIT(21)
#define LTC2984_THERMOC_OC_CHECK          BIT(20)
#define LTC2984_THERMOC_OC_CURRENT(oc)    (oc << 18)

/* Table 31 */
#define LTC2984_RSENSE_CH(ch)             (ch << 22)

#define LTC2984_SENSOR_CONFIG(conf)       (conf << 18)
#define LTC2984_2WIRE_EXTERNAL_GND        0x00U
#define LTC2984_2WIRE_INTERNAL_GND        0x01U
#define LTC2984_3WIRE_EXTERNAL_GND        0x02U
#define LTC2984_3WIRE_INTERNAL_GND        0x03U

/* Excitation current Table33 */
#define LTC2984_EXCITATION_CURRENT(conf)  (conf << 14)
#define LTC2984_EC_5UA                    0x01U
#define LTC2984_EC_10UA                   0x02U
#define LTC2984_EC_25UA                   0x03U
#define LTC2984_EC_50UA                   0x04U
#define LTC2984_EC_100UA                  0x05U
#define LTC2984_EC_250UA                  0x06U
#define LTC2984_EC_500UA                  0x07U
#define LTC2984_EC_1000UA                 0x08U

/* RTD Curve Table34 */
#define LTC2984_RTD_CURVE(curve)          (curve << 12)
#define LTC2984_RTC_CURVE_EUROPEAN        0x00U
#define LTC2984_RTC_CURVE_AMERICAN        0x01U
#define LTC2984_RTC_CURVE_JAPANESE        0x02U

/* Table 37 */
#define LTC2984_SENSE_RESISTOR_VALUE(ohm) (ohm * 1024)

#if 0
struct ltc2984_aux_config {
	const char *spi_cs_dev_name;
	u8_t spi_cs_pin;
};
#endif

struct ltc2984_config {
	const char *spi_dev_name;
	u32_t spi_freq;
	u16_t spi_slave;
	u8_t spi_cs_pin;
	const char *spi_cs_port;
	u8_t resolution;
	u8_t channels;
};

struct ltc2984_data {
	struct device *dev;
	struct device *spi_dev;
	struct spi_config spi_cfg;
	struct spi_cs_control spi_cs_ctrl;
	s32_t *buffer;
	s32_t *repeat_buffer;
	u32_t channels;
	u32_t channel_id;
};

static int ltc2984_read_reg(struct device *dev, u16_t addr, u8_t *dptr,
			    size_t len)
{
	struct ltc2984_data *data = dev->driver_data;

	u8_t buffer_tx[3] = { LTC2984_SPI_READ, addr, addr >> 8 };
	const struct spi_buf tx_buf = {
		.buf = buffer_tx,
		.len = 3,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 3,
		},
		{
			.buf = dptr,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	return spi_transceive(data->spi_dev, &data->spi_cfg, &tx, &rx);
}

static int ltc2984_read_reg8(struct device *dev, u16_t addr, u8_t *val)
{
	return ltc2984_read_reg(dev, addr, val, sizeof(val));
}

static int ltc2984_write_reg(struct device *dev, u16_t addr, u8_t *dptr,
			     size_t len)
{
	struct ltc2984_data *data = dev->driver_data;

	u8_t buffer_tx[3] = { LTC2984_SPI_WRITE, addr, addr >> 8 };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 3,
		},
		{
			.buf = dptr,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	return spi_write(data->spi_dev, &data->spi_cfg, &tx);
}

static int ltc2984_write_reg8(struct device *dev, u16_t addr, u8_t val)
{
	return ltc2984_write_reg(dev, addr, &val, sizeof(val));
}


static int ltc2984_channel_setup(struct device *dev, u8_t channel,
				 u32_t data)
{
	u8_t payload[4];
	u16_t addr;
	int ret;


	payload[0] = data;
	payload[1] = data >> 8;
	payload[2] = data >> 16;
	payload[3] = data >> 24;

	addr = LTC2984_REG_ASSIGNMENT(channel);
	ret = ltc2984_write_reg(dev, addr, payload, sizeof(payload));
	if (ret) {
		LOG_ERR("failed to configure channel (err %d)", ret);
	}

	return ret;
}

static int ltc2984_sample_fetch(struct device *dev, enum sensor_channel chan)
{
#if 0
	struct ltc2984_data *data = dev->driver_data;
	u8_t buf[8];
	s32_t adc_press, adc_temp, adc_humidity;
	int size = 6;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	if (data->chip_id == BME280_CHIP_ID) {
		size = 8;
	}
	ret = bm280_reg_read(data, BME280_REG_PRESS_MSB, buf, size);
	if (ret < 0) {
		return ret;
	}

	adc_press = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	adc_temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

	ltc2984_compensate_temp(data, adc_temp);
	ltc2984_compensate_press(data, adc_press);

	if (data->chip_id == BME280_CHIP_ID) {
		adc_humidity = (buf[6] << 8) | buf[7];
		ltc2984_compensate_humidity(data, adc_humidity);
	}
#endif
	return 0;
}

static int ltc2984_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
#if 0
	struct ltc2984_data *data = dev->driver_data;
	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/*
		 * data->comp_temp has a resolution of 0.01 degC.  So
		 * 5123 equals 51.23 degC.
		 */
		val->val1 = data->comp_temp / 100;
		val->val2 = data->comp_temp % 100 * 10000;
		break;
	case SENSOR_CHAN_PRESS:
		/*
		 * data->comp_press has 24 integer bits and 8
		 * fractional.  Output value of 24674867 represents
		 * 24674867/256 = 96386.2 Pa = 963.862 hPa
		 */
		val->val1 = (data->comp_press >> 8) / 1000U;
		val->val2 = (data->comp_press >> 8) % 1000 * 1000U +
			(((data->comp_press & 0xff) * 1000U) >> 8);
		break;
	case SENSOR_CHAN_HUMIDITY:
		/*
		 * data->comp_humidity has 22 integer bits and 10
		 * fractional.  Output value of 47445 represents
		 * 47445/1024 = 46.333 %RH
		 */
		val->val1 = (data->comp_humidity >> 10);
		val->val2 = (((data->comp_humidity & 0x3ff) * 1000U * 1000U) >> 10);
		break;
	default:
		return -EINVAL;
	}
#endif
	return 0;
}

static int ltc2984_init(struct device *dev)
{
	const struct ltc2984_config *config = dev->config->config_info;
	struct ltc2984_data *data = dev->driver_data;
	u32_t channel_data;

	data->dev = dev;

	data->spi_cfg.operation = (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
				   SPI_WORD_SET(8));
	data->spi_cfg.frequency = config->spi_freq;
	data->spi_cfg.slave = config->spi_slave;

	data->spi_dev = device_get_binding(config->spi_dev_name);
	if (!data->spi_dev) {
		LOG_ERR("SPI master device '%s' not found",
			config->spi_dev_name);
		return -EINVAL;
	}

#ifdef DT_INST_0_LLTC_LTC2984_CS_GPIOS_PIN
	data->spi_cs_ctrl.gpio_dev =
		device_get_binding(config->spi_cs_port);
	if (!data->spi_cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	data->spi_cs_ctrl.gpio_pin = config->spi_cs_pin;
	data->spi_cs_ctrl.delay = 0U;

	data->spi_cfg.cs = &data->spi_cs_ctrl;
#else
	data->spi_cfg.cs = NULL;
#endif  /* DT_INST_0_LLTC_LTC2984_CS_GPIOS_PIN */

	/* Channel 12/11 RTD */
	channel_data = LTC2984_SENSOR_TYPE_CONFIG(LTC2984_SENSOR_SENSE_RESISTOR) +
		       LTC2984_SENSE_RESISTOR_VALUE(2000);
	ltc2984_channel_setup(dev, 12, channel_data);

	/* Channel 12/13 RTD */
	channel_data = LTC2984_SENSOR_TYPE_CONFIG(LTC2984_SENSOR_RTD_PT100) +
		       LTC2984_RSENSE_CH(12) +
		       LTC2984_SENSOR_CONFIG(LTC2984_2WIRE_INTERNAL_GND) +
		       LTC2984_EXCITATION_CURRENT(LTC2984_EC_10UA) +
		       LTC2984_RTD_CURVE(LTC2984_RTC_CURVE_EUROPEAN);
	ltc2984_channel_setup(dev, 13, channel_data);

	/* Channel 17/18 RTD */
	channel_data = LTC2984_SENSOR_TYPE_CONFIG(LTC2984_SENSOR_SENSE_RESISTOR) +
		       LTC2984_SENSE_RESISTOR_VALUE(2000);
	ltc2984_channel_setup(dev, 12, channel_data);

	/* Channel 18/19 RTD */
	channel_data = LTC2984_SENSOR_TYPE_CONFIG(LTC2984_SENSOR_RTD_PT100) +
		       LTC2984_RSENSE_CH(12) +
		       LTC2984_SENSOR_CONFIG(LTC2984_2WIRE_INTERNAL_GND) +
		       LTC2984_EXCITATION_CURRENT(LTC2984_EC_10UA) +
		       LTC2984_RTD_CURVE(LTC2984_RTC_CURVE_EUROPEAN);
	ltc2984_channel_setup(dev, 13, channel_data);

	return 0;
}

static const struct sensor_driver_api ltc2984_api_funcs = {
	.sample_fetch = ltc2984_sample_fetch,
	.channel_get = ltc2984_channel_get,
};

static const struct ltc2984_config ltc2984_config = {
	.spi_dev_name = DT_INST_0_LLTC_LTC2984_BUS_NAME,
	.spi_freq = DT_INST_0_LLTC_LTC2984_SPI_MAX_FREQUENCY,
	.spi_slave = DT_INST_0_LLTC_LTC2984_BASE_ADDRESS,
#ifdef DT_INST_0_LLTC_LTC2984_CS_GPIOS_PIN
	.spi_cs_pin = DT_INST_0_LLTC_LTC2984_CS_GPIOS_PIN,
	.spi_cs_port = DT_INST_0_LLTC_LTC2984_CS_GPIOS_CONTROLLER,
#endif  /* DT_INST_0_LLTC_LTC2984_CS_GPIOS_PIN */
};

DEVICE_AND_API_INIT(ltc2984_0, DT_INST_0_LLTC_LTC2984_LABEL,
		    &ltc2984_init, NULL, &ltc2984_config,
		    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &ltc2984_api_funcs);
