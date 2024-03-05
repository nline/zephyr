/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 Linumiz
 */

#define DT_DRV_COMPAT memsic_mc3419

#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "mc3419.h"

LOG_MODULE_REGISTER(MC3419, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t mc3419_accel_sense_map[] = {1, 2, 4, 8, 6};
static struct mc3419_odr_map odr_map_table[] = {
	{25}, {50}, {62, 500}, {100},
	{125}, {250}, {500}, {1000}
};

static int mc3419_get_odr_value(uint16_t freq, uint16_t m_freq)
{
	for (int i = 0; i < ARRAY_SIZE(odr_map_table); i++) {
		if (odr_map_table[i].freq == freq &&
		    odr_map_table[i].mfreq == m_freq) {
			return i;
		}
	}

	return -EINVAL;
}

#if MC3419_BUS_SPI
static int mc3419_transceive(const struct device *dev, uint8_t reg,
			     bool write, void *buf, size_t length)
{
    if(write) {
        const struct mc3419_config *cfg = dev->config;
	    const struct spi_buf tx_buf[2] = {
	    	{
	    		.buf = &reg,
	    		.len = 1
	    	},
	    	{
	    		.buf = buf,
	    		.len = length
	    	}
	    };
	    const struct spi_buf_set tx = {
	    	.buffers = tx_buf,
	    	.count = buf ? 2 : 1
	    };

	    return spi_write_dt(&cfg->bus.spi, &tx);
    } else {
        const struct mc3419_config *cfg = dev->config;
        uint8_t dummy = 0;
	    const struct spi_buf tx_buf[2] = {
	    	{
	    		.buf = &reg,
	    		.len = 1
	    	},
	    	{
	    		.buf = &dummy,
	    		.len = 1
	    	},
	    	{
	    		.buf = buf,
	    		.len = length
	    	}
	    };
	    const struct spi_buf_set tx = {
	    	.buffers = tx_buf,
	    	.count = buf ? 3 : 1
	    };

		const struct spi_buf_set rx = {
			.buffers = tx_buf,
			.count = 3
		};

		return spi_transceive_dt(&cfg->bus.spi, &tx, &rx);
	}

}

#define MC3419_REG_READ			BIT(7)
#define MC3419_REG_MASK			0x7f

bool mc3419_bus_ready_spi(const struct device *dev)
{
	const struct mc3419_config *cfg = dev->config;

	return spi_is_ready_dt(&cfg->bus.spi);
}

int mc3419_read_spi(const struct device *dev,
		    uint8_t reg_addr, void *buf, uint8_t len)
{
	return mc3419_transceive(dev, reg_addr | MC3419_REG_READ, false,
				 buf, len);
}

int mc3419_write_spi(const struct device *dev,
		     uint8_t reg_addr, void *buf, uint8_t len)
{
	return mc3419_transceive(dev, reg_addr & MC3419_REG_MASK, true,
				 buf, len);
}

static const struct mc3419_bus_io mc3419_bus_io_spi = {
	.ready = mc3419_bus_ready_spi,
	.read = mc3419_read_spi,
	.write = mc3419_write_spi,
};
#endif

#if MC3419_BUS_I2C

bool mc3419_bus_ready_i2c(const struct device *dev)
{
	const struct mc3419_config *cfg = dev->config;

	return i2c_is_ready(cfg->bus.i2c);
}

int mc3419_read_i2c(const struct device *dev,
		    uint8_t reg_addr, void *buf, uint8_t len)
{
	const struct mc3219_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus.i2c, reg_addr, buf, len);
}

int mc3419_write_i2c(const struct device *dev,
		     uint8_t reg_addr, void *buf, uint8_t len)
{
	const struct mc3419_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->bus.i2c, reg_addr, buf, len);
}

static const struct mc3419_bus_io mc3419_bus_io_i2c = {
	.ready = mc3419_bus_ready_i2c,
	.read = mc3419_read_i2c,
	.write = mc3419_write_i2c,
};
#endif



static inline int mc3419_set_op_mode(const struct device *dev,
				     enum mc3419_op_mode mode)
{
    return cfg->bus_io->write(dev, MC3419_REG_OP_MODE, &mode, 1);
}

static int mc3419_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	int ret = 0;
	const struct mc3419_config *cfg = dev->config;
	struct mc3419_driver_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
    ret = cfg->bus_io->read(dev, MC3419_REG_XOUT_L,
				(uint8_t *)data->samples,
				MC3419_SAMPLE_READ_SIZE);
	k_sem_give(&data->sem);
	return ret;
}

static int mc3419_to_sensor_value(double sensitivity, int16_t *raw_data,
				  struct sensor_value *val)
{
	double value = sys_le16_to_cpu(*raw_data);

	value *= sensitivity * SENSOR_GRAVITY_DOUBLE / 1000;

	return sensor_value_from_double(val, value);
}

static int mc3419_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	int ret = 0;
	struct mc3419_driver_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ret = mc3419_to_sensor_value(data->sensitivity, &data->samples[0], val);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ret = mc3419_to_sensor_value(data->sensitivity, &data->samples[1], val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ret = mc3419_to_sensor_value(data->sensitivity, &data->samples[2], val);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = mc3419_to_sensor_value(data->sensitivity, &data->samples[0], &val[0]);
		ret |= mc3419_to_sensor_value(data->sensitivity, &data->samples[1], &val[1]);
		ret |= mc3419_to_sensor_value(data->sensitivity, &data->samples[2], &val[2]);
		break;
	default:
		LOG_ERR("Unsupported channel");
		ret = -ENOTSUP;
	}

	k_sem_give(&data->sem);
	return ret;
}

static int mc3419_set_accel_range(const struct device *dev, uint8_t range)
{
	int ret = 0;
	const struct mc3419_config *cfg = dev->config;
	struct mc3419_driver_data *data = dev->data;

	if (range >= MC3419_ACCL_RANGE_END) {
		LOG_ERR("Accel resolution is out of range");
		return -EINVAL;
	}

    uint8_t b;
    ret = cfg->bus_io->read(dev, MC3419_REG_RANGE_SELECT_CTRL, &b, 1);
    uint8_t to_write = (b & ~MC3419_RANGE_MASK) | ((range << 4) & MC3419_RANGE_MASK);
    ret |= cfg->bus_io->write(dev, MC3419_REG_RANGE_SELECT_CTRL, &to_write, 1);

	if (ret < 0) {
		LOG_ERR("Failed to set resolution (%d)", ret);
		return ret;
	}

	data->sensitivity = (double)(mc3419_accel_sense_map[range] *
				     SENSOR_GRAIN_VALUE);

	return 0;
}

static int mc3419_set_odr(const struct device *dev,
			  const struct sensor_value *val)
{
	int ret = 0;
	int data_rate = 0;
	const struct mc3419_config *cfg = dev->config;

	ret = mc3419_get_odr_value(val->val1, val->val2);
	if (ret < 0) {
		LOG_ERR("Failed to get odr value from odr map (%d)", ret);
		return ret;
	}

	data_rate = MC3419_BASE_ODR_VAL + ret;

    ret = cfg->bus_io->write(dev, MC3419_REG_SAMPLE_RATE, data_rate, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ODR (%d)", ret);
		return ret;
	}

	LOG_DBG("Set ODR Rate to 0x%x", data_rate);
	uint8_t b = CONFIG_MC3419_DECIMATION_RATE;
    ret = cfg->bus_io->write(dev, MC3419_REG_SAMPLE_RATE_2, &b, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set decimation rate (%d)", ret);
		return ret;
	}

	return 0;
}

#if defined(CONFIG_MC3419_TRIGGER)
static int mc3419_set_anymotion_threshold(const struct device *dev,
					  const struct sensor_value *val)
{
	int ret = 0;
	const struct mc3419_config *cfg = dev->config;
	uint8_t buf[3] = {0};

	if (val->val1 > MC3419_ANY_MOTION_THRESH_MAX) {
		return -EINVAL;
	}

	buf[0] = MC3419_REG_ANY_MOTION_THRES;
	sys_put_le16((uint16_t)val->val1, &buf[1]);

    ret = cfg->bus_io->write(dev, MC3419_REG_ANY_MOTION_THRES, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to set anymotion threshold (%d)", ret);
		return ret;
	}

	return 0;
}

static int mc3419_trigger_set(const struct device *dev,
			      const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	int ret = 0;
	const struct mc3419_config *cfg = dev->config;
	struct mc3419_driver_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
	ret = mc3419_set_op_mode(dev, MC3419_MODE_STANDBY);
	if (ret < 0) {
		goto exit;
	}

	ret = mc3419_configure_trigger(dev, trig, handler);
	if (ret < 0) {
		LOG_ERR("Failed to set trigger (%d)", ret);
	}

exit:
	mc3419_set_op_mode(cfg, MC3419_MODE_WAKE);

	k_sem_give(&data->sem);
	return ret;
}
#endif

static int mc3419_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	int ret = 0;
	struct mc3419_driver_data *data = dev->data;

	if (chan != SENSOR_CHAN_ACCEL_X &&
	    chan != SENSOR_CHAN_ACCEL_Y &&
	    chan != SENSOR_CHAN_ACCEL_Z &&
	    chan != SENSOR_CHAN_ACCEL_XYZ) {
		LOG_ERR("Not supported on this channel.");
		return -ENOTSUP;
	}

	k_sem_take(&data->sem, K_FOREVER);
	ret = mc3419_set_op_mode(dev->config, MC3419_MODE_STANDBY);
	if (ret < 0) {
		goto exit;
	}

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		ret = mc3419_set_accel_range(dev, val->val1);
		break;
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		ret = mc3419_set_odr(dev, val);
		break;
#if defined(CONFIG_MC3419_TRIGGER)
	case SENSOR_ATTR_SLOPE_TH:
		ret = mc3419_set_anymotion_threshold(dev, val);
		break;
#endif
	default:
		LOG_ERR("ACCEL attribute is not supported");
		ret = -EINVAL;
	}

exit:
	mc3419_set_op_mode(dev->config, MC3419_MODE_WAKE);

	k_sem_give(&data->sem);
	return ret;
}

static int mc3419_init(const struct device *dev)
{
	int ret = 0;
	struct mc3419_driver_data *data = dev->data;
	const struct mc3419_config *cfg = dev->config;

	if (!(cfg->bus_io->ready(dev))) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	k_sem_init(&data->sem, 1, 1);

#if defined(CONFIG_MC3419_TRIGGER)
	ret = mc3419_trigger_init(dev);
	if (ret < 0) {
		LOG_ERR("Could not initialize interrupts");
		return ret;
	}
#endif

	/* Leave the sensor in default power on state, will be
	 * enabled by configure attr or setting trigger.
	 */

	LOG_INF("MC3419 Initialized");

	return ret;
}

static const struct sensor_driver_api mc3419_api = {
	.attr_set = mc3419_attr_set,
#if defined(CONFIG_MC3419_TRIGGER)
	.trigger_set = mc3419_trigger_set,
#endif
	.sample_fetch = mc3419_sample_fetch,
	.channel_get = mc3419_channel_get,
};

#if defined(CONFIG_MC3419_TRIGGER)
#define MC3419_CFG_IRQ(idx)						\
	.int_gpio = GPIO_DT_SPEC_INST_GET_OR(idx, int_gpios, { 0 }),	\
	.int_cfg  = DT_INST_PROP(idx, int_pin2),
#else
#define MC3419_CFG_IRQ(idx)
#endif

#define MC3419_DEFINE_SPI(idx)						\
	static const struct mc3419_config mc3419_config_##idx = {	\
        .bus.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0), \
		.bus_io = &mc3419_bus_io_spi,				   \
		MC3419_CFG_IRQ(idx)					\
	};								\
	static struct mc3419_driver_data mc3419_data_##idx;		\
	SENSOR_DEVICE_DT_INST_DEFINE(idx,				\
				mc3419_init, NULL,			\
				&mc3419_data_##idx,			\
				&mc3419_config_##idx,			\
				POST_KERNEL,				\
				CONFIG_SENSOR_INIT_PRIORITY,		\
				&mc3419_api);


#define MC3419_DEFINE_I2C(idx)						\
	static const struct mc3419_config mc3419_config_##idx = {	\
        .bus.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.bus_io = &mc3419_bus_io_i2c,	\
		MC3419_CFG_IRQ(idx)					\
	};								\
	static struct mc3419_driver_data mc3419_data_##idx;		\
	SENSOR_DEVICE_DT_INST_DEFINE(idx,				\
				mc3419_init, NULL,			\
				&mc3419_data_##idx,			\
				&mc3419_config_##idx,			\
				POST_KERNEL,				\
				CONFIG_SENSOR_INIT_PRIORITY,		\
				&mc3419_api);


/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */
#define MC3419_DEFINE(inst)						\
	COND_CODE_1(DT_INST_ON_BUS(inst, spi),				\
		    (MC3419_DEFINE_SPI(inst)),				\
		    (MC3419_DEFINE_I2C(inst)))


DT_INST_FOREACH_STATUS_OKAY(MC3419_DEFINE)
