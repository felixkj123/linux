/*
 * Copyright 2017 IBM Corporation
 *
 * Joel Stanley <joel@jms.id.au>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * The DPS310 is a barometric pressure and temperature sensor.
 * Currently only reading a single temperature is supported by
 * this driver.
 *
 * https://www.infineon.com/dgdl/?fileId=5546d462576f34750157750826c42242
 *
 * Temperature calculation:
 *   c0 * 0.5 + c1 * T_raw / kT °C
 *
 * TODO:
 *  - Pressure sensor readings
 *  - Optionally support the FIFO
 */

#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define DPS310_PRS_B0		0x00
#define DPS310_PRS_B1		0x01
#define DPS310_PRS_B2		0x02
#define DPS310_TMP_B0		0x03
#define DPS310_TMP_B1		0x04
#define DPS310_TMP_B2		0x05
#define DPS310_PRS_CFG		0x06
#define  DPS310_PRS_RATE_BITS	GENMASK(6, 4)
#define  DPS310_PRS_PRC_BITS	GENMASK(3, 0)
#define DPS310_TMP_CFG		0x07
#define  DPS310_TMP_RATE_BITS	GENMASK(6, 4)
#define  DPS310_TMP_PRC_BITS	GENMASK(3, 0)
#define  DPS310_TMP_EXT		BIT(7)
#define DPS310_MEAS_CFG		0x08
#define  DPS310_MEAS_CTRL_BITS	GENMASK(2, 0)
#define   DPS310_PRS_EN		BIT(0)
#define   DPS310_TEMP_EN	BIT(1)
#define   DPS310_BACKGROUND	BIT(2)
#define  DPS310_PRS_RDY		BIT(4)
#define  DPS310_TMP_RDY		BIT(5)
#define  DPS310_SENSOR_RDY	BIT(6)
#define  DPS310_COEF_RDY	BIT(7)
#define DPS310_CFG_REG		0x09
#define  DPS310_INT_HL		BIT(7)
#define  DPS310_TMP_SHIFT_EN	BIT(3)
#define  DPS310_PRS_SHIFT_EN	BIT(4)
#define  DPS310_FIFO_EN		BIT(5)
#define  DPS310_SPI_EN		BIT(6)
#define DPS310_RESET		0x0c
#define  DPS310_RESET_MAGIC	(BIT(0) | BIT(3))
#define DPS310_COEF_BASE	0x10
#define DPS310_NUM_COEF_REGS	0x12

#define DPS310_PRS_BASE		DPS310_PRS_B0
#define DPS310_TMP_BASE		DPS310_TMP_B0

#define DPS310_CALC_RATE(_n)	ilog2(_n)
#define DPS310_CALC_PRC(_n)	ilog2(_n)

#define MCELSIUS_PER_CELSIUS	1000

const int scale_factor[] = {
	 524288,
	1572864,
	3670016,
	7864320,
	 253952,
	 516096,
	1040384,
	2088960,
};

struct dps310_data {
	struct i2c_client *client;
	struct regmap *regmap;

	s32 c0, c1;
	s32 c00, c10, c20, c30, c01, c11, c21;
	s32 pressure_raw;
	s32 temp_raw;
};

static const struct iio_chan_spec dps310_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_OFFSET) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_RAW),
	},
};

/* To be called after checking the COEF_RDY bit in MEAS_CFG */
static int dps310_get_coefs(struct dps310_data *data)
{
	struct regmap *regmap = data->regmap;
	uint8_t coef[DPS310_NUM_COEF_REGS] = {0};
	int r;
	u32 c0, c1;
	u32 c00, c10, c20, c30, c01, c11, c21;

	/* Read all sensor calibration coefficients from the COEF registers. */
	r = regmap_bulk_read(regmap, DPS310_COEF_BASE, coef,
			     DPS310_NUM_COEF_REGS);
	if (r < 0)
		return r;

	/*
	 * Calculate temperature calibration coefficients c0 and c1. The numbers
	 * are 12-bit 2's complement numbers.
	 */
	c0 = (coef[0] << 4) | (coef[1] >> 4);
	data->c0 = sign_extend32(c0, 11);

	c1 = ((coef[1] & GENMASK(3, 0)) << 8) | coef[2];
	data->c1 = sign_extend32(c1, 11);

	/*
	 * Calculate pressure calibration coefficients. c00 and c10 are 20 bit
	 * 2's complement numbers, while the rest are 16 bit 2's complement
	 * numbers.
	 */ 
	c00 = (coef[3] << 12) | (coef[4] << 4) | (coef[5] >> 4);
	data->c00 = sign_extend32(c00, 19);

	c10 = ((coef[5] & GENMASK(3, 0)) << 16) | (coef[6] << 8) | coef[7];
	data->c10 = sign_extend32(c10, 19);

	c01 = (coef[8] << 8) | coef[9];
	data->c01 = sign_extend32(c01, 15);

	c11 = (coef[10] << 8) | coef[11];
	data->c11 = sign_extend32(c11, 15);

	c20 = (coef[12] << 8) | coef[13];
	data->c20 = sign_extend32(c20, 15);

	c21 = (coef[14] << 8) | coef[15];
	data->c21 = sign_extend32(c21, 15);

	c30 = (coef[16] << 8) | coef[17];
	data->c30 = sign_extend32(c30, 15);

	return 0;
}

static int dps310_get_pres_precision(struct dps310_data *data)
{
	int val, r;

	r = regmap_read(data->regmap, DPS310_PRS_CFG, &val);
	if (r < 0)
		return r;

	return BIT(val & GENMASK(2, 0));
}

static int dps310_get_temp_precision(struct dps310_data *data)
{
	int val, r;

	r = regmap_read(data->regmap, DPS310_TMP_CFG, &val);
	if (r < 0)
		return r;

	/*
	 * Scale factor is bottom 4 bits of the register, but 1111 is
	 * reserved so just grab bottom three
	 */
	return BIT(val & GENMASK(2, 0));
}

static int dps310_set_pres_precision(struct dps310_data *data, int val)
{
	int ret;
	u8 shift_en;

	if (val < 0 || val > 128)
		return -EINVAL;

	shift_en = val >= 16 ? DPS310_PRS_SHIFT_EN : 0;
	ret = regmap_write_bits(data->regmap, DPS310_CFG_REG,
				DPS310_PRS_SHIFT_EN, shift_en);
	if (ret)
		return ret;

	return regmap_update_bits(data->regmap, DPS310_PRS_CFG,
				  DPS310_PRS_PRC_BITS, DPS310_CALC_PRC(val));
}

static int dps310_set_temp_precision(struct dps310_data *data, int val)
{
	int ret;
	u8 shift_en;

	if (val < 0 || val > 128)
		return -EINVAL;

	shift_en = val >= 16 ? DPS310_TMP_SHIFT_EN : 0;
	ret = regmap_write_bits(data->regmap, DPS310_CFG_REG,
				DPS310_TMP_SHIFT_EN, shift_en);

	if (ret)
		return ret;

	return regmap_update_bits(data->regmap, DPS310_TMP_CFG,
				  DPS310_TMP_PRC_BITS, DPS310_CALC_PRC(val));
}

static int dps310_set_pres_samp_freq(struct dps310_data *data, int freq)
{
	u8 val;

	if (freq < 0 || freq > 128)
		return -EINVAL;

	val = DPS310_CALC_RATE(freq) << 4;

	return regmap_update_bits(data->regmap, DPS310_PRS_CFG,
				  DPS310_PRS_RATE_BITS, val);
}

static int dps310_set_temp_samp_freq(struct dps310_data *data, int freq)
{
	uint8_t val;

	if (freq < 0 || freq > 128)
		return -EINVAL;

	val = DPS310_CALC_RATE(freq) << 4;

	return regmap_update_bits(data->regmap, DPS310_TMP_CFG,
			DPS310_TMP_RATE_BITS, val);
}

static int dps310_get_pres_samp_freq(struct dps310_data *data)
{
	int val, r;

	r = regmap_read(data->regmap, DPS310_PRS_CFG, &val);
	if (r < 0)
		return r;

	return BIT((val & DPS310_PRS_RATE_BITS) >> 4);
}

static int dps310_get_temp_samp_freq(struct dps310_data *data)
{
	int val, r;

	r = regmap_read(data->regmap, DPS310_TMP_CFG, &val);
	if (r < 0)
		return r;

	return BIT((val & DPS310_TMP_RATE_BITS) >> 4);
}

static int dps310_get_pres_k(struct dps310_data *data)
{
	int r = dps310_get_pres_precision(data);

	if (r < 0)
		return r;

	return scale_factor[DPS310_CALC_PRC(r)];
}

static int dps310_get_temp_k(struct dps310_data *data)
{
	int r = dps310_get_temp_precision(data);

	if (r < 0)
		return r;

	return scale_factor[DPS310_CALC_PRC(r)];
}

static int dps310_read_pres_raw(struct dps310_data *data)
{
	struct device *dev = &data->client->dev;
	uint8_t val[3];
	int r, ready;
	s32 raw;

	r = regmap_read(data->regmap, DPS310_MEAS_CFG, &ready);
	if (r < 0)
		return r;

	if (!(ready & DPS310_PRS_RDY)) {
		dev_dbg(dev, "pressure not ready\n");
		return -EAGAIN;
	}

	r = regmap_bulk_read(data->regmap, DPS310_PRS_BASE, val, 3);
	if (r < 0)
		return r;

	raw = (val[0] << 16) | (val[1] << 8) | val[2];
	data->pressure_raw = sign_extend32(raw, 23);

	return 0;
}

static int dps310_read_temp_raw(struct dps310_data *data)
{
	struct device *dev = &data->client->dev;
	struct regmap *regmap = data->regmap;
	uint8_t val[3] = {0};
	int r, ready;
	int T_raw;

	r = regmap_read(regmap, DPS310_MEAS_CFG, &ready);
	if (r < 0)
		return r;
	if (!(ready & DPS310_TMP_RDY)) {
		dev_dbg(dev, "temperature not ready\n");
		return -EAGAIN;
	}

	r = regmap_bulk_read(regmap, DPS310_TMP_BASE, val, 3);
	if (r < 0)
		return r;

	T_raw = (val[0] << 16) | (val[1] << 8) | val[2];
	data->temp_raw = sign_extend32(T_raw, 23);

	return 0;
}

static bool dps310_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DPS310_PRS_CFG:
	case DPS310_TMP_CFG:
	case DPS310_MEAS_CFG:
	case DPS310_CFG_REG:
	case DPS310_RESET:
	case 0x0e:
	case 0x0f:
	case 0x62:
		return true;
	default:
		return false;
	}
}

static bool dps310_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DPS310_PRS_B0:
	case DPS310_PRS_B1:
	case DPS310_PRS_B2:
	case DPS310_TMP_B0:
	case DPS310_TMP_B1:
	case DPS310_TMP_B2:
	case DPS310_MEAS_CFG:
	case 0x32:
		return true;
	default:
		return false;
	}
}

static int dps310_write_raw(struct iio_dev *iio,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long mask)
{
	struct dps310_data *data = iio_priv(iio);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->type) {
		case IIO_PRESSURE:
			return dps310_set_pres_samp_freq(data, val);

		case IIO_TEMP:
			return dps310_set_temp_samp_freq(data, val);

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		switch (chan->type) {
		case IIO_PRESSURE:
			return dps310_set_pres_precision(data, val);

		case IIO_TEMP:
			return dps310_set_temp_precision(data, val);

		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static int dps310_calculate_pressure(struct dps310_data *data)
{
	int i;
	int r;
	int kpi = dps310_get_pres_k(data);
	int kti = dps310_get_temp_k(data);
	s64 rem = 0ULL;
	s64 pressure = 0ULL;
	s64 p;
	s64 t;
	s64 denoms[7];
	s64 nums[7];
	s64 rems[7];
	s64 kp;
	s64 kt;

	if (kpi < 0)
		return kpi;

	if (kti < 0)
		return kti;

	kp = (s64)kpi;
	kt = (s64)kti;

	/* ignore errors and use the latest */
	dps310_read_temp_raw(data);

	p = (s64)data->pressure_raw;
	t = (s64)data->temp_raw;

	/* section 4.9.1 of the DPS310 spec; algebra'd to avoid underflow */
	nums[0] = (s64)data->c00;
	denoms[0] = 1LL;
	nums[1] = p * (s64)data->c10;
	denoms[1] = kp;
	nums[2] = p * p * (s64)data->c20;
	denoms[2] = kp * kp;
	nums[3] = p * p * p * (s64)data->c30;
	denoms[3] = kp * kp * kp;
	nums[4] = t * (s64)data->c01;
	denoms[4] = kt;
	nums[5] = t * p * (s64)data->c11;
	denoms[5] = kp * kt;
	nums[6] = t * p * p * (s64)data->c21;
	denoms[6] = kp * kp * kt;

	/* kernel lacks a div64_s64_rem function, denoms all positive */
	for (i = 0; i < 7; ++i) {
		u64 rem;

		if (nums[i] < 0LL) {
			pressure -= div64_u64_rem(-nums[i], denoms[i], &rem);
			rems[i] = -rem;
		} else {
			pressure += div64_u64_rem(nums[i], denoms[i], &rem);
			rems[i] = (s64)rem;
		}
	}

	/* increase precision and calculate the remainder sum; worth it? */
	for (i = 0; i < 7; ++i)
		rem += div64_s64((s64)rems[i] * 1000000000LL, denoms[i]);

	pressure += div_s64(rem, 1000000000LL);

	return (int)pressure;
}

static int dps310_read_pressure(struct dps310_data *data, int *val, int *val2,
				long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = dps310_get_pres_samp_freq(data);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		ret = dps310_read_pres_raw(data);
		if (ret)
			return ret;

		*val = dps310_calculate_pressure(data);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 1000; /* convert Pa to KPa per IIO ABI */
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = dps310_get_pres_precision(data);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int dps310_read_temp(struct dps310_data *data, int *val, int *val2,
			    long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = dps310_get_temp_samp_freq(data);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		ret = dps310_read_temp_raw(data);
		if (ret)
			return ret;

		*val = data->temp_raw * data->c1;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_OFFSET:
		*val = (data->c0 >> 1) * dps310_get_temp_k(data);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1000; /* milliCelsius per Celsius */
		*val2 = dps310_get_temp_k(data);
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = dps310_get_temp_precision(data);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int dps310_read_raw(struct iio_dev *iio,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct dps310_data *data = iio_priv(iio);

	switch (chan->type) {
	case IIO_PRESSURE:
		return dps310_read_pressure(data, val, val2, mask);

	case IIO_TEMP:
		return dps310_read_temp(data, val, val2, mask);

	default:
		return -EINVAL;
	}
}

static const struct regmap_config dps310_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = dps310_is_writeable_reg,
	.volatile_reg = dps310_is_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x62,
};

static const struct iio_info dps310_info = {
	.read_raw = dps310_read_raw,
	.write_raw = dps310_write_raw,
};

/*
 * Some verions of chip will read temperatures in the ~60C range when
 * its acutally ~20C. This is the manufacturer recommended workaround
 * to correct the issue.
 */
static int dps310_temp_workaround(struct dps310_data *data)
{
	int r, reg;

	r = regmap_read(data->regmap, 0x32, &reg);
	if (r < 0)
		return r;

	/* If bit 1 is set then the device is okay, and the workaround does not
	 * need to be applied */
	if (reg & BIT(1))
		return 0;

	r = regmap_write(data->regmap, 0x0e, 0xA5);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x0f, 0x96);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x62, 0x02);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x0e, 0x00);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x0f, 0x00);

	return r;
}

static int dps310_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct dps310_data *data;
	struct iio_dev *iio;
	int r, ready;

	iio = devm_iio_device_alloc(&client->dev,  sizeof(*data));
	if (!iio)
		return -ENOMEM;

	data = iio_priv(iio);
	data->client = client;

	iio->dev.parent = &client->dev;
	iio->name = id->name;
	iio->channels = dps310_channels;
	iio->num_channels = ARRAY_SIZE(dps310_channels);
	iio->info = &dps310_info;
	iio->modes = INDIO_DIRECT_MODE;

	data->regmap = devm_regmap_init_i2c(client, &dps310_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	/*
	 * Set up pressure sensor in single sample, one measurement per second
	 * mode
	 */
	r = regmap_write(data->regmap, DPS310_PRS_CFG,
			 DPS310_CALC_RATE(1) | DPS310_CALC_PRC(1));

	/*
	 * Set up external (MEMS) temperature sensor in single sample, one
	 * measurement per second mode
	 */
	r = regmap_write(data->regmap, DPS310_TMP_CFG, DPS310_TMP_EXT |
		DPS310_CALC_RATE(1) | DPS310_CALC_PRC(1));
	if (r < 0)
		return r;

	/* Temp and pressure shifts are disabled when PRC <= 8 */
	r = regmap_write_bits(data->regmap, DPS310_CFG_REG,
			      DPS310_TMP_SHIFT_EN | DPS310_PRS_SHIFT_EN, 0);
	if (r < 0)
		return r;

	/* MEAS_CFG doesn't seem to update unless first written with 0... */
	r = regmap_write_bits(data->regmap, DPS310_MEAS_CFG,
			      DPS310_MEAS_CTRL_BITS, 0);
	if (r < 0)
		return r;

	/* Turn on temperature and pressure measurement in the background */
	r = regmap_write_bits(data->regmap, DPS310_MEAS_CFG,
			DPS310_MEAS_CTRL_BITS,
			DPS310_PRS_EN | DPS310_TEMP_EN | DPS310_BACKGROUND);
	if (r < 0)
		return r;

	/*
	 * Calibration coefficients required for reporting temperature.
	 * They are available 40ms after the device has started
	 */
	r = regmap_read_poll_timeout(data->regmap, DPS310_MEAS_CFG, ready,
			ready & DPS310_COEF_RDY,
			10 * 1000,
			40 * 1000);
	if (r < 0)
		return r;

	r = dps310_get_coefs(data);
	if (r < 0)
		return r;

	r = dps310_temp_workaround(data);
	if (r < 0)
		return r;

	r = devm_iio_device_register(&client->dev, iio);
	if (r)
		return r;

	i2c_set_clientdata(client, iio);

	dev_info(&client->dev, "%s: sensor '%s'\n", dev_name(&iio->dev),
			client->name);

	return 0;
}

static int dps310_remove(struct i2c_client *client)
{
	struct dps310_data *data = i2c_get_clientdata(client);

	return regmap_write(data->regmap, DPS310_RESET, DPS310_RESET_MAGIC);
}

static const struct i2c_device_id dps310_id[] = {
	{ "dps310", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, dps310_id);

static const unsigned short normal_i2c[] = {
	0x77, 0x76, I2C_CLIENT_END
};

static struct i2c_driver dps310_driver = {
	.driver = {
		.name = "dps310",
	},
	.probe = dps310_probe,
	.remove = dps310_remove,
	.address_list = normal_i2c,
	.id_table = dps310_id,
};
module_i2c_driver(dps310_driver);

MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_DESCRIPTION("Infineon DPS310 pressure and temperature sensor");
MODULE_LICENSE("GPL");
