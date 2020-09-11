/* drivers/staging/iio/magnetometer/ak09940.c
 *
 * ak09940.c -- A sensor driver for the 3D magnetometer AK09940.
 *
 * Copyright (C) 2019 Asahi Kasei Microdevices Corporation
 *					   Date		Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *					  19/03/27		0.1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/*
 * #define KERNEL_3_18_XX
 */

/*
 * #define AK09940_DEBUG
 */

#ifdef AK09940_DEBUG
#define akdbgprt dev_dbg
#else
#define akdbgprt(format, arg ...)  do {} while (0)
#endif


/*
 * Register definitions, as well as various shifts and masks to get at the
 * individual fields of the registers.
 */


/* REGISTER MAP */
#define AK09940_REG_WIA				0x00
#define AK09940_DEVICE_ID				0x48A1


#define AK09940_REG_ST1				0x10
#define AK09940_REG_HXL				0x11
#define AK09940_REG_HYL				0x14
#define AK09940_REG_HZL				0x17


#define AK09940_REG_TMPS			0x1A
#define AK09940_REG_ST2				0x1B

#define AK09940_REG_CNTL1			0x30
#define AK09940_REG_CNTL2			0x31
#define AK09940_REG_CNTL3			0x32
#define AK09940_REG_SRST				0x33
#define AK09940_REG_I2CDIS			0x36

#define AK09940_MAX_REGS			AK09940_REG_I2CDIS

#define AK09940_MEASUREMENT_WAIT_TIME		2

#define AK09940_REG_HXL_HZH_LENGTH		9
#define AK09940_REG_ST1_ST2_LENGTH		12
#define AK09940_REG_HXL_ST2_LENGTH		11	//no ST1 in FIFO

#define AK09940_MODE_PDN			0x00
#define AK09940_MODE_SNG			0x01
#define AK09940_MODE_CONT_10HZ			0x02
#define AK09940_MODE_CONT_20HZ			0x04
#define AK09940_MODE_CONT_50HZ			0x06
#define AK09940_MODE_CONT_100HZ		0x08
#define AK09940_MODE_CONT_200HZ		0x0A
#define AK09940_MODE_CONT_400HZ		0x0C
#define AK09940_MODE_SELFTEST			0x10
#define AK09940_MODE_NUM			9


/* AK09915 selftest threshold */
#define AK09940_TEST_LOLIM_X			(-1200)
#define AK09940_TEST_HILIM_X			(-300)
#define AK09940_TEST_LOLIM_Y			(300)
#define AK09940_TEST_HILIM_Y			(1200)
#define AK09940_TEST_LOLIM_Z			(-1600)
#define AK09940_TEST_HILIM_Z			(-400)

#define AK09940_WM(wm)				((wm - 1) & 0x07)
#define AK09940_WM_EN(en)			((en) << 7)
#define AK09940_MT(mt)				((mt) << 5)

#define AK09940_WM_EN_MASK			0x80
#define AK09940_WM_EN_SHIFT		7
#define AK09940_MT_MASK			0x60
#define AK09940_MT_SHIFT			5
#define AK09940_MODE_MASK			0x1F
#define AK09940_MODE_SHIFT			0
#define AK09940_FNUM_MASK			0x1e
#define AK09940_FNUN_SHIFT			1
#define AK09940_FIFO_MAX_SIZE		8
#define AK09940_FIFO_INV_BIT_MASK		0x02
#define AK09940_DRDY_MASK			0x01
#define AK09940_TEMP_BIT_MASK		0x40
#define AK09940_TEMP_BIT_SHIFT		6

#define AK09940_DATA_OVERFLOW_VALUE		0x1FFFF
#define AK09940_OVERFLOW			1

#define AK09940_PDN_MODE_DELAY	20

/* for selsf-test start*/
#define TLIMIT_NO_CNT_1ST			0x101
#define TLIMIT_LO_CNT_1ST			1
#define TLIMIT_HI_CNT_1ST			1
#define TLIMIT_NO_CNT_2ND			0x102
#define TLIMIT_LO_CNT_2ND			0
#define TLIMIT_HI_CNT_2ND			0
#define TLIMIT_OVERFLOW			0x103

#define TLIMIT_NO_SLF_RVHX			0x201
#define TLIMIT_NO_SLF_RVHY			0x202
#define TLIMIT_NO_SLF_RVHZ			0x203

#define TLIMIT_LO_SLF_RVHX_AK09940	-1200
#define TLIMIT_HI_SLF_RVHX_AK09940	-300
#define TLIMIT_LO_SLF_RVHY_AK09940	300
#define TLIMIT_HI_SLF_RVHY_AK09940	1200
#define TLIMIT_LO_SLF_RVHZ_AK09940	-1600
#define TLIMIT_HI_SLF_RVHZ_AK09940	-400

/* for selsf-test end*/

#define SELF_TEST_SUCCESS	0
#define SELF_TEST_FAIL		1

/* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
#define AK09940_IIO_DEVICE_ATTR_PERMISSION	0660

#define NUM_OF_AXIS				3

#define RAW_DATA_TO_Q10(x)		(x << 10)

enum {
	AK09940_ST1_POS = 0,
	AK09940_DATA_POS = 1,
	AK09940_TMPS_POS = 10,
	AK09940_ST2_POS = 11
} AK09940_REG_POS_ENUM;
enum {
	AK09940_FIFO_DATA_POS = 0,
	AK09940_FIFO_TMPS_POS = 9,
	AK09940_FIFO_ST2_POS = 10
} AK09940_FIFO_REG_POS_ENUM;
/*
 *0 : Power Down
 *1 : Measurement Mode 1
 *2 : Measurement Mode 2
 *3 : Measurement Mode 3
 *4 : Measurement Mode 4
 *5 : Measurement Mode 5
 *6 : Measurement Mode 6
 */

struct FreqModeTable {
	int freq;
	u8 reg;
} FreqModeTableStruct;
static struct FreqModeTable measurementFreqModeTable[] = {
	{0,	AK09940_MODE_PDN},
	{10,	AK09940_MODE_CONT_10HZ},
	{20,	AK09940_MODE_CONT_20HZ},
	{50,	AK09940_MODE_CONT_50HZ},
	{100,	AK09940_MODE_CONT_100HZ},
	{200,	AK09940_MODE_CONT_200HZ},
	{400,	AK09940_MODE_CONT_400HZ}
};

/*
 * Per-instance context data for the device.
 */
struct ak09940_data {
	struct i2c_client  *client;
	struct iio_trigger *trig;
	/* to protect fifo read flow*/
	struct mutex fifo_mutex;
	struct work_struct	flush_work;
	struct workqueue_struct *wq;
	int				rstn_gpio;
	int				int_gpio;
	int				irq;
	/* this value represents current operation mode.
	 * this value should be register value
	 * CNTL3(bit0:bit4)
	 */
	u8			mode;
	struct FreqModeTable	*freqmodeTable;
	u8				 freq_num;

	u8				 MTbit;
	u8				 selftest;
	/* size of FIFO, range is 1 - 8 */
	u8				 watermark;
	u8				 watermark_en;
	u8				 TEMPbit;

	/* Axis conversion */
	u8				 axis_order[NUM_OF_AXIS];
	u8				 axis_sign[NUM_OF_AXIS];

	s32				test_lolim[3];
	s32				test_hilim[3];


	/* previous timestamp [nsec] */
	s64				prev_time_ns;

};

static void ak09940_fifo_read_and_event(struct iio_dev *indio_dev);

/********************  Register R/W  *********************/
static int ak09940_i2c_reads(
	struct i2c_client *client,
	u8				*reg,
	int			   reglen,
	u8				*rdata,
	int			   datalen)
{
	struct i2c_msg xfer[2];
	int			ret;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = reglen;
	xfer[0].buf = reg;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = datalen;
	xfer[1].buf = rdata;

	ret = i2c_transfer(client->adapter, xfer, 2);

	if (ret == 2)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int ak09940_i2c_read(
	struct i2c_client *client,
	u8				address,
	int			   rLen,
	u8				*rdata)
{
	u8  tx[1];
	int i, ret;

	akdbgprt(&client->dev,
		"[AK09940] %s address=%x, length=%d\n",
		__func__, (int)address, rLen);

	tx[0] = address;

	ret = ak09940_i2c_reads(client, tx, 1, rdata, rLen);

	if (ret < 0) {
		dev_err(&client->dev, "[AK09940] I2C Read Error\n");

		for (i = 0; i < rLen; i++)
			rdata[i] = 0;
	}

	return ret;
}

static int ak09940_i2c_write(
	struct i2c_client *client,
	u8				address,
	u8				value)
{
	int ret;

	akdbgprt(&client->dev,
		"[AK09940] %s (0x%02X, 0x%02X)\n",
		__func__, (int)address, (int)value);

	ret = i2c_smbus_write_byte_data(client, address, value);

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: comm error, ret= %d\n",
			__func__, ret);
	}

	return ret;
}

/*******************************************************************/

#define AK099XX_STATUS_CHANNEL(index)	 { \
	.type = IIO_MAGN, \
	.modified = 1, \
	.channel2 = IIO_NO_MOD, \
	.info_mask_separate = \
		BIT(IIO_CHAN_INFO_RAW), \
	.scan_index = index, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = 16, \
	}, \
}

#define AK099XX_MAG_CHANNEL(axis, index)  {  \
	.type = IIO_MAGN, \
	.modified = 1, \
	.channel2 = IIO_ ## axis, \
	.info_mask_separate = \
		BIT(IIO_CHAN_INFO_RAW) | \
		BIT(IIO_CHAN_INFO_SCALE), \
	.info_mask_shared_by_type = \
		BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = index, \
	.scan_type = { \
		.sign = 's', \
		.realbits = 18, \
		.storagebits = 18, \
		.shift = 0, \
	}, \
}

static const struct iio_chan_spec ak09940_channels[] = {
	AK099XX_STATUS_CHANNEL(0),
	AK099XX_MAG_CHANNEL(MOD_X, 1),
	AK099XX_MAG_CHANNEL(MOD_Y, 2),
	AK099XX_MAG_CHANNEL(MOD_Z, 3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};
/*
 * check operation mode
 */
static int ak09940_check_mode_available(
	struct ak09940_data *akm,
	u8				  mode)
{
	u8 i = 0;

	akdbgprt(&akm->client->dev,
		"[AK09940] %s called, mode=%d", __func__, mode);
	if ((mode == AK09940_MODE_SNG) || (mode == AK09940_MODE_SELFTEST))
		return 0;

	for (i = 0; i < akm->freq_num; i++) {
		if (akm->freqmodeTable[i].reg == mode)
			return 0;
	}
	return -EINVAL;
}
/*
 * check watermark value, range is 1-8
 */
static int check_watermark_available(u8 watermark)
{
	if ((watermark < 1) || (watermark > 8))
		return -EINVAL;
	return 0;
}
/*
 * update watermark_en flag by watermark value
 */
static void update_watermark_flag(
	struct ak09940_data *akm,
	u8 value)
{
	akm->watermark = value;
	/* according watermark reset watermark_en*/
	if (akm->watermark == 1)
		akm->watermark_en = 0;
	else if ((akm->watermark > 1) &&
		(akm->watermark <= 8)) {
		akm->watermark_en = 1;
	}
}
/*
 * if change operation mode, transit to PDN mode first
 * the SNG mode and self-test mode will transit to PDN mode automatically
 * return 0 means chip could change mode directly
 */
static int check_mode_change_directly(
	struct ak09940_data *akm)
{
	if ((akm->mode == AK09940_MODE_PDN) ||
		(akm->mode == AK09940_MODE_SNG) ||
		(akm->mode == AK09940_MODE_SELFTEST))
		return 0;
	else
		return -EBUSY;
}
/*
 * if change operation mode, transit to PDN mode first
 * the SNG mode and self-test mode will transit to PDN mode automatically
 */
static void set_start_measure_time(
	struct ak09940_data *akm)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(akm);
#ifdef KERNEL_3_18_XX
	akm->prev_time_ns = iio_get_time_ns();
#else
	akm->prev_time_ns = iio_get_time_ns(indio_dev);
#endif
}
/*
 * set watermar bits(CNTL1)
 * It is prohibited to change WM[2:0] bits
 * in any other modes than Power-down mode.
 */
static int ak09940_set_watermark(
	struct ak09940_data *akm,
	u8 value)
{
	int error = 0;

	if (value == akm->watermark)
		return error;

	/*change CTNL1, must in PDN mode*/
	if (akm->mode == AK09940_MODE_PDN) {
		/* 1. check watermark value */
		error = check_watermark_available(value);
		if (value < 0) {
			dev_err(&akm->client->dev,
				"[AK09940] %s watermark value error\n",
				__func__);
			return error;
		}
		/* 2. set CTNL1 register */
		/*register value range 0-7*/
		/*watermark size value range 1-8*/
		error = ak09940_i2c_write(
			akm->client,
			AK09940_REG_CNTL1,
			value - 1);
		if (error) {
		/* I2C write failed*/
			return error;
		}
		/* 5. set watermark and watermark_en */
		update_watermark_flag(akm, value);
	} else {
		dev_err(&akm->client->dev,
			"[AK09940] %s chip is busy now, set PDN mode first\n",
			__func__);
		return -EBUSY;
	}
	return error;
}
/*
 * set temperature bits(CNTL2)
 */
static int ak09940_set_temperature_en(
	struct ak09940_data *akm,
	u8 value)
{
	u8  cntl2_value = 0;
	int error = 0;

	if (value == akm->TEMPbit)
		return error;

	cntl2_value = (value << AK09940_TEMP_BIT_SHIFT) &&
		AK09940_TEMP_BIT_MASK;
	/*change CTNL2, must in PDN mode*/
	if (akm->mode == AK09940_MODE_PDN) {
		error = ak09940_i2c_write(
			akm->client,
			AK09940_REG_CNTL2,
			cntl2_value);
		if (error) {
			/* I2C write failed*/
			return error;
		}
	} else {
		dev_err(&akm->client->dev,
			"[AK09940] %s chip is busy now, set PDN mode first\n",
			__func__);
		return -EBUSY;
	}
	return error;
}
/*
 * set sensor drive bits(CNTL3 MT[1:0])
 */
static int ak09940_set_sensor_drive(
	struct ak09940_data *akm,
	u8 value)
{
	u8  cntl3_value = 0;
	int error = 0;

	if (value == akm->MTbit)
		return error;

	/*change MT[1:0], must in PDN mode*/
	if (akm->mode == AK09940_MODE_PDN) {
		cntl3_value =
			AK09940_WM_EN(akm->watermark_en) +
			AK09940_MT(value);
		error = ak09940_i2c_write(
			akm->client,
			AK09940_REG_CNTL3,
			cntl3_value);
		if (error) {
			/* I2C write failed*/
			return error;
		}
	} else {
		dev_err(&akm->client->dev,
			"[AK09940] %s chip is busy now, set PDN mode first\n",
			__func__);
		return -EBUSY;
	}
	return error;
}
/*
 * set chip to PDN mode
 * clear all other bits(WM[2:0]/TEM/FIFO/MT[1:0])
 */
static int ak09940_set_PDN_mode(
	struct ak09940_data *akm)
{
	u8  cntl3_value = 0;
	int error = 0;
	struct iio_dev *indio_dev = iio_priv_to_dev(akm);

	if (akm->mode == AK09940_MODE_PDN)
		return error;
	/* if use FIFO mode, read out all FIFO data first*/
	if (akm->watermark_en == 1) {
		mutex_lock(&akm->fifo_mutex);
		ak09940_fifo_read_and_event(indio_dev);
		mutex_unlock(&akm->fifo_mutex);
	}
	cntl3_value =
		AK09940_WM_EN(akm->watermark_en) +
		AK09940_MT(akm->MTbit) +
		AK09940_MODE_PDN;
	error = ak09940_i2c_write(
		akm->client,
		AK09940_REG_CNTL3,
		cntl3_value);
	if (error) {
		/* I2C write failed*/
		return error;
	}
	akm->mode = AK09940_MODE_PDN;
	udelay(AK09940_PDN_MODE_DELAY);
	return error;
}
/*
 * set chip to SNG mode, do not set FIFO bit
 * It is prohibited to enable FIFO other than Continuous measurement mode.
 */
static int ak09940_set_single_measure(
	struct ak09940_data *akm)
{
	int error = 0;
	u8  cntl3_value = 0;

	error = check_mode_change_directly(akm);
	if (error) {
		dev_err(&akm->client->dev,
			"[AK09940] %s chip is busy now, set PDN mode first\n",
			__func__);
		return error;
	}
	cntl3_value =
		AK09940_MT(akm->MTbit) +
		AK09940_MODE_SNG;
	error = ak09940_i2c_write(
		akm->client,
		AK09940_REG_CNTL3,
		cntl3_value);
	if (error) {
		/* I2C write failed */
		dev_err(&akm->client->dev,
			"[AK09940] %s,%d failed\n",
			__func__, __LINE__);
		return error;
	}
	return error;
}
/*
 * set chip to self-test mode, do not set FIFO bit, MT[1:0] = "11"
 * It is prohibited to enable FIFO other than Continuous measurement mode.
 */
static int ak09940_set_selftest_measure(
	struct ak09940_data *akm)
{
	int error = 0;
	u8  cntl3_value = 0;
	u8 temp_mt = 0x3;

	error = check_mode_change_directly(akm);
	if (error) {
		dev_err(&akm->client->dev,
			"[AK09940] %s chip is busy now, set PDN mode first\n",
			__func__);
		return error;
	}
	cntl3_value =
		AK09940_MT(temp_mt) +
		AK09940_MODE_SELFTEST;
	error = ak09940_i2c_write(
		akm->client,
		AK09940_REG_CNTL3,
		cntl3_value);
	if (error) {
		/* I2C write failed */
		dev_err(&akm->client->dev,
			"[AK09940] %s,%d failed\n",
			__func__, __LINE__);
		return error;
	}
	return error;
}
/*
 * set chip to continue measurement mode
 */
static int ak09940_set_continue_measure(
	struct ak09940_data *akm,
	u8 mode)
{
	int error = 0;
	u8  cntl3_value = 0;

	if (akm->mode == mode) {
		/* new mode is the same with current mode*/
		akdbgprt(&akm->client->dev,
		"[AK09940] %s mode is the same, no need to set", __func__);
		return error;
	}
	error = check_mode_change_directly(akm);
	if (error) {
		dev_err(&akm->client->dev,
			"[AK09940] %s chip is busy now, set PDN mode first\n",
			__func__);
		return error;
	}
	if (mode == AK09940_MODE_CONT_400HZ) {
		if ((akm->MTbit != 0) &&
			(akm->MTbit != 1)) {
			dev_err(&akm->client->dev,
				"[AK09940] %s MT[1:0] must be 0 or 1 in 400HZ\n",
				__func__);
			return -EINVAL;
		}
	}
	cntl3_value = AK09940_WM_EN(akm->watermark_en) +
		AK09940_MT(akm->MTbit) +
		mode;
	error = ak09940_i2c_write(
		akm->client,
		AK09940_REG_CNTL3,
		cntl3_value);
	if (error) {
		/* I2C write failed */
		dev_err(&akm->client->dev,
			"[AK09940] %s set PDN failed\n",
			__func__);
		return error;
	}
	set_start_measure_time(akm);
	return error;
}
static int ak09940_set_mode(
	struct ak09940_data *akm,
	u8				  mode)
{
	int error = 0;

	akdbgprt(&akm->client->dev,
		"[AK09940] %s called", __func__);

	error = ak09940_check_mode_available(akm, mode);
	if (error)
		return error;
	switch (mode) {
	case AK09940_MODE_PDN:
		error = ak09940_set_PDN_mode(akm);
		if (error) {
			/* set PDN mode failed */
			dev_err(&akm->client->dev,
				"[AK09940] %s,%d failed\n",
				__func__, __LINE__);
			return error;
		}
		break;
	case AK09940_MODE_SNG:
		error = ak09940_set_single_measure(akm);
		if (error) {
			/* set SNG mode failed */
			dev_err(&akm->client->dev,
				"[AK09940] %s,%d failed\n",
				__func__, __LINE__);
			return error;
		}
		break;
	case AK09940_MODE_SELFTEST:
		error = ak09940_set_selftest_measure(akm);
		if (error) {
			/* set SNG mode failed */
			dev_err(&akm->client->dev,
				"[AK09940] %s,%d failed\n",
				__func__, __LINE__);
			return error;
		}
		break;
	case AK09940_MODE_CONT_10HZ:
	case AK09940_MODE_CONT_20HZ:
	case AK09940_MODE_CONT_50HZ:
	case AK09940_MODE_CONT_100HZ:
	case AK09940_MODE_CONT_200HZ:
	case AK09940_MODE_CONT_400HZ:
		error = ak09940_set_continue_measure(akm, mode);
		if (error) {
			/* set SNG mode failed */
			dev_err(&akm->client->dev,
				"[AK09940] %s,%d failed\n",
				__func__, __LINE__);
			return error;
		}
		break;
	}
	akm->mode = mode;
	return error;
}
static u8 ak09940_get_continue_mode_by_interval(
	struct ak09940_data *akm,
	int				 interval)
{
	int n = 0;
	int mode = AK09940_MODE_PDN;
	int freq = 0;

	akdbgprt(&akm->client->dev,
		"[AK09940] %s inverval = %d\n", __func__, interval);

	if (interval < 0) {
		dev_err(&akm->client->dev, "[AK09940] %s Val Error val = %d\n",
				__func__, interval);
		return mode;
	}

	if (interval == 0)
		return mode;

	freq = 1000 / interval;

	for (n = 0; n < akm->freq_num - 1; n++) {
		if (freq <= akm->freqmodeTable[n].freq)
			break;
	}

	mode = akm->freqmodeTable[n].reg;
	akdbgprt(&akm->client->dev,
		"[AK09940] %s mode = %d\n", __func__, mode);

	return mode;
}
static int ak09940_get_samp_freq(
	struct ak09940_data *akm,
	int mode)
{
	u8 i;
	/* -1 means unknown. e.g. selftest mode etc. */
	int freq = -1;

	for (i = 0; i < akm->freq_num; i++) {
		if (mode == akm->freqmodeTable[i].reg) {
			freq = akm->freqmodeTable[i].freq;
			break;
		}
	}
	return freq;
}
/*
 * convert from reg data to mag data
 * reg: from HXL to HZH
 * mag: 3axis 18 bit mag data
 */
static int ak09940_parse_raw_data(
	u8  *reg,
	s32 *mag)
{
	int i = 0;

	if (reg == NULL)
		return -EINVAL;

	if (mag == NULL)
		return -EINVAL;

	/* Store data */
	for (i = 0; i < NUM_OF_AXIS; i++) {
		/* convert to int32 data */
		*(mag + i) = (int32_t)
			(((uint32_t)reg[i * 3 + 2] << 24) |
			 ((uint32_t)reg[i * 3 + 1] << 16) |
			 ((uint32_t)reg[i * 3] << 8)) >> 8;
	}
	return 0;
}

static int ak09940_data_check_overflow(s32 *mag)
{
	int i = 0;

	if (mag == NULL)
		return -EINVAL;
	for (i = 0; i < NUM_OF_AXIS; i++) {
		if (mag[i] == AK09940_DATA_OVERFLOW_VALUE)
			return AK09940_OVERFLOW;
	}

	return 0;
}
static int ak09940_software_reset(
	struct ak09940_data *akm)
{
	int error = 0;
	u8  address = 0;
	u8  value = 0;

	akdbgprt(&akm->client->dev,
		"[AK09940] %s called", __func__);
	address = AK09940_REG_SRST;
	value = 0x01;
	error = ak09940_i2c_write(akm->client, address, value);
	if (error) {
		/* I2C write failed */
		dev_err(&akm->client->dev,
			"[AK09940] %s set software reset failed\n",
			__func__);
		return error;
	}
	/* reset variables */
	akm->mode = AK09940_MODE_PDN;
	akm->watermark = 0;
	akm->watermark_en = 0;
	akm->TEMPbit = 1;
	akm->MTbit = 0;
	/* reset timestamp */
	akm->prev_time_ns = 0;
	return error;
}
static int ak09940_read_device_and_check(
	struct ak09940_data *akm)
{
	int ret = 0;
	u8 deviceID[2] = {0};

	ret = ak09940_i2c_read(akm->client, AK09940_REG_WIA, 2, deviceID);

	if (ret < 0) {
		dev_err(&akm->client->dev,
			"[AK09940]i2c read failure, AK09940_REG_WIA err=%d\n",
			ret);
		return -EIO;
	}

	akdbgprt(&akm->client->dev,
		"[AK09940] Device ID = %x,%x\n", deviceID[0], deviceID[1]);

	if ((deviceID[0] << 8 | deviceID[1]) != AK09940_DEVICE_ID) {
		dev_err(&akm->client->dev,
			"[AK09940] Device ID = %x,%x\n",
			deviceID[0], deviceID[1]);
		return -ENODEV;
	}
	return ret;
}

static ssize_t attr_setting_reg_show(
	struct device		   *dev,
	struct device_attribute *attr,
	char					*buf)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
	u8				  cntl[3] = {0};

	akdbgprt(dev, "[AK09940] %s called", __func__);

	ak09940_i2c_read(akm->client, AK09940_REG_CNTL1, 3, cntl);
	dev_info(dev, "[AK09940] CNTL1(30H)=%XH", (int)cntl[0]);
	dev_info(dev, "[AK09940] CNTL2(31H)=%XH", (int)cntl[1]);
	dev_info(dev, "[AK09940] CNTL3(32H)=%XH", (int)cntl[2]);

	return snprintf(buf, 10, "%02X,%02x,%02x\n", cntl[0], cntl[1], cntl[2]);
}

static IIO_DEVICE_ATTR(setting_reg,
		AK09940_IIO_DEVICE_ATTR_PERMISSION,
		attr_setting_reg_show,
		NULL,
		0);

static ssize_t attr_data_reg_show(
	struct device		   *dev,
	struct device_attribute *attr,
	char					*buf)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
	/* ST1+FIFO data length*/
	/* FIFO data length: (HXL_ST2_LENGTH) * FIFO MAX STEP*/
	u8 result[AK09940_REG_HXL_ST2_LENGTH * AK09940_FIFO_MAX_SIZE + 1] = {0};
	s32 mag[3];
	int ret;
	int i = 0;
	u8 st1 = 0;
	int event_num = 0;
	int read_bytes = 0;
	int event_pos = 0;
	int str_len = 0;

	akdbgprt(dev, "[AK09940] %s called", __func__);

	if (akm->watermark_en == 0) {
		ret = ak09940_i2c_read(akm->client,
			AK09940_REG_ST1,
			AK09940_REG_ST1_ST2_LENGTH,
			result);

#ifdef AK09940_DEBUG
		for (i = 0; i < AK09940_REG_ST1_ST2_LENGTH; i++) {
			akdbgprt(dev,
				"[AK09940] %s %d %02XH\n",
				__func__, i, result[i]);
		}
#endif

		if (ret < 0)
			return ret;

		ak09940_parse_raw_data(&result[AK09940_DATA_POS], mag);

		if (ak09940_data_check_overflow(mag)) {
			dev_err(dev,
				"[AK09940] %s read mag data overflow!!!!!!!!!",
				__func__);
		}
		return snprintf(buf, 34, "%02x,%08X,%08x,%08x,%02x\n",
			result[AK09940_ST1_POS],
			mag[0], mag[1], mag[2],
			result[AK09940_ST2_POS]);
	} else {
		mutex_lock(&akm->fifo_mutex);
		/* FIFO is enable */
		/* 1. check how many event in FIFO by FNUM */
		ret = ak09940_i2c_read(akm->client,
			AK09940_REG_ST1,
			1,
			result);
		st1 = result[0];
		event_num = (st1 & AK09940_FNUM_MASK) >>
			AK09940_FNUN_SHIFT;
		/* total bytes is ST1 + event_num * AK09940_DATA_FIFO_LENGTH*/
		read_bytes = event_num * AK09940_REG_HXL_ST2_LENGTH + 1;
		ret = ak09940_i2c_read(akm->client,
			AK09940_REG_ST1,
			read_bytes,
			result);
#ifdef AK09940_DEBUG
		for (i = 0; i < read_bytes; i++) {
			akdbgprt(dev,
				"[AK09940] %s %d %02XH\n",
				__func__, i, result[i]);
		}
#endif
		/* 2. process every events*/
		for (i = 0; i < event_num; i++) {
			event_pos = i * AK09940_REG_HXL_ST2_LENGTH + 1;
			ak09940_parse_raw_data(
				&result[event_pos + AK09940_DATA_POS],
				mag);
			if (ak09940_data_check_overflow(mag)) {
				dev_err(dev,
					"[AK09940] %s read mag data overflow!!!!!!!!!",
					__func__);
			}
			str_len += snprintf(buf, 34,
				"%02x,%08X,%08x,%08x,%02x\n",
				result[event_pos],
				mag[0], mag[1], mag[2],
				result[event_pos + AK09940_ST2_POS]);
		}
		mutex_unlock(&akm->fifo_mutex);
		return str_len;
	}
}

static IIO_DEVICE_ATTR(data_reg,
		AK09940_IIO_DEVICE_ATTR_PERMISSION,
		attr_data_reg_show,
		NULL,
		0);
static int ak0991x_test_threshold(
	uint16_t testno,
	int16_t testdata,
	int16_t lolimit,
	int16_t hilimit,
	uint32_t *err)
{
	if ((lolimit <= testdata) && (testdata <= hilimit))
		return SELF_TEST_SUCCESS;

	*err = (uint32_t)((((uint32_t)testno) << 16) |
		((uint16_t)testdata));
	return SELF_TEST_FAIL;
}
static ssize_t attr_selftest_show(
	struct device		   *dev,
	struct device_attribute *attr,
	char					*buf)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
	u8  result[AK09940_REG_ST1_ST2_LENGTH];
	s32 mag[3];
	int error = 0;
	u8 drdy = 0;
	u32 err;

	akdbgprt(dev, "[AK09940] %s called", __func__);
	/* reset chip first */
	error = ak09940_software_reset(akm);
	if (error)
		return error;

	/* Step 1: check measurement data*/
	/* set to sngle mode*/
	error = ak09940_set_mode(akm, AK09940_MODE_SNG);
	if (error)
		return error;

	/* wait for double length measurement time*/
	msleep(20);
	/* get measurement data for 1st */
	ak09940_i2c_read(akm->client,
		AK09940_REG_ST1,
		AK09940_REG_ST1_ST2_LENGTH,
		result);
	/* measurement finish, so DRDY should be 1*/
	drdy = result[0] & AK09940_DRDY_MASK;
	if (ak0991x_test_threshold(TLIMIT_NO_CNT_1ST,
		drdy,
		TLIMIT_LO_CNT_1ST,
		TLIMIT_HI_CNT_1ST,
		&err))
		goto SELF_TEST_FAILED;
	/* get measurement data for 1st */
	ak09940_i2c_read(akm->client,
		AK09940_REG_ST1,
		AK09940_REG_ST1_ST2_LENGTH,
		result);
	/* data have been read out, so DRDY should be 0*/
	drdy = result[0] & AK09940_DRDY_MASK;
	if (ak0991x_test_threshold(TLIMIT_NO_CNT_2ND,
		drdy,
		TLIMIT_LO_CNT_2ND,
		TLIMIT_HI_CNT_2ND,
		&err))
		goto SELF_TEST_FAILED;
	/* get magnetic data */
	ak09940_parse_raw_data(result+1, mag);
	/* check data overflow */
	error = ak09940_data_check_overflow(mag);
	if (error) {
		error = TLIMIT_OVERFLOW << 16 | error;
		goto SELF_TEST_FAILED;
	}

	/* reset chip */
	error = ak09940_software_reset(akm);
	if (error)
		return error;

	/* Step 2: check self-test mode*/
	error = ak09940_set_mode(akm, AK09940_MODE_SELFTEST);
	if (error)
		return error;

	/* wait for double length measurement time*/
	msleep(20);
	/* get measurement data */
	ak09940_i2c_read(akm->client,
		AK09940_REG_ST1,
		AK09940_REG_ST1_ST2_LENGTH,
		result);

	/* get magnetic data */
	ak09940_parse_raw_data(result+1, mag);
	/* check value */
	if (ak0991x_test_threshold(TLIMIT_NO_SLF_RVHX,
		mag[0],
		TLIMIT_LO_SLF_RVHX_AK09940,
		TLIMIT_HI_SLF_RVHX_AK09940,
		&err))
		goto SELF_TEST_FAILED;
	if (ak0991x_test_threshold(TLIMIT_NO_SLF_RVHY,
		mag[1],
		TLIMIT_LO_SLF_RVHY_AK09940,
		TLIMIT_HI_SLF_RVHY_AK09940,
		&err))
		goto SELF_TEST_FAILED;
	if (ak0991x_test_threshold(TLIMIT_NO_SLF_RVHZ,
		mag[2],
		TLIMIT_LO_SLF_RVHZ_AK09940,
		TLIMIT_HI_SLF_RVHZ_AK09940,
		&err))
		goto SELF_TEST_FAILED;
	return snprintf(buf, 6, "pass\n");
SELF_TEST_FAILED:
	dev_err(dev,
		"[AK09940] selftest faile, err=%x",
		err);
	return snprintf(buf, 8, "failed\n");
}

static IIO_DEVICE_ATTR(selftest,
		AK09940_IIO_DEVICE_ATTR_PERMISSION,
		attr_selftest_show,
		NULL,
		0);

static ssize_t attr_operation_mode_show(
	struct device		   *dev,
	struct device_attribute *attr,
	char					*buf)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));

	akdbgprt(dev, "[AK09940] %s called", __func__);

	return snprintf(buf, 4, "%02X\n", akm->mode);
}
static ssize_t attr_operation_mode_store(
	struct device		   *dev,
	struct device_attribute *attr,
	const char			  *buf,
	size_t				  count)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
	long mode = 0;
	int error = 0;

	akdbgprt(dev, "[AK09940] %s called, buf=%s", __func__, buf);
	error = kstrtol(buf, 10, &mode);
	if (error)
		return error;
	ak09940_set_mode(akm, (u8)mode);
	return count;
}
static IIO_DEVICE_ATTR(operation_mode,
		AK09940_IIO_DEVICE_ATTR_PERMISSION,
		attr_operation_mode_show,
		attr_operation_mode_store,
		0);

static ssize_t attr_watermark_show(
	struct device		   *dev,
	struct device_attribute *attr,
	char					*buf)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));

	akdbgprt(dev, "[AK09940] %s called", __func__);

	return snprintf(buf, 4, "%02X\n", akm->watermark);
}
static ssize_t attr_watermark_store(
	struct device		   *dev,
	struct device_attribute *attr,
	const char			  *buf,
	size_t				  count)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
	long watermark = 0;
	int error = 0;

	akdbgprt(dev, "[AK09940] %s called, buf=%s", __func__, buf);
	error = kstrtol(buf, 10, &watermark);
	if (error)
		return error;
	ak09940_set_watermark(akm, (u8)watermark);
	return count;
}

static IIO_DEVICE_ATTR(watermark,
		AK09940_IIO_DEVICE_ATTR_PERMISSION,
		attr_watermark_show,
		attr_watermark_store,
		0);

static ssize_t attr_sensor_drive_show(
	struct device		   *dev,
	struct device_attribute *attr,
	char					*buf)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));

	akdbgprt(dev, "[AK09940] %s called", __func__);

	return snprintf(buf, 4, "%02X\n", akm->MTbit);
}
static ssize_t attr_sensor_drive_store(
	struct device		   *dev,
	struct device_attribute *attr,
	const char			  *buf,
	size_t				  count)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
	long mt = 0;
	int error = 0;

	akdbgprt(dev, "[AK09940] %s called, buf=%s", __func__, buf);
	error = kstrtol(buf, 10, &mt);
	if (error)
		return error;
	ak09940_set_sensor_drive(akm, (u8)mt);
	return count;
}

static IIO_DEVICE_ATTR(sensor_drive,
		AK09940_IIO_DEVICE_ATTR_PERMISSION,
		attr_sensor_drive_show,
		attr_sensor_drive_store,
		0);
static ssize_t attr_softreset_store(
	struct device		   *dev,
	struct device_attribute *attr,
	const char			  *buf,
	size_t				  count)
{
	struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
	int error = 0;
	long reset = 0;

	akdbgprt(dev, "[AK09940] %s called, buf=%s", __func__, buf);
	error = kstrtol(buf, 10, &reset);
	if (error)
		return error;
	if (reset == 0)
		return error;
	error = ak09940_software_reset(akm);
	if (error)
		return error;

	return count;
}

static IIO_DEVICE_ATTR(reset,
		AK09940_IIO_DEVICE_ATTR_PERMISSION,
		NULL,
		attr_softreset_store,
		0);

static struct attribute *ak09940_attributes[] = {
	&iio_dev_attr_setting_reg.dev_attr.attr,
	&iio_dev_attr_data_reg.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_operation_mode.dev_attr.attr,
	&iio_dev_attr_watermark.dev_attr.attr,
	&iio_dev_attr_sensor_drive.dev_attr.attr,
	&iio_dev_attr_reset.dev_attr.attr,
	NULL
};
static const struct attribute_group ak09940_attrs_group = {
	.attrs = ak09940_attributes,
};

/*******************************************************************/

static int ak09940_read_axis(
	struct ak09940_data *akm,
	int				 index,
	int				 *val)
{
	int	 address;
	u8	  rdata[3];
	u16	 len;
	u8	  st2 = 0;
	int32_t mag = 0;

	akdbgprt(&akm->client->dev,
		"[AK09940] %s index = %d\n", __func__, (int)index);

	if ((index < 0) || (index > 3)) {
		dev_err(&akm->client->dev,
				"[ak09940] %s index Error index = %d\n",
				__func__, index);
		return -EINVAL;
	}

	len = 3;
	switch (index) {
	case 1:
		address = AK09940_REG_HXL;
		break;

	case 2:
		address = AK09940_REG_HYL;
		break;

	case 3:
		address = AK09940_REG_HZL;
		break;

	case 0:
	default:
		address = AK09940_REG_ST1;
		len = 1;
		break;
	}

	ak09940_i2c_read(akm->client, address, len, rdata);
	ak09940_i2c_read(akm->client, AK09940_REG_ST2, 1, &st2);

	if (index != 0) {
		mag = (int32_t)
			(((uint32_t)rdata[2] << 24) |
			 ((uint32_t)rdata[1] << 16) |
			 ((uint32_t)rdata[0] << 8)) >> 8;

		if (mag == AK09940_DATA_OVERFLOW_VALUE) {
			dev_err(&akm->client->dev,
				"[AK09940] %s mag data overflow!!!!!!!\n",
				__func__);
		}

		*val = mag;
		akdbgprt(&akm->client->dev,
			"[AK09940] %s read mag[%d] data = %d\n",
			__func__, index, *val);
	} else {
		*val = (rdata[len - 1] << 8) | st2;
	}

	return 0;
}

static int ak09940_read_raw(
	struct iio_dev			 *indio_dev,
	struct iio_chan_spec const *chan,
	int						*val,
	int						*val2,
	long					   mask)
{
	struct ak09940_data *akm = iio_priv(indio_dev);
	int				 readValue;
	int				 ret;

	akdbgprt(&akm->client->dev,
		"%s called (index=%d)", __func__, chan->scan_index);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;
		ret = ak09940_read_axis(akm, chan->scan_index, &readValue);
		akdbgprt(&akm->client->dev,
			"[AK09940] %s : scan_index=%d, readValue=%X\n",
			__func__, chan->scan_index, readValue);
		*val = readValue;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ak09940_get_samp_freq(akm, akm->mode);
		akdbgprt(&akm->client->dev,
			"[AK09940] %s : mode=%x, freq=%d\n",
			__func__, akm->mode, *val);
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ak09940_set_measurement_freq(
	struct ak09940_data *akm,
	int				 val)
{
	u8  mode;
	int error = 0;

	akdbgprt(&akm->client->dev,
		"[AK09940] %s val = %d\n", __func__, (int)val);

	mode = ak09940_get_continue_mode_by_interval(akm, val);

	error = ak09940_set_mode(akm, mode);
	if (error < 0) {
		dev_err(&akm->client->dev,
			"[AK09940] %s(%d)  set mode failed\n",
			 __func__, __LINE__);
		return error;
	}

	akdbgprt(&akm->client->dev,
		"[AK09940] %s mode = %d\n",
		__func__, akm->mode);

	return error;
}

static int ak09940_write_raw(
	struct iio_dev			 *indio_dev,
	struct iio_chan_spec const *chan,
	int						val,
	int						val2,
	long					   mask)
{
	struct ak09940_data *akm = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ak09940_set_measurement_freq(akm, val);
	}

	return -EINVAL;
}

static const struct iio_info ak09940_info = {
	.attrs = &ak09940_attrs_group,
	.read_raw = &ak09940_read_raw,
	.write_raw = &ak09940_write_raw,
	.driver_module = THIS_MODULE,
};

static int ak09940_convert_axis(
	s32 *pre_data, s32 *new_data,
	u8 *axis_order, u8 *axis_sign)
{
	int i = 0;

	if (pre_data == NULL) {
		pr_err("[AK09940] %s pre data is NULL!!!\n", __func__);
		return -1;
	}
	if (new_data == NULL) {
		pr_err("[AK09940] %s new_data is NULL!!!\n", __func__);
		return -1;
	}
	if (axis_order == NULL) {
		pr_err("[AK09940] %s axis_order is NULL!!!\n", __func__);
		return -1;
	}
	if (axis_sign == NULL) {
		pr_err("[AK09940] %s axis_sign is NULL!!!\n", __func__);
		return -1;
	}
	for (i = 0; i < NUM_OF_AXIS; i++) {
		new_data[i] = pre_data[axis_order[i]];
		if (axis_sign[i])
			new_data[i] *= -1;
	}
	return 0;
}
static void ak09940_read_and_event(struct iio_dev *indio_dev)
{
	struct ak09940_data *akm = iio_priv(indio_dev);
	struct i2c_client   *client = akm->client;
	u8 rdata[AK09940_REG_ST1_ST2_LENGTH];
	/* data(32bit) * 3-axis + status(16bit) + timestamp(64bit) */
	u8  event[sizeof(s32) * 3 + sizeof(s16) + sizeof(s64)];
	s32 *pevent;
	s32 temp_event[3];
	int j = 0;

	ak09940_i2c_read(client,
		AK09940_REG_ST1,
		AK09940_REG_ST1_ST2_LENGTH,
		rdata);

	memset(event, 0, sizeof(event));

	event[0] = rdata[AK09940_ST1_POS];
	event[1] = rdata[AK09940_ST2_POS];

	pevent = (s32 *)&event[2];
	/*  convert register data to magdata
	 *  register data: 3 * 8bit
	 *  magdata: 3 * 18bit
	 */
	ak09940_parse_raw_data(&rdata[AK09940_DATA_POS],
					temp_event);

	if (ak09940_data_check_overflow(temp_event)) {
		dev_err(&client->dev,
			"[AK09940] %s(%d)  read mag data overflow\n",
			 __func__, __LINE__);
	}
	ak09940_convert_axis(temp_event, pevent,
			akm->axis_order, akm->axis_sign);
#ifdef AK09940_DEBUG
	akdbgprt(&client->dev,
		"[AK09940] %s mag, %04XH,%04XH,%04XH\n",
		__func__, *pevent, *(pevent + 1), *(pevent + 2));
#endif

	/*  convert data to Q10 format
	 *  register data is 18bit
	 *  so we use Q10 format here
	 */
	for (j = 0; j < NUM_OF_AXIS; j++)
		temp_event[j] = RAW_DATA_TO_Q10(temp_event[j]);

#ifdef KERNEL_3_18_XX
	iio_push_to_buffers_with_timestamp(indio_dev, event,
							iio_get_time_ns());
#else
	iio_push_to_buffers_with_timestamp(indio_dev, event,
						iio_get_time_ns(indio_dev));
#endif
}
static void ak09940_fifo_read_and_event(struct iio_dev *indio_dev)
{
	struct ak09940_data *akm = iio_priv(indio_dev);
	struct i2c_client   *client = akm->client;
	s64 now;
	int i = 0;
	int j = 0;
	/* position:
	 * ST1(8bit)+ST2(8bit)+data(32bit) * 3-axis + timestamp(64bit)
	 */
	u8  event[sizeof(s32) * 3 + sizeof(s16) + sizeof(s64)];
	s32 *pevent;
	s32 temp_event[3];
	u64 time_interval = 0;
	s64 cur_time = 0;
	int read_bytes = 0;
	int event_pos = 0;
	/* ST1+FIFO data length*/
	/* FIFO data length: (HXL_ST2_LENGTH) * FIFO MAX STEP*/
	u8 rdata[AK09940_REG_HXL_ST2_LENGTH * AK09940_FIFO_MAX_SIZE + 1] = {0};
	u8 st1 = 0;
	int event_num = 0;

#ifdef KERNEL_3_18_XX
	now = iio_get_time_ns();
#else
	now = iio_get_time_ns(indio_dev);
#endif
	time_interval = now - akm->prev_time_ns;
	do_div(time_interval, akm->watermark);
	/* read ST1 to get how many event in FIFO */
	ak09940_i2c_read(client,
		AK09940_REG_ST1,
		1,
		rdata);
	st1 = rdata[0];
	akdbgprt(&client->dev, "[AK09940] %s, st1=%x\n", __func__, st1);
	event_num = (st1 & AK09940_FNUM_MASK) >> AK09940_FNUN_SHIFT;
	/* total bytes is ST1 + event_num * AK09940_DATA_FIFO_LENGTH*/
	read_bytes = AK09940_REG_HXL_ST2_LENGTH * event_num + 1;
	ak09940_i2c_read(client, AK09940_REG_ST1,
		read_bytes,
		rdata);
	for (i = 0; i < akm->watermark; i++) {
		event_pos = i * AK09940_REG_HXL_ST2_LENGTH + 1;
		memset(event, 0, sizeof(event));

		event[0] = rdata[AK09940_ST1_POS] & AK09940_DRDY_MASK;
		event[1] = rdata[event_pos + AK09940_FIFO_ST2_POS];

		pevent = (s32 *)&event[2];
		/*  convert register data to magdata
		 *  register data: 3 * 8bit
		 *  magdata: 3 * 18bit
		 */
		ak09940_parse_raw_data(
			&rdata[event_pos + AK09940_FIFO_DATA_POS],
			temp_event);
		if (ak09940_data_check_overflow(temp_event)) {
			dev_err(&client->dev,
			"[AK09940] %s(%d)  read mag data overflow!!!!!!!!!\n",
			__func__, __LINE__);
		}
		ak09940_convert_axis(temp_event, pevent,
			akm->axis_order, akm->axis_sign);
#ifdef AK09940_DEBUG
	akdbgprt(&client->dev,
		"[AK09940] %s mag, %04XH,%04XH,%04XH,time,%lld\n",
		__func__, *pevent, *(pevent + 1), *(pevent + 2), cur_time);
#endif
		/*  convert data to Q10 format
		 *  register data is 18bit
		 *  so we use Q10 format here
		 */
		for (j = 0; j < 3; j++)
			temp_event[j] = RAW_DATA_TO_Q10(temp_event[j]);

		cur_time = akm->prev_time_ns + time_interval * (i + 1);
		iio_push_to_buffers_with_timestamp(indio_dev, event, cur_time);
	}
	akm->prev_time_ns = now;
}
/*******************************************************************/
static void ak09940_send_event(struct iio_dev *indio_dev)
{
	struct ak09940_data *akm = iio_priv(indio_dev);

	if (akm->watermark_en == 0) {
		ak09940_read_and_event(indio_dev);
	} else {
		mutex_lock(&akm->fifo_mutex);
		ak09940_fifo_read_and_event(indio_dev);
		mutex_unlock(&akm->fifo_mutex);
	}
}

/*******************************************************************/
static void ak09940_flush_handler(struct work_struct *work)
{
	struct ak09940_data *akm =
		container_of(work, struct ak09940_data, flush_work);
	struct iio_dev *indio_dev = iio_priv_to_dev(akm);

	ak09940_send_event(indio_dev);
}

/******************************************************************/
static irqreturn_t ak09940_handle_trigger(
	int  irq,
	void *p)
{
	const struct iio_poll_func *pf = p;
	struct iio_dev			 *indio_dev = pf->indio_dev;
	struct ak09940_data *akm = iio_priv(indio_dev);

	akdbgprt(&akm->client->dev,
		"[AK09940] %s(%d), time:%lld\n",
		__func__, __LINE__, iio_get_time_ns(indio_dev));

	queue_work(akm->wq, &akm->flush_work);

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/******************************************************************/
static int ak09940_setup(struct i2c_client *client)
{
	struct iio_dev	  *indio_dev = i2c_get_clientdata(client);
	struct ak09940_data *akm = iio_priv(indio_dev);
	s32				 err;

	akdbgprt(&akm->client->dev,
		"[AK09940] %s(%d)\n", __func__, __LINE__);

	if (akm->rstn_gpio > 0) {
		gpio_set_value(akm->rstn_gpio, 0);
		mdelay(1);
		gpio_set_value(akm->rstn_gpio, 1);
		mdelay(1);
	}

	err = ak09940_read_device_and_check(akm);
	if (err < 0)
		return err;

	akm->test_lolim[0] = AK09940_TEST_LOLIM_X;
	akm->test_lolim[1] = AK09940_TEST_LOLIM_Y;
	akm->test_lolim[2] = AK09940_TEST_LOLIM_Z;

	akm->test_hilim[0] = AK09940_TEST_HILIM_X;
	akm->test_hilim[1] = AK09940_TEST_HILIM_Y;
	akm->test_hilim[2] = AK09940_TEST_HILIM_Z;

	akm->freqmodeTable = measurementFreqModeTable;
	akm->freq_num = ARRAY_SIZE(measurementFreqModeTable);

	akm->mode = AK09940_MODE_PDN;
	akm->watermark = 0;
	akm->watermark_en = 0;
	akm->TEMPbit = 1;
	akm->MTbit = 0;
	/* reset timestamp */
	akm->prev_time_ns = 0;

	return 0;
}

static void ak09940_set_default_axis(
	struct ak09940_data *akm)
{
	int i;

	for (i = 0; i < 3; i++) {
		akm->axis_order[i] = i;
		akm->axis_sign[i] = 0;
	}
}
static void ak09940_init_axis(struct ak09940_data *akm)
{
	struct device	  *dev;
	struct device_node *np;

	dev = &(akm->client->dev);
	np = dev->of_node;

	if (np) {
		/* get from device node */
		/* parameters are declared as 'unsigned char' */
		/* if parameter cannot be get, use default value */
		if (of_property_read_u8(np, "axis_order_x",
					&akm->axis_order[0]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(np, "axis_order_y",
					&akm->axis_order[1]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(np, "axis_order_z",
					&akm->axis_order[2]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(np, "axis_sign_x",
					&akm->axis_sign[0]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(np, "axis_sign_y",
					&akm->axis_sign[1]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(np, "axis_sign_z",
					&akm->axis_sign[2]) != 0)
			goto SET_DEFAULT_AXIS;
	} else {
		ak09940_set_default_axis(akm);
	}
	akdbgprt(dev, "%s : axis=[%d,%d,%d] sign=[%d,%d,%d]", __func__,
		akm->axis_order[0], akm->axis_order[1], akm->axis_order[2],
		akm->axis_sign[0], akm->axis_sign[1], akm->axis_sign[2]);
	return;
SET_DEFAULT_AXIS:
	dev_dbg(dev, "%s : set default axis", __func__);
	/* set default axis value */
	ak09940_set_default_axis(akm);
}

static int ak09940_parse_dt(struct ak09940_data *ak09940)
{
	struct device	  *dev;
	struct device_node *np;
	int				ret;

	dev = &(ak09940->client->dev);
	np = dev->of_node;

	if (!np) {
		dev_err(dev, "[AK09940] %s : device tree is null\n", __func__);
		/* device tree is null */
		/* set default axis value */
		ak09940_set_default_axis(ak09940);
		return -EINVAL;
	}
	ak09940_init_axis(ak09940);

	/* init RST pin */
	ak09940->rstn_gpio = of_get_named_gpio(np, "ak09940,rst_gpio", 0);
	if (ak09940->rstn_gpio < 0) {
		dev_err(dev, "[AK09940] %s : rstn gpio is null\n", __func__);
		ak09940->rstn_gpio = -1;
	} else {
		ret = gpio_request(ak09940->rstn_gpio, "ak09940 rstn");
		if (ret < 0) {
			akdbgprt(dev,
				"[AK09940] %s : gpio_request ret = %d\n",
				__func__, ret);
			ak09940->rstn_gpio = -1;
			return ret;
		}
		gpio_direction_output(ak09940->rstn_gpio, 0);
	}

	/* init INT pin */
	ak09940->int_gpio = of_get_named_gpio(np, "ak09940,int_gpio", 0);
	if (ak09940->int_gpio < 0) {
		dev_err(dev, "[AK09940] %s : rstn gpio is null\n", __func__);
		ak09940->int_gpio = -1;
	} else {
		ret = devm_gpio_request_one(dev,
			ak09940->int_gpio,
			GPIOF_IN,
			"ak09940_int");
		if (ret < 0) {
			akdbgprt(dev,
				"[AK09940] %s : gpio_request ret = %d\n",
				__func__, ret);
			ak09940->int_gpio = -1;
			return ret;
		}
		ak09940->irq = gpio_to_irq(ak09940->int_gpio);
	}

	return 0;
}

static int ak09940_set_trigger_state(
	struct iio_trigger *trig,
	bool			   state)
{
	const struct iio_dev *indio_dev = dev_get_drvdata(
							trig->dev.parent);
	const struct ak09940_data *akm = iio_priv(indio_dev);

	akdbgprt(&akm->client->dev,
		"%s called. st=%s",
		__func__, (state ? "true" : "false"));
	return 0;
}

static const struct iio_trigger_ops ak09940_trigger_ops = {
	.set_trigger_state = ak09940_set_trigger_state,
	.owner = THIS_MODULE,
};


static int ak09940_probe(
	struct i2c_client		  *client,
	const struct i2c_device_id *id)
{
	struct ak09940_data *akm;
	struct iio_dev	  *indio_dev;
	struct device	   *dev = &client->dev;
	int				 err;
	const char		  *name = NULL;

	akdbgprt(dev, "[AK09940] %s(%d)\n", __func__, __LINE__);


	/* Register with IIO */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*akm));

	if (indio_dev == NULL)
		return -ENOMEM;

	akm = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);

	akm->client = client;

	err = ak09940_parse_dt(akm);
	if (err < 0) {
		if ((akm->rstn_gpio == -1) ||
			(akm->int_gpio == -1))
			goto err_gpio_request;
		else
			dev_err(&client->dev,
			"[AK09940] Device Tree Setting was not found!\n");
	}

	if (id)
		name = id->name;
	err = ak09940_setup(client);

	if (err < 0) {
		dev_err(&client->dev, "%s initialization fails\n", name);
		goto err_setup;
	}

	if (akm->irq) {
		akm->trig = iio_trigger_alloc(
			"%s-dev%d",
			name, indio_dev->id);
		if (!akm->trig) {
			err = -ENOMEM;
			goto err_trigger_alloc;
		}
		err = devm_request_irq(&client->dev,
			akm->irq,
			iio_trigger_generic_data_rdy_poll,
			IRQF_TRIGGER_RISING,
			dev_name(&client->dev),
			akm->trig);

		if (err) {
			dev_err(&client->dev,
				"[AK09940] devm_request_irq Error! err=%d\n",
				err);
			goto err_request_irq;
		}

		akm->trig->dev.parent = dev;
		akm->trig->ops = &ak09940_trigger_ops;
		iio_trigger_set_drvdata(akm->trig, indio_dev);
		/* indio_dev->trig = iio_trigger_get(akm->trig); */
		err = iio_trigger_register(akm->trig);

		if (err) {
			dev_err(&client->dev,
				"[AK09940] iio_trigger_register Error! err=%d\n",
				err);
			goto err_trigger_register;
		} else {
			indio_dev->trig = akm->trig;
		}
	}



	mutex_init(&akm->fifo_mutex);

	INIT_WORK(&akm->flush_work, ak09940_flush_handler);
	akm->wq = create_singlethread_workqueue("ak09940");
	if (akm->wq == NULL) {
		dev_err(dev, "%s: create single thread wq failed.", __func__);
		goto err_create_thread_wq;
	}

	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = ak09940_channels;
	indio_dev->num_channels = ARRAY_SIZE(ak09940_channels);
	indio_dev->info = &ak09940_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = name;

	err = iio_triggered_buffer_setup(indio_dev,
						NULL,
						ak09940_handle_trigger,
						NULL);

	if (err) {
		dev_err(&client->dev, "triggered buffer setup failed\n");
		goto err_iio_buffer_setup;
	}

	akdbgprt(dev, "[AK09940] %s(iio_triggered_buffer_setup=%d)\n",
		__func__, err);

	err = iio_device_register(indio_dev);

	if (err) {
		dev_err(&client->dev, "device register failed\n");
		goto err_iio_device_register;
	}

	akdbgprt(dev, "[AK09940] %s(iio_device_register=%d)\n",
		__func__, err);
	return err;
err_iio_device_register:
	iio_triggered_buffer_cleanup(indio_dev);

err_iio_buffer_setup:
	if (akm->wq)
		destroy_workqueue(akm->wq);
err_create_thread_wq:
	if (akm->irq)
		iio_trigger_unregister(akm->trig);
err_trigger_register:
	if (akm->irq)
		devm_free_irq(dev, akm->irq, akm);
err_request_irq:
	if (akm->irq)
		iio_trigger_free(akm->trig);

err_trigger_alloc:
err_setup:
err_gpio_request:
	iio_device_free(indio_dev);

	return err;
}

static int ak09940_remove(struct i2c_client *client)
{
	struct iio_dev	  *indio_dev = i2c_get_clientdata(client);
	struct ak09940_data *akm = iio_priv(indio_dev);

	iio_trigger_free(akm->trig);
	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);

	ak09940_set_PDN_mode(akm);

	if (akm->rstn_gpio > 0) {
		gpio_set_value(akm->rstn_gpio, 0);
		msleep(20);
		gpio_free(akm->rstn_gpio);
	}
	return 0;
}

static int ak09940_i2c_suspend(struct device *dev)
{
	struct i2c_client   *i2c = to_i2c_client(dev);
	struct iio_dev	  *indio_dev = i2c_get_clientdata(i2c);
	struct ak09940_data *akm = iio_priv(indio_dev);

	/* Reset device and all configure */
	ak09940_software_reset(akm);
	if (akm->rstn_gpio > 0)
		gpio_set_value(akm->rstn_gpio, 0);

	return 0;
}

static int ak09940_i2c_resume(struct device *dev)
{
	struct i2c_client   *i2c = to_i2c_client(dev);
	struct iio_dev	  *indio_dev = i2c_get_clientdata(i2c);
	struct ak09940_data *akm = iio_priv(indio_dev);

	if (akm->rstn_gpio > 0) {
		gpio_set_value(akm->rstn_gpio, 0);
		mdelay(10);
		gpio_set_value(akm->rstn_gpio, 1);
		mdelay(1);
	}
	/* Reset device, system will set mode later*/
	ak09940_software_reset(akm);
	return 0;
}

static const struct dev_pm_ops ak09940_i2c_pops = {
	.suspend = ak09940_i2c_suspend,
	.resume = ak09940_i2c_resume,
};

static const struct i2c_device_id ak09940_id[] = {
	{ "ak09940", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ak09940_id);

static const struct of_device_id ak09940_of_match[] = {
	{ .compatible = "asahi-kasei,ak09940"},
	{},
};
MODULE_DEVICE_TABLE(of, ak09940_of_match);

static struct i2c_driver ak09940_driver = {
	.driver = {
		.name = "ak09940",
		.pm = &ak09940_i2c_pops,
		.of_match_table = of_match_ptr(ak09940_of_match),
		.owner = THIS_MODULE,
	},
	.probe = ak09940_probe,
	.remove = ak09940_remove,
	.id_table = ak09940_id,
};
module_i2c_driver(ak09940_driver);

MODULE_AUTHOR("Junichi Wakasugi <wakasugi.jb@om.asahi-kasei.co.jp>");
MODULE_DESCRIPTION("AK09940 magnetometer driver");
MODULE_LICENSE("GPL v2");
