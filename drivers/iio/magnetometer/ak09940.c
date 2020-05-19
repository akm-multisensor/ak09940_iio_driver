/* drivers/staging/iio/magnetometer/ak09940.c
 *
 * ak09940.c -- A sensor driver for the 3D magnetometer AK09940.
 *
 * Copyright (C) 2019 Asahi Kasei Microdevices Corporation
 *                       Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      19/03/27	    0.1
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

//#define KERNEL_3_18_XX

#define AK09940_DEBUG

#ifdef AK09940_DEBUG
#define akdbgprt  printk
#else
#define akdbgprt(format, arg ...)  do {} while (0)
#endif


/*
 * Register definitions, as well as various shifts and masks to get at the
 * individual fields of the registers.
 */


// REGISTER MAP
#define AK09940_REG_WIA    0x00
#define AK09940_DEVICE_ID  0x48A1


#define AK09940_REG_ST1  0x10
#define AK09940_REG_HXL  0x11
#define AK09940_REG_HYL  0x14
#define AK09940_REG_HZL  0x17


#define AK09940_REG_TMPS               0x1A
#define AK09940_REG_ST2                0x1B

#define AK09940_REG_CNTL1              0x30
#define AK09940_REG_CNTL2              0x31
#define AK09940_REG_CNTL3              0x32
#define AK09940_REG_SRST               0x33
#define AK09940_REG_I2CDIS             0x36

#define AK09940_MAX_REGS               AK09940_REG_I2CDIS

#define AK09940_MEASUREMENT_WAIT_TIME  2

#define AK09940_MAG_DATA_LENTH         9
#define AK09940_READ_DATA_LENTH        12

#define AK09940_MODE_PDN               0x00
#define AK09940_MODE_SNG               0x01
#define AK09940_MODE_CONT_10HZ         0X02
#define AK09940_MODE_CONT_20HZ         0X04
#define AK09940_MODE_CONT_50HZ         0X06
#define AK09940_MODE_CONT_100HZ        0X08
#define AK09940_MODE_CONT_200HZ        0X0A
#define AK09940_MODE_CONT_400HZ        0X0C
#define AK09940_MODE_SELFTEST          0x10
#define AK09940_MODE_NUM               9


/* AK09915 selftest threshold */
#define AK09940_TEST_LOLIM_X  (-1200)
#define AK09940_TEST_HILIM_X  (-300)
#define AK09940_TEST_LOLIM_Y  (300)
#define AK09940_TEST_HILIM_Y  (1200)
#define AK09940_TEST_LOLIM_Z  (-1600)
#define AK09940_TEST_HILIM_Z  (-400)

#define AK09940_WM(wm)     ((wm - 1) & 0x07)
#define AK09940_WM_EN(en)  ((en) << 7)
#define AK09940_SDR(sdr)   ((sdr) << 5)

#define AK09940_WM_EN_MASK               0x80
#define AK09940_WM_EN_SHIFT              7
#define AK09940_SDR_MASK                 0x60
#define AK09940_SDR_SHIFT                5
#define AK09940_MODE_MASK                0x1F
#define AK09940_MODE_SHIFT               0

#define AK09940_DATA_OVERFLOW_VALUE      0x1FFFF

#define AK09940_PDN_TO_OTHER_MODE_DELAY  1

enum {
    AK09940_ST1_POS = 0,
    AK09940_DATA_POS = 1,
    AK09940_TMPS_POS = 10,
    AK09940_ST2_POS = 11
} AK09940_REG_POS_ENUM;


static int measurementWaitTime[4] =
{
    1, 1, 2, 4
};

// 0 : Power Down
// 1 : Measurement Mode 1
// 2 : Measurement Mode 2
// 3 : Measurement Mode 3
// 4 : Measurement Mode 4
// 5 : Measurement Mode 5
// 6 : Measurement Mode 6


static int measurementFreqTable[] = {
    0,
    10,
    20,
    50,
    100,
    200,
    400
};
static int measurementModeRegTable[] = {
    AK09940_MODE_PDN,
    AK09940_MODE_CONT_10HZ,
    AK09940_MODE_CONT_20HZ,
    AK09940_MODE_CONT_50HZ,
    AK09940_MODE_CONT_100HZ,
    AK09940_MODE_CONT_200HZ,
    AK09940_MODE_CONT_400HZ,
};


/*
 * Per-instance context data for the device.
 */
struct ak09940_data {
    struct i2c_client  *client;
    struct iio_trigger *trig;
    struct mutex       lock;
    int                int_gpio;
    int                irq;
    int                rstn_gpio;

    u8                 mode;
    s16                numMode;

    u8                 SDRbit;
    u8                 selftest;

    u8                 watermark;
    u8                 watermark_en;
    u8                 TEMPbit;

    s32                test_lolim[3];
    s32                test_hilim[3];
    int                *freq_table;
    int                *modeReg_table;
    u8                 freq_num;
};

// ****************** Register R/W  ************************************
static int ak09940_i2c_reads(
    struct i2c_client *client,
    u8                *reg,
    int               reglen,
    u8                *rdata,
    int               datalen)
{
    struct i2c_msg xfer[2];
    int            ret;

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

    if (ret == 2) {
        return 0;
    } else if (ret < 0) {
        return ret;
    } else {
        return -EIO;
    }
}

static int ak09940_i2c_read(
    struct i2c_client *client,
    u8                address,
    int               rLen,
    u8                *rdata)
{
    u8  tx[1];
    int i, ret;

    akdbgprt("[AK09940] %s address=%x, lenth=%d\n", __func__, (int)address,
             rLen);

    tx[0] = address;

    ret = ak09940_i2c_reads(client, tx, 1, rdata, rLen);

    if (ret < 0) {
        dev_err(&client->dev, "[AK09940] I2C Read Error\n");

        for (i = 0; i < rLen; i++) {
            rdata[i] = 0;
        }
    }

    return (ret);
}

static int ak09940_i2c_write(
    struct i2c_client *client,
    u8                address,
    u8                value)
{
    int ret;

    akdbgprt("[AK09940] %s (0x%02X, 0x%02X)\n", __func__, (int)address,
             (int)value);

    ret = i2c_smbus_write_byte_data(client, address, value);

    if (ret < 0) {
        pr_err("%s: comm error, ret= %d\n", __func__, ret);
        //dbg_show(__func__);
    }

    pr_debug("%s return %d\n", __func__, ret);

    return ret;
}

//********************************************************************************

#define AK099XX_STATUS_CHANNEL(index)     { \
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

/*****************************************************************************/
static int ak09940_check_measurement_mode(
    struct ak09940_data *akm,
    u8                  mode)
{
    u8 i = 0;

    akdbgprt("[AK09940] %s called, mode=%d", __func__, mode);

    for (i = 0; i < akm->freq_num; i++) {
        if (akm->modeReg_table[i] == mode) {
            return 0;
        }
    }

    return -EINVAL;
}

static int ak09940_set_continue_mode_measure(
    struct ak09940_data *akm,
    u8                  mode)
{
    int error = 0;
    u8  cntl1_value = 0, cntl2_value = 0, cntl3_value = 0;
    akdbgprt("[AK09940] %s called", __func__);

    error = ak09940_check_measurement_mode(akm, mode);

    if (error) {
        return error;
    }

    //set watermark
    if (akm->watermark > 0) {
        cntl1_value = AK09940_WM(akm->watermark);
        error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL1, cntl1_value);
    }

    //set TEMP enable
    if (akm->TEMPbit) {
        cntl2_value = 0x40;
        error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL2, cntl2_value);
    }

    if ((akm->mode != AK09940_MODE_PDN) && (mode != AK09940_MODE_PDN)) {
        //now mode is not PDN mode, set PDN mode first
        cntl3_value = AK09940_WM_EN(akm->watermark_en) +
            AK09940_SDR(akm->SDRbit) + AK09940_MODE_PDN;
        error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3, cntl3_value);

        if (error) {
            akm->mode = AK09940_MODE_PDN;
            return error;
        }

        msleep(AK09940_PDN_TO_OTHER_MODE_DELAY);
    }

    akm->mode = mode;
    cntl3_value = AK09940_WM_EN(akm->watermark_en) + AK09940_SDR(akm->SDRbit) +
        akm->mode;
    error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3, cntl3_value);

    if (error) {
        akm->mode = AK09940_MODE_PDN;
        return error;
    }

    return error;
}

static int ak09940_get_continue_mode_by_interval(
    struct ak09940_data *akm,
    int                 interval)
{
    int n = 0, mode = 0;
    int freq = 0;


    akdbgprt("[AK09940] %s inverval = %d\n", __FUNCTION__, interval);

    if (interval < 0) {
        dev_err(&akm->client->dev, "[AK09940] %s Val Error val = %d\n",
                __func__, interval);
        return -EINVAL;
    }

    if (interval == 0) {
        mode = AK09940_MODE_PDN;
        return mode;
    }

    freq = 1000 / interval;

    while ((freq > measurementFreqTable[n]) && ( n < ( akm->numMode - 1))) {
        n++;
    }

    mode = measurementModeRegTable[n];
    akdbgprt("[AK09940] %s mode = %d\n", __FUNCTION__, mode);

    return mode;
}

static int ak09940_parse_raw_data(
    u8  *reg,
    s32 *mag)
{
    int i = 0;

    if (NULL == reg) {
        return -1;
    }

    if (NULL == mag) {
        return -1;
    }

    /* Store data (uT) */
    for (i = 0; i < 3; i++) {
        /* convert to int32 data */
        *(mag + i) = (int32_t)
            (((uint32_t)reg[i * 3 + 2] << 24) |
             ((uint32_t)reg[i * 3 + 1] << 16) |
             ((uint32_t)reg[i * 3] << 8)) >> 8;
    }

    return 0;
}

static int ak09940_one_axis_data_check_overflow(s32 data)
{
    if (data == AK09940_DATA_OVERFLOW_VALUE) {
        return 1;
    } else {
        return 0;
    }
}

static int ak09940_data_check_overflow(s32 *mag)
{
    int i = 0;
    int ret = 0;

    for (i = 0; i < 3; i++) {
        if (NULL == mag) {
            return -EINVAL;
        }

        ret = ak09940_one_axis_data_check_overflow(*(mag + i));

        if (ret) {
            return 1;
        }
    }

    return 0;
}

static ssize_t attr_setting_reg_show(
    struct device           *dev,
    struct device_attribute *attr,
    char                    *buf)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
    u8                  cntl[3];
    akdbgprt("[AK09940] %s called", __func__);

    ak09940_i2c_read(akm->client, AK09940_REG_CNTL1, 3, cntl);
    dev_info(dev, "[AK09940] CNTL1(30H)=%XH", (int)cntl[0]);
    dev_info(dev, "[AK09940] CNTL1(31H)=%XH", (int)cntl[1]);
    dev_info(dev, "[AK09940] CNTL1(32H)=%XH", (int)cntl[2]);

    return snprintf(buf, 9, "%02X,%02x,%02x\n", cntl[0], cntl[1], cntl[2]);
}

static ssize_t attr_setting_reg_store(
    struct device           *dev,
    struct device_attribute *attr,
    const char              *buf,
    size_t                  count)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
    char                *ptr_data = (char *)buf;
    char                *p;
    int                 pt_count = 0;
    u8                  val[2];
    int                 ret = 0;
    u8                  address = 0, value = 0;
    u8                  i = 0;
    u8                  new_mode = 0;
    akdbgprt("[AK09940] %s called: '%s'(%zu)", __func__, buf, count);

    if (buf == NULL) {
        return -EINVAL;
    }

    if (count == 0) {
        return 0;
    }

    while ((p = strsep(&ptr_data, ","))) {
        if (!*p) {
            break;
        }

        if (pt_count >= 3) {
            break;
        }

        if (( pt_count == 0 ) ||
            (( val[0] >= AK09940_REG_CNTL1 ) &&
             (val[0] <= AK09940_REG_CNTL3))) {
            val[pt_count] = simple_strtol(p, NULL, 10);
        } else {
            sscanf(p, "%x", &value);
            val[pt_count] = (unsigned long)value;
        }

        pt_count++;
    }

    if (pt_count != 2) {
        dev_err(dev, "[AK09940] %s pt_count = %d, Error", __func__, pt_count);
        return -EINVAL;
    } else {
        switch (val[0]) {
            case 0x30:
                akm->watermark = val[1];
                break;

            case 0x31:
                akm->TEMPbit = val[1];
                break;

            case 0x32:
                akm->watermark_en = (val[1] & AK09940_WM_EN_MASK) >>
                    AK09940_WM_EN_SHIFT;
                akm->SDRbit = (val[1] & AK09940_SDR_MASK) >> AK09940_SDR_SHIFT;
                new_mode = (val[1] & AK09940_MODE_MASK) >> AK09940_MODE_SHIFT;

                while (new_mode != measurementModeRegTable[i]) {
                    i++;
                    continue;
                }

                akm->mode = i;
                break;
        }
    }

    address = val[0];
    value = val[1];
    ret = ak09940_i2c_write(akm->client, address, value);

    return count;
}

static IIO_DEVICE_ATTR(setting_reg,
                       0664, /* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
                       attr_setting_reg_show,
                       attr_setting_reg_store,
                       0);

static ssize_t attr_data_reg_show(
    struct device           *dev,
    struct device_attribute *attr,
    char                    *buf)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
//         u8  cntl[3];
    u8  result[AK09940_READ_DATA_LENTH];
    s32 mag[3];
    int ret;
#ifdef AK09940_DEBUG
    int i = 0;
#endif

    akdbgprt("[AK09940] %s called", __func__);

    ret = ak09940_i2c_read(akm->client, AK09940_REG_ST1,
                           AK09940_READ_DATA_LENTH, result);

#ifdef AK09940_DEBUG

    for (i = 0; i < AK09940_READ_DATA_LENTH; i++) {
        akdbgprt("[AK09940] %s %d %02XH\n", __func__, i, result[i]);
    }
#endif

    if (ret < 0) {
        return ret;
    }

    ak09940_parse_raw_data(&result[AK09940_DATA_POS], mag);

    if (ak09940_data_check_overflow(mag)) {
        akdbgprt("[AK09940] %s read mag data overflow!!!!!!!!!", __func__);
    }

    result[0] &= 0x3FF;
    return snprintf(buf, 15, "%04X,%04x,%04x\n", mag[0], mag[1], mag[2]);
}

static IIO_DEVICE_ATTR(data_reg,
                       0664, /* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
                       attr_data_reg_show,
                       NULL,
                       0);

static int ak09940_set_selftest_mode_measure(struct ak09940_data *akm)
{
    int error = 0;
    u8  cntl1_value = 0, cntl2_value = 0, cntl3_value = 0;
    akdbgprt("[AK09940] %s called", __func__);

    if (akm->mode != AK09940_MODE_PDN) {
        //now mode is not PDN mode, set PDN mode first
        akm->mode = AK09940_MODE_PDN;
        cntl3_value = AK09940_WM_EN(akm->watermark_en) +
            AK09940_SDR(akm->SDRbit) + akm->mode;
        error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3, cntl3_value);

        if (error) {
            akm->mode = AK09940_MODE_PDN;
            return error;
        }

        msleep(AK09940_PDN_TO_OTHER_MODE_DELAY);
    }

    cntl3_value = AK09940_WM_EN(akm->watermark_en) + AK09940_SDR(akm->SDRbit) +
        AK09940_MODE_SELFTEST;
    error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3, cntl3_value);

    if (error) {
        akm->mode = AK09940_MODE_PDN;
        return error;
    }

    return error;
}

static void selftest_judgement(
    struct ak09940_data *akm,
    const s32           data[])
{
    int i;
    int result = 0;
    akdbgprt("AK09940] %s , mag, %d,%d,%d\n", __func__, data[0], data[1],
             data[2]);

    for (i = 0; i < 3; i++) {
        if ((data[i] < akm->test_lolim[i]) ||
            (akm->test_hilim[i] < data[i])) {
            akdbgprt(
                "[AK09940] %s , axis %d failed,  data,%d, low_limit,%d,hi_limit,%d\n", __func__, i,
                data[i], akm->test_lolim[i], akm->test_hilim[i]);
            result |= (1 << i);
        }
    }

    akm->selftest = result;
}

static ssize_t attr_selftest_show(
    struct device           *dev,
    struct device_attribute *attr,
    char                    *buf)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
//         u8  cntl[3];
    u8  result[AK09940_READ_DATA_LENTH];
    s32 mag[3];
    int ret;

    akdbgprt("[AK09940] %s called", __func__);

    ret = ak09940_set_selftest_mode_measure(akm);

    if (ret) {
        akdbgprt("[AK09940] %s device set selftest mode fail", __func__);
        return 0;
    }

    msleep(10);
    ret = ak09940_i2c_read(akm->client, AK09940_REG_ST1,
                           AK09940_READ_DATA_LENTH, result);

    if (ret < 0) {
        return ret;
    }

    ak09940_parse_raw_data(&result[AK09940_DATA_POS], mag);
    akdbgprt("[AK09940] mag[X,Y,Z]=[%d,%d,%d]\n", (s32)mag[0], (s32)mag[1],
             (s32)mag[2]);

    selftest_judgement(akm, mag);
    return snprintf(buf, 5, "pass\n");
}

static IIO_DEVICE_ATTR(selftest,
                       0664, /* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
                       attr_selftest_show,
                       NULL,
                       0);

static int ak09940_set_single_mode_measure(struct ak09940_data *akm)
{
    int error = 0;
    u8  cntl1_value = 0, cntl2_value = 0, cntl3_value = 0;
    akdbgprt("[AK09940] %s called", __func__);

    if (akm->mode != AK09940_MODE_PDN) {
        //now mode is not PDN mode, set PDN mode first
        akm->mode = AK09940_MODE_PDN;
        cntl3_value = AK09940_WM_EN(akm->watermark_en) +
            AK09940_SDR(akm->SDRbit) + akm->mode;
        error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3, cntl3_value);

        if (error) {
            akm->mode = AK09940_MODE_PDN;
            return error;
        }

        msleep(AK09940_PDN_TO_OTHER_MODE_DELAY);
    }

    cntl3_value = AK09940_WM_EN(akm->watermark_en) + AK09940_SDR(akm->SDRbit) +
        AK09940_MODE_SNG;
    error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3, cntl3_value);

    if (error) {
        akm->mode = AK09940_MODE_PDN;
        return error;
    }

    return error;
}

static ssize_t attr_single_store(
    struct device           *dev,
    struct device_attribute *attr,
    const char              *buf,
    size_t                  count)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
//         u8  cntl[3];
    u8  result[AK09940_READ_DATA_LENTH];
    s32 mag[3];
    int ret;
#ifdef AK09940_DEBUG
    int i = 0;
#endif

    akdbgprt("[AK09940] %s called", __func__);


    ak09940_set_single_mode_measure(akm);

    return count;
}

static IIO_DEVICE_ATTR(single,
                       0664, /* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
                       NULL,
                       attr_single_store,
                       0);

static ssize_t attr_continuous_store(
    struct device           *dev,
    struct device_attribute *attr,
    const char              *buf,
    size_t                  count)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
    u8                  cntl3;
    int                 ret;
    int                 interval = 0, mode = 0;
#ifdef AK09940_DEBUG
    int i = 0;
#endif

    akdbgprt("[AK09940] %s called, buf=%s", __func__, buf);
    interval = simple_strtol(buf, NULL, 10);

    mode = ak09940_get_continue_mode_by_interval(akm, interval);

    ak09940_set_continue_mode_measure(akm, mode);

    return count;
}

static IIO_DEVICE_ATTR(continuous,
                       0664, /* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
                       NULL,
                       attr_continuous_store,
                       0);

static ssize_t attr_watermark_store(
    struct device           *dev,
    struct device_attribute *attr,
    const char              *buf,
    size_t                  count)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
    u8                  cntl3;
    int                 ret;
    int                 interval = 0, watermark = 0, cntl3_value = 0, error = 0;
#ifdef AK09940_DEBUG
    int i = 0;
#endif

    akdbgprt("[AK09940] %s called, buf=%s", __func__, buf);
    watermark = simple_strtol(buf, NULL, 10);

    if (akm->watermark == watermark) {
        return count;
    } else {
        akm->watermark = watermark;
        error =
            ak09940_i2c_write(akm->client, AK09940_REG_CNTL1, akm->watermark);

        if (error) {
            dev_err(&akm->client->dev,
                    "[AK09940] %s attr_watermark_store set cntl1 faild\n",
                    __func__);
            return error;
        }

        akdbgprt("[AK09940] %s called, watermark=%d, watermark_en = %d",
                 __func__, akm->watermark, akm->watermark_en);

        if (akm->watermark == 0) {
            if (akm->watermark_en != 0) {
                akm->watermark_en = 0;
                cntl3_value = AK09940_WM_EN(akm->watermark_en) + AK09940_SDR(
                        akm->SDRbit) + akm->mode;
                error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3,
                                          cntl3_value);

                if (error) {
                    akm->mode = AK09940_MODE_PDN;
                    return error;
                }
            }
        } else {
            if (akm->watermark_en == 0) {
                akm->watermark_en = 1;
                cntl3_value = AK09940_WM_EN(akm->watermark_en) + AK09940_SDR(
                        akm->SDRbit) + akm->mode;
                error = ak09940_i2c_write(akm->client, AK09940_REG_CNTL3,
                                          cntl3_value);

                if (error) {
                    akm->mode = AK09940_MODE_PDN;
                    return error;
                }
            }
        }
    }


    return count;
}

static IIO_DEVICE_ATTR(watermark,
                       0664, /* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
                       NULL,
                       attr_watermark_store,
                       0);
static ssize_t attr_softreset_store(
    struct device           *dev,
    struct device_attribute *attr,
    const char              *buf,
    size_t                  count)
{
    struct ak09940_data *akm = iio_priv(dev_to_iio_dev(dev));
    int                 ret = 0;
    u8                  address = 0, value = 0;
    akdbgprt("[AK09940] %s called: '%s'(%zu)", __func__, buf, count);

    address = AK09940_REG_SRST;
    value = 0x01;
    ret = ak09940_i2c_write(akm->client, address, value);

    return count;
}

static IIO_DEVICE_ATTR(reset,
                       0664, /* S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP */
                       NULL,
                       attr_softreset_store,
                       0);

static struct attribute             *ak09940_attributes[] = {
    &iio_dev_attr_setting_reg.dev_attr.attr,
    &iio_dev_attr_data_reg.dev_attr.attr,
    &iio_dev_attr_selftest.dev_attr.attr,
    &iio_dev_attr_single.dev_attr.attr,
    &iio_dev_attr_continuous.dev_attr.attr,
    &iio_dev_attr_watermark.dev_attr.attr,
    &iio_dev_attr_reset.dev_attr.attr,
    NULL
};
static const struct attribute_group ak09940_attrs_group = {
    .attrs = ak09940_attributes,
};

/*****************************************************************************/

static int ak09940_read_axis(
    struct ak09940_data *akm,
    int                 index,
    int                 *val)
{
    int     address;
    u8      rdata[3];
    u16     len;
    u8      st2 = 0;
    int32_t mag = 0;

    akdbgprt("[AK09940] %s index = %d\n", __FUNCTION__, (int)index);

    if ((index < 0) || (index > 3)) {
        dev_err(&akm->client->dev, "[ak09940] %s index Error index = %d\n",
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

        if (ak09940_one_axis_data_check_overflow(mag)) {
            akdbgprt("[AK09940] %s mag data overflow!!!!!!!, mag = %x,%x,%x\n",
                     __FUNCTION__, rdata[2], rdata[1], rdata[0]);
        }

        *val = mag;
        akdbgprt("[AK09940] %s read mag[%d] data = %d\n", __FUNCTION__, index,
                 *val);
    } else {
        *val = (rdata[len - 1] << 8) | st2;
    }

    return 0;
}

static int ak09940_read_raw(
    struct iio_dev             *indio_dev,
    struct iio_chan_spec const *chan,
    int                        *val,
    int                        *val2,
    long                       mask)
{
    struct ak09940_data *akm = iio_priv(indio_dev);
    int                 readValue;
    int                 ret;
    u8                  cntlValue;

    akdbgprt("%s called (index=%d)", __func__, chan->scan_index);

    switch (mask) {
        case IIO_CHAN_INFO_RAW:
            ret = ak09940_read_axis(akm, chan->scan_index, &readValue);
            pr_info("[AK09940] %s : scan_index=%d, readValue=%X\n", __func__,
                    chan->scan_index, readValue);
            *val = readValue;
            return IIO_VAL_INT;

        case IIO_CHAN_INFO_SCALE:
            *val = 1;
            return IIO_VAL_INT;

        case IIO_CHAN_INFO_SAMP_FREQ:
            readValue = ak09940_i2c_read(akm->client, AK09940_REG_CNTL3, 1,
                                         &cntlValue);
            readValue &= 0x0F;
            pr_info("[AK09940] %s : mode=%d, MODEbits=%X\n", __func__,
                    akm->mode, cntlValue);
            *val = akm->mode;
            return IIO_VAL_INT;
    }

    return -EINVAL;
}

static int ak09940_set_measurement_freq(

    struct ak09940_data *akm,
    int                 val)
{
    u8  mode;
    int n;

    akdbgprt("[AK09940] %s freq = %d\n", __FUNCTION__, (int)val);

    mode = ak09940_get_continue_mode_by_interval(akm, val);


    ak09940_set_continue_mode_measure(akm, mode);

    akdbgprt("[AK09940] %s mode = %d\n", __FUNCTION__, akm->mode);

    return 0;
}

static int ak09940_write_raw(
    struct iio_dev             *indio_dev,
    struct iio_chan_spec const *chan,
    int                        val,
    int                        val2,
    long                       mask)
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


// ******************************************************************************************
static void ak09940_send_event(struct iio_dev *indio_dev)
{
    struct ak09940_data *akm = iio_priv(indio_dev);
    struct i2c_client   *client = akm->client;
#ifdef AK09940_DEBUG
    int i;
#endif
    u8 rdata[AK09940_READ_DATA_LENTH];
    /* data(32bit) * 3-axis + status(16bit) + timestamp(64bit) */
    u8  event[sizeof(s32) * 3 + sizeof(s16) + sizeof(s64)];
    s32 *pevent;

    akdbgprt("[AK09940] %s(%d)\n", __FUNCTION__, __LINE__);

    mutex_lock(&akm->lock);
    ak09940_i2c_read(client, AK09940_REG_ST1, AK09940_READ_DATA_LENTH, rdata);

    memset(event, 0, sizeof(event));
#if 0//def AK09940_DEBUG

    for (i = 0; i < AK09940_READ_DATA_LENTH; i++) {
        akdbgprt("[AK09940] %s %d %02XH\n", __func__, i, rdata[i]);
    }
#endif

    event[0] = rdata[AK09940_ST1_POS];
    event[1] = rdata[AK09940_ST2_POS];

    pevent = &event[2];
    ak09940_parse_raw_data(&rdata[AK09940_DATA_POS], pevent);

    if (ak09940_data_check_overflow(pevent)) {
        akdbgprt("[AK09940] %s(%d)  read mag data overflow!!!!!!!!!\n",
                 __FUNCTION__, __LINE__);
    }

#if 0//def AK09940_DEBUG

    for (i = 0; i < AK09940_READ_DATA_LENTH - 1; i += 3) {
        akdbgprt("[AK09940] %s %d %02XH,%02XH,%02XH\n", __func__, i, event[i],
                 event[i + 1], event[i + 2]);
    }
#endif

#ifdef AK09940_DEBUG
    akdbgprt("[AK09940] %s mag, %04XH,%04XH,%04XH\n", __func__, *pevent,
             *pevent + 1, *pevent + 2);
#endif

#ifdef KERNEL_3_18_XX
    iio_push_to_buffers_with_timestamp(indio_dev, event, iio_get_time_ns());
#else
    iio_push_to_buffers_with_timestamp(indio_dev, event,
                                       iio_get_time_ns(indio_dev));
#endif

    mutex_unlock(&akm->lock);

    return;
}

static irqreturn_t ak09940_handle_trigger(
    int  irq,
    void *p)
{
    const struct iio_poll_func *pf = p;
    struct iio_dev             *indio_dev = pf->indio_dev;

    akdbgprt("[AK09940] %s(%d), time:%lld\n", __FUNCTION__, __LINE__, iio_get_time_ns(
                 indio_dev));

    ak09940_send_event(indio_dev);
    iio_trigger_notify_done(indio_dev->trig);
    return IRQ_HANDLED;
}

// ******************************************************************************************
static int ak09940_setup(struct i2c_client *client)
{
    struct iio_dev      *indio_dev = i2c_get_clientdata(client);
    struct ak09940_data *akm = iio_priv(indio_dev);
    s32                 err;
    u8                  deviceID[2];
    u8                  ctnl3;

    akdbgprt("[AK09940] %s(%d)\n", __FUNCTION__, __LINE__);

    if (akm->rstn_gpio > 0) {
        gpio_set_value(akm->rstn_gpio, 0);
        mdelay(1);
        gpio_set_value(akm->rstn_gpio, 1);
        mdelay(1);
    }

    err = ak09940_i2c_read(client, AK09940_REG_WIA, 2, deviceID);

    if (err < 0) {
        pr_err("[AK09940]i2c read failure, AK09940_REG_WIA err=%d\n", err);
        return -EIO;
    }

    printk("[AK09940] Device ID = %x,%x\n", deviceID[0], deviceID[1]);

    if ((deviceID[0] << 8 | deviceID[1] ) != AK09940_DEVICE_ID) {
        pr_err("[AK09940] Device ID = %x,%x\n", deviceID[0], deviceID[1]);
        return -EIO;
    }

    if (akm->mode < ARRAY_SIZE(measurementModeRegTable)) {
        ctnl3 = AK09940_WM_EN(akm->watermark_en) + AK09940_SDR(akm->SDRbit) +
            measurementModeRegTable[akm->mode];
    } else {
        ctnl3 = 0;
    }

    err = ak09940_i2c_write(client, AK09940_REG_CNTL3, ctnl3);

    akm->test_lolim[0] = AK09940_TEST_LOLIM_X;
    akm->test_lolim[1] = AK09940_TEST_LOLIM_Y;
    akm->test_lolim[2] = AK09940_TEST_LOLIM_Z;

    akm->test_hilim[0] = AK09940_TEST_HILIM_X;
    akm->test_hilim[1] = AK09940_TEST_HILIM_Y;
    akm->test_hilim[2] = AK09940_TEST_HILIM_Z;

    akm->freq_table = measurementFreqTable;
    akm->modeReg_table = measurementModeRegTable;
    akm->freq_num = ARRAY_SIZE(measurementFreqTable);
    return 0;
}

static int ak09940_parse_dt(struct ak09940_data *ak09940)
{
    struct device      *dev;
    struct device_node *np;
    int                ret;

    dev = &(ak09940->client->dev);
    np = dev->of_node;

    if (!np) {
        return -EINVAL;
    }

    ak09940->rstn_gpio = of_get_named_gpio(np, "ak09940,rst_gpio", 0);

    if (ak09940->rstn_gpio < 0) {
        akdbgprt("[AK09940] %s : rstn gpio is null\n", __FUNCTION__);
        ak09940->rstn_gpio = -1;
    } else {
        if (!gpio_is_valid(ak09940->rstn_gpio)) {
            printk(KERN_ERR "AK09940 rstn pin(%u) is valid\n",
                   ak09940->rstn_gpio);
            ak09940->rstn_gpio = -1;
        } else {
            ret = gpio_request(ak09940->rstn_gpio, "ak09940 rstn");
            akdbgprt("[AK09940] %s : gpio_request ret = %d\n", __FUNCTION__,
                     ret);
            gpio_direction_output(ak09940->rstn_gpio, 0);
        }
    }


    ret = of_property_read_u8(np, "ak09940,opetation_mode", &ak09940->mode);

    if (ret && (ret != -EINVAL)) {
        ak09940->mode = 0;
        akdbgprt("[AK09940] %s mode(error) = %d\n", __FUNCTION__,
                 ak09940->mode);
    } else {
        if (ak09940->mode > AK09940_MODE_NUM - 1) {
            ak09940->mode = AK09940_MODE_CONT_400HZ;
        }
    }

    ret = of_property_read_u8(np, "ak09940,drive_setting", &ak09940->SDRbit);

    if (ret && (ret != -EINVAL)) {
        ak09940->SDRbit = 0;
    }

    ret = of_property_read_u8(np, "ak09940,watermark", &ak09940->watermark);

    if (ret && (ret != -EINVAL)) {
        ak09940->watermark = 0;
    }

    if (ak09940->watermark != 0) {
        ak09940->watermark_en = 1;
    }

    ret = of_property_read_u8(np, "ak09940,temperature", &ak09940->TEMPbit);

    if (ret && (ret != -EINVAL)) {
        ak09940->TEMPbit = 0;
    }

    return 0;
}

static int ak09940_set_trigger_state(
    struct iio_trigger *trig,
    bool               state)
{
    //struct iio_dev      *indio_dev = iio_trigger_get_drvdata(trig);
    //struct ak09940_data *ak09940 = iio_priv(indio_dev);
    pr_err("%s called. st=%s", __func__, (state ? "true" : "false"));

    return 0;
}

static const struct iio_trigger_ops ak09940_trigger_ops = {
    .set_trigger_state = ak09940_set_trigger_state,
    .owner = THIS_MODULE,
};


static int ak09940_probe(
    struct i2c_client          *client,
    const struct i2c_device_id *id)
{
    struct ak09940_data *akm;
    struct iio_dev      *indio_dev;
    struct device       *dev = &client->dev;
    int                 err;
    const char          *name = NULL;

    akdbgprt("[AK09940] %s(%d)\n", __FUNCTION__, __LINE__);


    /* Register with IIO */
    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*akm));

    if (indio_dev == NULL) {
        return -ENOMEM;
    }

    akm = iio_priv(indio_dev);
    i2c_set_clientdata(client, indio_dev);

    akm->client = client;

    akm->numMode = ARRAY_SIZE(measurementModeRegTable);

    akm->int_gpio =
        of_get_named_gpio(client->dev.of_node, "ak09940,int_gpio", 0);

    if (gpio_is_valid(akm->int_gpio)) {
        err = devm_gpio_request_one(&client->dev, akm->int_gpio,
                                    GPIOF_IN, "ak09940_int");

        if (err < 0) {
            dev_err(&client->dev,
                    "AK09940] failed to request GPIO %d, error %d\n",
                    akm->int_gpio, err);
            return err;
        }
    }

    akm->irq = gpio_to_irq(akm->int_gpio);

    err = ak09940_parse_dt(akm);

    if (err < 0) {
        dev_err(&client->dev, "[AK09940] Device Tree Setting was not found!\n");
    }

    if (id) {
        name = id->name;
    }

    if (akm->irq) {
        akm->trig = iio_trigger_alloc(
                "%s-dev%d",
                name, indio_dev->id);
        err = devm_request_irq(&client->dev, akm->irq,
                               iio_trigger_generic_data_rdy_poll,
                               IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                               dev_name(&client->dev), akm->trig);

        if (err) {
            dev_err(&client->dev, "[AK09940] devm_request_irq Error! err=%d\n",
                    err);
        }

        akm->trig->dev.parent = dev;
        akm->trig->ops = &ak09940_trigger_ops;
        iio_trigger_set_drvdata(akm->trig, indio_dev);
        /* indio_dev->trig = iio_trigger_get(akm->trig); */
        err = iio_trigger_register(akm->trig);

        if (err) {
            dev_err(&client->dev,
                    "[AK09940] iio_trigger_register Error! err=%d\n", err);
        } else {
            indio_dev->trig = akm->trig;
        }
    }


    err = ak09940_setup(client);

    if (err < 0) {
        dev_err(&client->dev, "%s initialization fails\n", name);
        return err;
    }

    mutex_init(&akm->lock);
    indio_dev->dev.parent = &client->dev;
    indio_dev->channels = ak09940_channels;
    indio_dev->num_channels = ARRAY_SIZE(ak09940_channels);
    indio_dev->info = &ak09940_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->name = name;

    err = iio_triggered_buffer_setup(indio_dev, NULL, ak09940_handle_trigger,
                                     NULL);

    if (err) {
        dev_err(&client->dev, "triggered buffer setup failed\n");
        return err;
    }

    akdbgprt("[AK09940] %s(iio_triggered_buffer_setup=%d)\n", __FUNCTION__,
             err);

    err = iio_device_register(indio_dev);

    if (err) {
        dev_err(&client->dev, "device register failed\n");
    }

    akdbgprt("[AK09940] %s(iio_device_register=%d)\n", __FUNCTION__, err);

    return err;
}

static int ak09940_remove(struct i2c_client *client)
{
    struct iio_dev      *indio_dev = i2c_get_clientdata(client);
    struct ak09940_data *akm = iio_priv(indio_dev);

    iio_trigger_free(akm->trig);
    iio_device_unregister(indio_dev);
    iio_triggered_buffer_cleanup(indio_dev);

    ak09940_i2c_write(client, AK09940_REG_CNTL3, 0);

    if (akm->rstn_gpio > 0) {
        gpio_set_value(akm->rstn_gpio, 0);
        msleep(1);
        gpio_free(akm->rstn_gpio);
    }


    return 0;
}

static int ak09940_i2c_suspend(struct device *dev)
{
    struct i2c_client   *i2c = to_i2c_client(dev);
    struct iio_dev      *indio_dev = i2c_get_clientdata(i2c);
    struct ak09940_data *akm = iio_priv(indio_dev);

    ak09940_i2c_write(i2c, AK09940_REG_CNTL3, 0);

    if (akm->rstn_gpio > 0) {
        gpio_set_value(akm->rstn_gpio, 0);
    }

    return 0;
}

static int ak09940_i2c_resume(struct device *dev)
{
    struct i2c_client *i2c = to_i2c_client(dev);

    ak09940_setup(i2c);

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
    { .compatible = "akm,ak09940"},
    {}
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
