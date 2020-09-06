/*
 * ADXL345 accelerometer driver
 *
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */


#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>

#include "../../../arch/arm/mach-msm/proc_comm.h"

#define ADXL345_VERSION "1.2.0"
#define ADXL345_NAME    "adxl345"

/* for debugging */
#define DEBUG 0
#define DEBUG_DELAY 0
#define DEBUG_THRESHOLD 0
#define TRACE_FUNC() pr_debug(ADXL345_NAME ": <trace> %s()\n", __FUNCTION__)
#define DBG 



#ifdef CONFIG_HAS_EARLYSUSPEND
#if DEBUG
#undef ADXL345_HAS_EARLYSUSPEND
#else
#define ADXL345_HAS_EARLYSUSPEND
#endif
#endif

/*
 * Default parameters
 */
#define CONFIG_INPUT_ADXL345_POSITION		0
#define ADXL345_DEFAULT_DELAY               200
#define ADXL345_MAX_DELAY                   2000
#define ADXL345_MIN_DELAY                   10

/*
 * Registers
 */
#define ADXL345_DEVID_REG                   0x00
#define ADXL345_DEVID                       0xe6

#define ADXL345_ACC_REG                     0x32

#define ADXL345_SOFT_RESET_REG              0x1d
#define ADXL345_SOFT_RESET_MASK             0x80
#define ADXL345_SOFT_RESET_SHIFT            7

#define ADXL345_POWER_CONTROL_REG           0x2d
#define ADXL345_POWER_CONTROL_MASK          0x08
#define ADXL345_POWER_CONTROL_SHIFT         3

#define ADXL345_RANGE_REG                   0x31
#define ADXL345_RANGE_MASK                  0x03
#define ADXL345_RANGE_SHIFT                 0
#define ADXL345_RANGE_16G                   3
#define ADXL345_RANGE_8G                    2
#define ADXL345_RANGE_4G                    1
#define ADXL345_RANGE_2G                    0

#define ADXL345_BANDWIDTH_REG               0x2c
#define ADXL345_BANDWIDTH_MASK              0x0f
#define ADXL345_BANDWIDTH_SHIFT             0
#define ADXL345_BANDWIDTH_800HZ             13
#define ADXL345_BANDWIDTH_400HZ             12
#define ADXL345_BANDWIDTH_200HZ             11
#define ADXL345_BANDWIDTH_100HZ             10
#define ADXL345_BANDWIDTH_50HZ               9
#define ADXL345_BANDWIDTH_25HZ               8
#define ADXL345_BANDWIDTH_12HZ               7
#define ADXL345_BANDWIDTH_6HZ                6
#define ADXL345_BANDWIDTH_3HZ                5
#define ADXL345_BANDWIDTH_2HZ                4
#define ADXL345_BANDWIDTH_1HZ                3

#define  NV_TSB_TOP_ITEMS_I  10000
#define  NV_ACCM_X_OFFSET_I  (NV_TSB_TOP_ITEMS_I+37)
#define  NV_ACCM_Y_OFFSET_I  (NV_TSB_TOP_ITEMS_I+38)
#define  NV_ACCM_Z_OFFSET_I  (NV_TSB_TOP_ITEMS_I+39)


/*
 * Acceleration measurement
 */
#define ADXL345_RESOLUTION                   256
#define GRAVITY_EARTH_RES                    43585

/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                        9806550
#define ABSMIN_2G                            (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                            (GRAVITY_EARTH * 2)

#if DEBUG
#define DEVID                                0x00
#define THRESH_TAP                           0x1d
#define OFSX                                 0x1e
#define OFSY                                 0x1f
#define OFSZ                                 0x20
#define DUR                                  0x21
#define LATENT                               0x22
#define WINDOW                               0x23
#define THRESH_ACT                           0x24
#define THRESH_INACT                         0x25
#define TIME_INACT                           0x26
#define ACT_INACT_CTL                        0x27
#define THRESH_FF                            0x28
#define TIME_FF                              0x29
#define TAP_AXES                             0x2a
#define ACT_TAP_STATUS                       0x2b
#define BW_RATE                              0x2c
#define POWER_CTL                            0x2d
#define INT_ENABLE                           0x2e
#define INT_MAP                              0x2f
#define INT_SOURCE                           0x30
#define DATA_FORMAT                          0x31
#define DATAX0                               0x32
#define DATAX1                               0x33
#define DATAY0                               0x34
#define DATAY1                               0x35
#define DATAZ0                               0x36
#define DATAZ1                               0x37
#define FIFO_CTL                             0x38
#define FIFO_STATUS                          0x39
#endif /* DEBUG */

struct acceleration {
    int x;
    int y;
    int z;
};

/*
 * Output data rate
 */
struct adxl345_odr {
        unsigned long delay;            /* min delay (msec) in the range of ODR */
        u8 odr;                         /* bandwidth register value */
};



/* 
 * Data structure for  mesuring the data for calibration.
 */
#define GSCALC_DATA_NUM		(20)

typedef struct {
	   int x[3];
} Adxl_data_t;

typedef struct {
	int  gCalculatorIndex;
	int gCalculatorNum; 
	Adxl_data_t gCalculatorData[GSCALC_DATA_NUM];
}Adxl_Gs_Data_t;

static Adxl_Gs_Data_t AdxlGs_Data;

#if 1
static long g_nv_accm_x = 0;
static long g_nv_accm_y = 0;
static long g_nv_accm_z = 0;
static int g_x_off = 0;
static int g_y_off = 0;
static int g_z_off = 0;
#endif

#define ADXL_IOCTL_SET_CALIB_INIT	(1)
#define ADXL_IOCTL_SET_CALIB_DONE	(6)
#define ADXL_IOCTL_GET_CALIB_DATA	(3)
#define ADXL_IOCTL_SET_CALIB_DATA	(4)
#define ADXL_IOCTL_GET_CALIB_REQ	(5)

static atomic_t adxl_calibration_requested;
static atomic_t adxl_calibration_progress;

static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static struct mutex adxl345_work_lock;

static const struct adxl345_odr adxl345_odr_table[] = {
    {1,     ADXL345_BANDWIDTH_800HZ},
    {2,     ADXL345_BANDWIDTH_400HZ},
    {5,     ADXL345_BANDWIDTH_200HZ},
    {10,    ADXL345_BANDWIDTH_100HZ},
    {20,    ADXL345_BANDWIDTH_50HZ},
    {40,    ADXL345_BANDWIDTH_25HZ},
    {80,    ADXL345_BANDWIDTH_12HZ}, /* 12.5Hz   */
    {160,   ADXL345_BANDWIDTH_6HZ},  /*  6.25Hz  */
    {320,   ADXL345_BANDWIDTH_3HZ},  /*  3.125Hz */
    {640,   ADXL345_BANDWIDTH_2HZ},  /*  1.563Hz */
    {1280,  ADXL345_BANDWIDTH_1HZ},  /*  0.782Hz */
};

/*
 * Transformation matrix for chip mounting position
 */
static const int adxl345_position_map[][3][3] = {
    {{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}}, /* top/upper-left */
    {{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}}, /* top/upper-right */
    {{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}}, /* top/lower-right */
    {{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}}, /* top/lower-left */
    {{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}}, /* bottom/upper-left */
    {{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}}, /* bottom/upper-right */
    {{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}}, /* bottom/lower-right */
    {{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}}, /* bottom/upper-left */
};
/*
 * driver private data
 */
struct adxl345_data {
    atomic_t enable;                /* attribute value */
    atomic_t delay;                 /* attribute value */
    atomic_t position;              /* attribute value */
    atomic_t threshold;             /* attribute value */
    struct acceleration last;       /* last measured data */
    struct mutex enable_mutex;
    struct mutex data_mutex;
    struct spi_device  *spi;
    struct input_dev *input;
    struct delayed_work work;
#if DEBUG
    int suspend;
#endif
#ifdef ADXL345_HAS_EARLYSUSPEND
	struct early_suspend     e_sus_fcn;
#endif
};

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

#ifdef ADXL345_HAS_EARLYSUSPEND
static void adxl345_early_suspend(struct early_suspend *h);
static void adxl345_late_resume(struct early_suspend *h);
#endif

static struct mutex g_start_mutex;
static struct adxl345_data *g_adxl345_data;
static atomic_t g_enable;

static void
gsStableData_reset(void)
{
    AdxlGs_Data.gCalculatorIndex = 0;
    AdxlGs_Data.gCalculatorNum = 0;
}

static int
gsStableData_add(Adxl_data_t *pdata)
{
    if (pdata == 0) {
        return -1;
    }
    AdxlGs_Data.gCalculatorData[AdxlGs_Data.gCalculatorIndex++] = *pdata;
    AdxlGs_Data.gCalculatorNum++;

    if (AdxlGs_Data.gCalculatorIndex >= GSCALC_DATA_NUM) {
        AdxlGs_Data.gCalculatorIndex = 0;
    }
    if (AdxlGs_Data.gCalculatorNum >= GSCALC_DATA_NUM) {
        AdxlGs_Data.gCalculatorNum = GSCALC_DATA_NUM;
    }
    return 0;
}


static char spi_adxl345_byte_read(struct spi_device  *spi, unsigned char reg)
{
	char tx_buf[8];
	char rx_buf[8];
	struct spi_message  m;
	struct spi_transfer t;
	int rc;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	tx_buf[0] = (reg | 0x80); /* ORing with 0x80=10000000b,R/W bit 1,mode bit 0, rest 6 bits are
								 reg address*/
	t.len     = 2;
	rc = spi_sync(spi, &m);
	if(rc < 0)
	{
		printk("\nADXL345 READ ERROR : rc value :[%d]====\n",rc);
	}
		
	return (rx_buf[1]); /*returning the read data byte*/

}


static void spi_adxl345_byte_write(struct spi_device  *spi, unsigned char reg, char data)
{
	char tx_buf[8];
	struct spi_message  m;
	struct spi_transfer t;
	int rc;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = NULL/*rx_buf*/;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	tx_buf[0] = (reg | 0x00);
	tx_buf[1] = data;
	t.len     = 2;
	rc = spi_sync(spi, &m);
	if(rc < 0)
	{
		printk("\nADXL345 WRITE ERROR : rc value :[%d]====\n",rc);
	}
		
}


static int spi_adxl345_multi_byte_read(struct spi_device  *spi, unsigned char reg, int len, char *data)
{
	char tx_buf[8];
	char rx_buf[8];
	struct spi_message  m;
	struct spi_transfer t;
	int rc;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	tx_buf[0] = (reg | 0xC0); /* ORing with 0xC0=11000000b,R/W bit 1,mode bit 1, rest 6 bits are
								 reg address*/
	t.len     = len + 1; //7;  As we need to ask with  7 bytes.
	rc = spi_sync(spi, &m);
	memcpy(data,rx_buf+1,6);
	if(rc < 0)
	{
		printk("\nADXL345 READ ERROR : rc value :[%d]====\n",rc);
		return rc;
	}
	return len;
		
}

/* register access functions */
#define adxl345_read_bits(p,r) \
    ((spi_adxl345_byte_read((p)->spi, r##_REG) & r##_MASK) >> r##_SHIFT)
#define adxl345_update_bits(p,r,v) \
    spi_adxl345_byte_write((p)->spi, r##_REG, \
                              ((spi_adxl345_byte_read((p)->spi,r##_REG) & ~r##_MASK) | ((v) << r##_SHIFT)))



/*
 * Device dependant operations
 */
static int adxl345_power_up(struct adxl345_data *adxl345)
{
    adxl345_update_bits(adxl345, ADXL345_POWER_CONTROL, 1);

    return 0;
}

static int adxl345_power_down(struct adxl345_data *adxl345)
{
    adxl345_update_bits(adxl345, ADXL345_POWER_CONTROL, 0);

    return 0;
}

static int adxl345_hw_init(struct adxl345_data *adxl345)
{
    adxl345_power_down(adxl345);

    adxl345_update_bits(adxl345, ADXL345_RANGE, ADXL345_RANGE_2G);

    return 0;
}

static int adxl345_get_enable(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    return atomic_read(&adxl345->enable);
}

static void adxl345_set_enable(struct device *dev, int enable)
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);
    int delay = atomic_read(&adxl345->delay);

	mutex_lock(&g_start_mutex);
	g_adxl345_data = spi_get_drvdata(spi);
	mutex_unlock(&g_start_mutex);
	atomic_set(&g_enable,enable);

    mutex_lock(&adxl345->enable_mutex);

    if (enable) {                   /* enable if state will be changed */
        if (!atomic_cmpxchg(&adxl345->enable, 0, 1)) {
            adxl345_power_up(adxl345);
            schedule_delayed_work(&adxl345->work, delay_to_jiffies(delay) + 1);
        }
    } else {                        /* disable if state will be changed */
        if (atomic_cmpxchg(&adxl345->enable, 1, 0)) {
            cancel_delayed_work_sync(&adxl345->work);
            adxl345_power_down(adxl345);
        }
    }
    atomic_set(&adxl345->enable, enable);

    mutex_unlock(&adxl345->enable_mutex);
}

static int adxl345_get_delay(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    return atomic_read(&adxl345->delay);
}

static void adxl345_set_delay(struct device *dev, int delay)
{

    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);
    u8 odr;
    int i;

    /* determine optimum ODR */
    for (i = 1; (i < ARRAY_SIZE(adxl345_odr_table)) &&
             (actual_delay(delay) >= adxl345_odr_table[i].delay); i++)
        ;
    odr = adxl345_odr_table[i-1].odr;
    atomic_set(&adxl345->delay, delay);

    mutex_lock(&adxl345->enable_mutex);

    if (adxl345_get_enable(dev)) {
        cancel_delayed_work_sync(&adxl345->work);
        adxl345_update_bits(adxl345, ADXL345_BANDWIDTH, odr);
        schedule_delayed_work(&adxl345->work, delay_to_jiffies(delay) + 1);
    } else {
        adxl345_power_up(adxl345);
        adxl345_update_bits(adxl345, ADXL345_BANDWIDTH, odr);
        adxl345_power_down(adxl345);
    }

    mutex_unlock(&adxl345->enable_mutex);
}

static int adxl345_get_position(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    return atomic_read(&adxl345->position);
}

static void adxl345_set_position(struct device *dev, int position)
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    atomic_set(&adxl345->position, position);
}

static int adxl345_get_threshold(struct device *dev)
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    return atomic_read(&adxl345->threshold);
}

static void adxl345_set_threshold(struct device *dev, int threshold)
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    atomic_set(&adxl345->threshold, threshold);
}

static int adxl345_data_filter(struct device *dev, struct acceleration *accel, int data[])
{
    struct spi_device *spi = to_spi_device(dev);
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);
    int threshold = atomic_read(&adxl345->threshold);
#if DEBUG_THRESHOLD
    int update;
#endif
#if DEBUG_THRESHOLD
    update = 0;
#endif
    mutex_lock(&adxl345->data_mutex);
    if ((abs(adxl345->last.x - data[0]) > threshold) ||
        (abs(adxl345->last.y - data[1]) > threshold) ||
        (abs(adxl345->last.z - data[2]) > threshold)) {
        accel->x = data[0];
        accel->y = data[1];
        accel->z = data[2];
#if DEBUG_THRESHOLD
        update = 1;
#endif
    } else {
        *accel = adxl345->last;
    }

#if DEBUG_THRESHOLD
    if (update == 1) {
        dev_info(&spi->dev, "threshold=%d x(%d) y(%d) z(%d) accel(%d,%d,%d) ****\n", threshold,
                 adxl345->last.x - data[0], adxl345->last.y - data[1], adxl345->last.z - data[2], accel->x, accel->y, accel->z);
    } else {
        dev_info(&spi->dev, "threshold=%d x(%d) y(%d) z(%d) accel(%d,%d,%d)\n", threshold,
                 adxl345->last.x - data[0], adxl345->last.y - data[1], adxl345->last.z - data[2], accel->x, accel->y, accel->z);
    }
#endif
    mutex_unlock(&adxl345->data_mutex);

    return 0;
}

static void input_calib_data(int *data)
{
	int x, y, z;
	int calib_req = atomic_read(&adxl_calibration_requested);
	Adxl_data_t cAdxlData;
    mutex_lock(&adxl345_work_lock);
	if (calib_req == 1) {
		gsStableData_reset();
		atomic_set(&adxl_calibration_progress, 1);
		atomic_set(&adxl_calibration_requested, 0);
	}
	x = data[0];
	y = data[1];
	z = data[2];
	cAdxlData.x[0] = x;
	cAdxlData.x[1] = y;
	cAdxlData.x[2] = z;
	gsStableData_add(&cAdxlData);
	x = x - g_x_off;
	y = y - g_y_off;
	z = z - g_z_off;
	data[0] = x;
	data[1] = y;
	data[2] = z;
    mutex_unlock(&adxl345_work_lock);
}


static int adxl345_measure(struct adxl345_data *adxl345, struct acceleration *accel)
{
    struct spi_device *spi = adxl345->spi;
    u8 buf[6];
    int raw[3], data[3];
    int pos = atomic_read(&adxl345->position);
    long long g;
    int i, j;
#if DEBUG_DELAY
    struct timespec t;
#endif

#if DEBUG_DELAY
    getnstimeofday(&t);
#endif

    /* read acceleration data */
    if (spi_adxl345_multi_byte_read(spi, ADXL345_ACC_REG, 6, buf) != 6) {
        dev_err(&spi->dev,
                "I2C block read error: addr=0x%02x, len=%d\n",
                ADXL345_ACC_REG, 6);
            raw[0] = 0;
            raw[1] = 0;
            raw[2] = 0;
    } else {
        raw[0] = (int) (s16)(((buf[1]) << 8) | buf[0]);
        raw[1] = (int) (s16)(((buf[3]) << 8) | buf[2]);
        raw[2] = (int) (s16)(((buf[5]) << 8) | buf[4]);
    }

    /* for X, Y, Z axis */
    for (i = 0; i < 3; i++) {
        /* coordinate transformation */
        data[i] = 0;
        for (j = 0; j < 3; j++) {
            data[i] += raw[j] * adxl345_position_map[pos][i][j];
        }
	}
	input_calib_data(data);
    for (i = 0; i < 3; i++) {
		/* Input for calibration*/
        /* normalization */
        g = (long long)data[i] * GRAVITY_EARTH / ADXL345_RESOLUTION;
        data[i] = g;
    }

    dev_dbg(&spi->dev, "raw(%5d,%5d,%5d) => norm(%8d,%8d,%8d)\n",
            raw[0], raw[1], raw[2], data[0], data[1], data[2]);

    DBG("raw(%5d,%5d,%5d) => norm(%8d,%8d,%8d)\n", raw[0], raw[1], raw[2], data[0], data[1], data[2]);

#if DEBUG_DELAY
    dev_info(&spi->dev, "%ld.%lds:raw(%5d,%5d,%5d) => norm(%8d,%8d,%8d)\n", t.tv_sec, t.tv_nsec,
             raw[0], raw[1], raw[2], data[0], data[1], data[2]);
#endif

    adxl345_data_filter(&spi->dev, accel, data);

    return 0;
}

static void adxl345_work_func(struct work_struct *work)
{
    struct adxl345_data *adxl345 = container_of((struct delayed_work *)work, struct adxl345_data, work);
    struct acceleration accel;
    unsigned long delay = delay_to_jiffies(atomic_read(&adxl345->delay));

    adxl345_measure(adxl345, &accel);
    mutex_lock(&adxl345_work_lock);



    input_report_abs(adxl345->input, ABS_X, accel.x);
    input_report_abs(adxl345->input, ABS_Y, accel.y);
    input_report_abs(adxl345->input, ABS_Z, accel.z);
    input_sync(adxl345->input);
    mutex_unlock(&adxl345_work_lock);

    mutex_lock(&adxl345->data_mutex);
    adxl345->last = accel;
    mutex_unlock(&adxl345->data_mutex);

    schedule_delayed_work(&adxl345->work, delay);
}

/*
 * Input device interface
 */
static int adxl345_input_init(struct adxl345_data *adxl345)
{
    struct input_dev *dev;
    int err;

    dev = input_allocate_device();
    if (!dev) {
        return -ENOMEM;
    }
    dev->name = "accelerometer";
    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_drvdata(dev, adxl345);

    err = input_register_device(dev);
    if (err < 0) {
        input_free_device(dev);
        return err;
    }
    adxl345->input = dev;

    return 0;
}

static void adxl345_input_fini(struct adxl345_data *adxl345)
{
    struct input_dev *dev = adxl345->input;

    input_unregister_device(dev);
    input_free_device(dev);
}

/*
 * sysfs device attributes
 */
static ssize_t adxl345_enable_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", adxl345_get_enable(dev));
}

static ssize_t adxl345_enable_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);

    if ((enable == 0) || (enable == 1)) {
        adxl345_set_enable(dev, enable);
    }

    return count;
}

static ssize_t adxl345_delay_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", adxl345_get_delay(dev));
}

static ssize_t adxl345_delay_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
    unsigned long delay = simple_strtoul(buf, NULL, 10);

    if (delay > ADXL345_MAX_DELAY) {
        delay = ADXL345_MAX_DELAY;
    }
	if (delay < ADXL345_MIN_DELAY) {
		delay = ADXL345_MIN_DELAY;
	}

    adxl345_set_delay(dev, delay);

    return count;
}

static ssize_t adxl345_position_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", adxl345_get_position(dev));
}

static ssize_t adxl345_position_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    unsigned long position;

    position = simple_strtoul(buf, NULL,10);
    if ((position >= 0) && (position <= 7)) {
        adxl345_set_position(dev, position);
    }

    return count;
}

static ssize_t adxl345_threshold_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", adxl345_get_threshold(dev));
}

static ssize_t adxl345_threshold_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
    unsigned long threshold;

    threshold = simple_strtoul(buf, NULL,10);
    if (threshold >= 0 && threshold <= ABSMAX_2G) {
        adxl345_set_threshold(dev, threshold);
    }

    return count;
}

static ssize_t adxl345_wake_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    static atomic_t serial = ATOMIC_INIT(0);

    input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

    return count;
}

static ssize_t adxl345_data_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct adxl345_data *adxl345 = input_get_drvdata(input);
    struct acceleration accel;

    mutex_lock(&adxl345->data_mutex);
    accel = adxl345->last;
    mutex_unlock(&adxl345->data_mutex);

    return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}

#if DEBUG
static ssize_t adxl345_debug_reg_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct adxl345_data *adxl345 = input_get_drvdata(input);
    struct spi_device *spi = adxl345->spi;
    ssize_t count = 0;
    u8 reg;

    reg = spi_adxl345_byte_read(spi, DEVID);
    count += sprintf(&buf[count], "%02x: %d\n", DEVID, reg);

    reg = spi_adxl345_byte_read(spi, THRESH_TAP);
    count += sprintf(&buf[count], "%02x: %d\n", THRESH_TAP, reg);

    reg = spi_adxl345_byte_read(spi, OFSX);
    count += sprintf(&buf[count], "%02x: %d\n", OFSX, reg);

    reg = spi_adxl345_byte_read(spi, OFSY);
    count += sprintf(&buf[count], "%02x: %d\n", OFSY, reg);

    reg = spi_adxl345_byte_read(spi, OFSZ);
    count += sprintf(&buf[count], "%02x: %d\n", OFSZ, reg);

    reg = spi_adxl345_byte_read(spi, DUR);
    count += sprintf(&buf[count], "%02x: %d\n", DUR, reg);

    reg = spi_adxl345_byte_read(spi, LATENT);
    count += sprintf(&buf[count], "%02x: %d\n", LATENT, reg);

    reg = spi_adxl345_byte_read(spi, WINDOW);
    count += sprintf(&buf[count], "%02x: %d\n", WINDOW, reg);

    reg = spi_adxl345_byte_read(spi, THRESH_ACT);
    count += sprintf(&buf[count], "%02x: %d\n", THRESH_ACT, reg);

    reg = spi_adxl345_byte_read(spi, THRESH_INACT);
    count += sprintf(&buf[count], "%02x: %d\n", THRESH_INACT, reg);

    reg = spi_adxl345_byte_read(spi, TIME_INACT);
    count += sprintf(&buf[count], "%02x: %d\n", TIME_INACT, reg);

    reg = spi_adxl345_byte_read(spi, ACT_INACT_CTL);
    count += sprintf(&buf[count], "%02x: %d\n", ACT_INACT_CTL, reg);

    reg = spi_adxl345_byte_read(spi, THRESH_FF);
    count += sprintf(&buf[count], "%02x: %d\n", THRESH_FF, reg);

    reg = spi_adxl345_byte_read(spi, TIME_FF);
    count += sprintf(&buf[count], "%02x: %d\n", TIME_FF, reg);

    reg = spi_adxl345_byte_read(spi, TAP_AXES);
    count += sprintf(&buf[count], "%02x: %d\n", TAP_AXES, reg);

    reg = spi_adxl345_byte_read(spi, ACT_TAP_STATUS);
    count += sprintf(&buf[count], "%02x: %d\n", ACT_TAP_STATUS, reg);

    reg = spi_adxl345_byte_read(spi, BW_RATE);
    count += sprintf(&buf[count], "%02x: %d\n", BW_RATE, reg);

    reg = spi_adxl345_byte_read(spi, POWER_CTL);
    count += sprintf(&buf[count], "%02x: %d\n", POWER_CTL, reg);

    reg = spi_adxl345_byte_read(spi, INT_ENABLE);
    count += sprintf(&buf[count], "%02x: %d\n", INT_ENABLE, reg);

    reg = spi_adxl345_byte_read(spi, INT_MAP);
    count += sprintf(&buf[count], "%02x: %d\n", INT_MAP, reg);

    reg = spi_adxl345_byte_read(spi, INT_SOURCE);
    count += sprintf(&buf[count], "%02x: %d\n", INT_SOURCE, reg);

    reg = spi_adxl345_byte_read(spi, DATA_FORMAT);
    count += sprintf(&buf[count], "%02x: %d\n", DATA_FORMAT, reg);

    reg = spi_adxl345_byte_read(spi, DATAX0);
    count += sprintf(&buf[count], "%02x: %d\n", DATAX0, reg);

    reg = spi_adxl345_byte_read(spi, DATAX1);
    count += sprintf(&buf[count], "%02x: %d\n", DATAX1, reg);

    reg = spi_adxl345_byte_read(spi, DATAY0);
    count += sprintf(&buf[count], "%02x: %d\n", DATAY0, reg);

    reg = spi_adxl345_byte_read(spi, DATAY1);
    count += sprintf(&buf[count], "%02x: %d\n", DATAY1, reg);

    reg = spi_adxl345_byte_read(spi, DATAZ0);
    count += sprintf(&buf[count], "%02x: %d\n", DATAZ0, reg);

    reg = spi_adxl345_byte_read(spi, DATAZ1);
    count += sprintf(&buf[count], "%02x: %d\n", DATAZ1, reg);

    reg = spi_adxl345_byte_read(spi, FIFO_CTL);
    count += sprintf(&buf[count], "%02x: %d\n", FIFO_CTL, reg);

    reg = spi_adxl345_byte_read(spi, FIFO_STATUS);
    count += sprintf(&buf[count], "%02x: %d\n", FIFO_STATUS, reg);

    return count;
}

static int adxl345_suspend(struct spi_device *spi, pm_message_t mesg);
static int adxl345_resume(struct spi_device *spi);

static ssize_t adxl345_debug_suspend_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct adxl345_data *adxl345 = input_get_drvdata(input);

    return sprintf(buf, "%d\n", adxl345->suspend);
}

static ssize_t adxl345_debug_suspend_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct adxl345_data *adxl345 = input_get_drvdata(input);
    struct spi_device *spi = adxl345->spi;
    unsigned long suspend = simple_strtoul(buf, NULL, 10);

    if (suspend) {
        pm_message_t msg;
        adxl345_suspend(spi, msg);
    } else {
        adxl345_resume(spi);
    }

    return count;
}
#endif /* DEBUG */

//static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWUGO,
                   adxl345_enable_show, adxl345_enable_store);
//static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWUGO,
                   adxl345_delay_show, adxl345_delay_store);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR,
                   adxl345_position_show, adxl345_position_store);
static DEVICE_ATTR(threshold, S_IRUGO|S_IWUSR,
                   adxl345_threshold_show, adxl345_threshold_store);
//static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, adxl345_wake_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP|S_IWUGO, NULL, adxl345_wake_store);
//static DEVICE_ATTR(data, S_IRUGO,
static DEVICE_ATTR(data, S_IRUGO|S_IWUGO,
                   adxl345_data_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_reg, S_IRUGO,
                   adxl345_debug_reg_show, NULL);
static DEVICE_ATTR(debug_suspend, S_IRUGO|S_IWUSR,
                   adxl345_debug_suspend_show, adxl345_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *adxl345_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_position.attr,
    &dev_attr_threshold.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
#if DEBUG
    &dev_attr_debug_reg.attr,
    &dev_attr_debug_suspend.attr,
#endif /* DEBUG */
    NULL
};

static struct attribute_group adxl345_attribute_group = {
    .attrs = adxl345_attributes
};

/*
 * I2C client
 */
static int adxl345_detect(struct spi_device *spi)
{
    int id;

    id = spi_adxl345_byte_read(spi, ADXL345_DEVID_REG);
	DBG("ID value is = %x, expected= %x\n",id,ADXL345_DEVID);
    if (id != ADXL345_DEVID)
        return -ENODEV;

    return 0;
}

#if 1

static int adxl345_open(struct inode *inode, struct file *file)
{
	int ret = -1;
    DBG(KERN_INFO "***** ACCEL_DEBUG :ENTER %s\n", __func__);
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			ret = 0;
		}
	}
    DBG(KERN_INFO "***** ACCEL_DEBUG :LEAVE %s, ret = %d\n", __func__,ret);
	return ret;
	return 0;
}

static int adxl345_release(struct inode *inode, struct file *file)
{
    DBG(KERN_INFO "***** ACCEL_DEBUG : ENTER %s \n", __func__);
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
    DBG(KERN_INFO "***** ACCEL_DEBUG : LEAVE %s \n", __func__);
	return 0;
}

static int adxl_read_nvitem(unsigned int id, long *data)
{
        int rc;
        unsigned long l_zero = 0;

        DBG("===ADXL func :[%s], line : [%d], id : [%d], data : [%d], ====\n", __func__, __LINE__,id, *data);

        /* NULL check */
        if( data == NULL )
        {
                DBG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* VALUE check */
        if(( NV_ACCM_X_OFFSET_I > id ) || ( id > NV_ACCM_Z_OFFSET_I ))
        {
                DBG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* read NV */
label1 :
        rc = msm_proc_comm(PCOM_NV_READ, &id, (unsigned *)data);
        DBG("ADXL_NV_DEBUG:Error %s(%d), ID [%d], after msm_proc_comm, return val :[%d]! \n", __func__, __LINE__,id, rc);
        if( rc )
        {
                DBG("ADXL_NV_DEBUG:Error %s(%d), ID [%d], going to write again, return val :[%d]! \n", __func__, __LINE__,id, rc);
                /* write NV */
                rc = msm_proc_comm(PCOM_NV_WRITE, &id, (unsigned *)&l_zero);
                if( rc )
                {
                        DBG("===ADXL func :[%s], line : [%d], return : [%d], WRITE ERROR====\n", __func__, __LINE__,rc);
                }
                else
                {
                        goto label1;
                }


        }

        return rc;

}


static int adxl_write_nvitem(unsigned int id, long *data)
{
        int rc;

        DBG("===ADXL func :[%s], line : [%d], id : [%d], data : [%d], ====\n", __func__, __LINE__,id, *data);

        /* NULL check */
        if( data == NULL )
        {
                DBG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* VALUE check */
        if(( NV_ACCM_X_OFFSET_I > id ) || ( id > NV_ACCM_Z_OFFSET_I ))
        {
                DBG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* write VN */
        rc = msm_proc_comm(PCOM_NV_WRITE, &id, (unsigned *)data);
        if( rc )
        {
#if 1 //DEBUG -->
                DBG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
#endif//<-- DEBUG
        }


        return rc;
}

static int convert_int2signedchar(int input, signed char *output)
{
signed char cnt=0;
int i=0;
		DBG("\n input [%d]===\n", input);
        if(input < 0)
        {
		*output = abs(input);
		DBG("\n <ADXL> output value [%d]===\n", *output);
        *output = (signed char)((*output) * (-1));
        }
	else
		*output=input;

return 0;
}


#if 1
static int adxl345_get_calibration_data(void)
{
	int rc = -1;
	signed char x,y,z;

        DBG("***** adxl345_get_calibration_data : ENTER %s \n", __func__);

		/* read X axi offset */
	rc = adxl_read_nvitem( NV_ACCM_X_OFFSET_I, (signed char *)&x);
		/* read Y axi offset */
		if (!rc)
		{
			rc = adxl_read_nvitem( NV_ACCM_Y_OFFSET_I, (signed char *)&y);
		}
		/* read Z axi offset */
		if (!rc)
		{
			rc = adxl_read_nvitem( NV_ACCM_Z_OFFSET_I, (signed char *)&z);
		}
	DBG("\n<ADXL>====GET calibration values-x[%d],y[%d],z[%d]===\n",x,y,z);
	if(!rc)
	{
	g_x_off = x * 4;
	g_y_off = y * 4;
	g_z_off = z * 4;
	}
	DBG("\n ADXL NV calibration values ,values [%d,%d,%d]:===\n", g_x_off,g_y_off,g_z_off);
	DBG("\n===<ADXL>===adxl345_get_calibration_data, nv ram return value:[%d]===\n",rc);

	return rc;
}

static int adxl345_set_calibration_data(void)
{
        int rc = -1;
	signed char x,y,z;

    	DBG("***** adxl345_set_calibration_data : ENTER %s \n", __func__);
	if((g_nv_accm_x !=0) && (g_nv_accm_y != 0) && (g_nv_accm_z !=0))
	{
		g_nv_accm_x = g_nv_accm_x / 4;
		g_nv_accm_y = g_nv_accm_y / 4;
		g_nv_accm_z = g_nv_accm_z / 4;

		convert_int2signedchar(g_nv_accm_x, &x);
		convert_int2signedchar(g_nv_accm_y, &y);
		convert_int2signedchar(g_nv_accm_z, &z);
		DBG("\n<ADXL>====set calibration values-x[%d],y[%d],z[%d]===\n",x,y,z);
                /* read X axi offset */
                rc = adxl_write_nvitem( NV_ACCM_X_OFFSET_I, (signed char *)&x);
                /* read Y axi offset */
                if (!rc)
                {
                        rc = adxl_write_nvitem( NV_ACCM_Y_OFFSET_I, (signed char *)&y);
                }
                /* read Z axi offset */
                if (!rc)
                {
                        rc = adxl_write_nvitem( NV_ACCM_Z_OFFSET_I, (signed char *)&z);
                }
	}
	DBG("\n===<ADXL>===adxl345_set_calibration_data, nv ram return value:[%d]===\n",rc);
        return rc;
}


#endif



static int
adxl345_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	Adxl_data_t  *p = 0, calibdata;
	Adxl_Gs_Data_t *pGsData;
	int result = 0;
    DBG(KERN_INFO "***** ACCEL_DEBUG :ENTER %s cmd=%x\n", __func__, cmd);
	mutex_lock(&adxl345_work_lock);
	switch (cmd) {
	  case ADXL_IOCTL_SET_CALIB_INIT:
		  atomic_set(&adxl_calibration_requested, 1);
    	  DBG(KERN_INFO "Request for Calibration:%d\n",atomic_read(&adxl_calibration_requested));
		  break;
	  case ADXL_IOCTL_SET_CALIB_DONE:
		  atomic_set(&adxl_calibration_progress, 0);
    	  DBG(KERN_INFO "Calibration Done:%d\n",atomic_read(&adxl_calibration_progress));
		  DBG("\n ADXL ADXL_IOCTL_SET_CALIB_DONE:===\n");
		  break;
	  case ADXL_IOCTL_SET_CALIB_DATA:
		  {
			DBG("\n ADXL ADXL_IOCTL_SET_CALIB_DATA:===\n");
			  p = (Adxl_data_t *)argp;
			  if (copy_from_user(&calibdata, p, sizeof(Adxl_data_t))) {
					result = -EFAULT;
			  } else {
				if((calibdata.x[0] == 0) && (calibdata.x[1] == 0) && (calibdata.x[2] == 0))
				{
				  adxl345_get_calibration_data();
				}
				else
				{
				  g_nv_accm_x = calibdata.x[0];
				  g_nv_accm_y = calibdata.x[1];
				  g_nv_accm_z = calibdata.x[2];
				  g_x_off = g_nv_accm_x;
				  g_y_off = g_nv_accm_y;
				  g_z_off = g_nv_accm_z;
				  adxl345_set_calibration_data();
				}
				DBG("\n ADXL ADXL_IOCTL_SET_CALIB_DATA,values [%d,%d,%d]:===\n", calibdata.x[0],calibdata.x[1],calibdata.x[2]);
			  }
		  }
		  break;
	  case ADXL_IOCTL_GET_CALIB_DATA:
		  {
			DBG("\n ADXL ADXL_IOCTL_GET_CALIB_DATA:===\n");
			 pGsData = (Adxl_Gs_Data_t *)argp;
			 if (copy_to_user(pGsData, &AdxlGs_Data, sizeof(Adxl_Gs_Data_t))) {
    			 DBG(KERN_INFO "***** ACCEL_DEBUG :copy to user failed:%d\n",__LINE__);
				  result = -EFAULT;
			 }
		  }
		  break;
	  case ADXL_IOCTL_GET_CALIB_REQ:
		   {
			  int status = atomic_read(&adxl_calibration_progress);
			  if (copy_to_user(argp, &status, sizeof(status))) {
    			  	  DBG(KERN_INFO "***** ACCEL_DEBUG :copy to user failed:%d\n",__LINE__);
					  result = -EFAULT;
			  }
		   }
		  break;

	   default:
    	  DBG(KERN_INFO "default case\n");
	}
	mutex_unlock(&adxl345_work_lock);

	return result;
}

#endif

#if 1
static struct file_operations adxl345_fops = {
	.owner = THIS_MODULE,
	.open = adxl345_open,
	.release = adxl345_release,
	.ioctl = adxl345_ioctl,
};

static struct miscdevice adxl345_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "adxl345_2",
	.fops = &adxl345_fops,
};
#endif

static int adxl345_probe(struct spi_device *spi)
{
    struct adxl345_data *adxl345;
    int err;
	int rc;
    TRACE_FUNC();

    /* setup private data */
    adxl345 = kzalloc(sizeof(struct adxl345_data), GFP_KERNEL);
    if (!adxl345) {
        err = -ENOMEM;
        goto error_0;
    }

    mutex_init(&adxl345->enable_mutex);
    mutex_init(&adxl345->data_mutex);
    mutex_init(&adxl345_work_lock);
    mutex_init(&g_start_mutex);

	atomic_set(&g_enable,0);



    /* setup spi data */
	spi_set_drvdata(spi, adxl345);
    adxl345->spi = spi;

    /* detect and init hardware */
    if (err = adxl345_detect(spi)) {
        goto error_1;
    }

    adxl345_hw_init(adxl345);
    adxl345_set_delay(&spi->dev, ADXL345_DEFAULT_DELAY);
    adxl345_set_position(&spi->dev, CONFIG_INPUT_ADXL345_POSITION);

    /* setup driver interfaces */
    INIT_DELAYED_WORK(&adxl345->work, adxl345_work_func);

    err = adxl345_input_init(adxl345);
    if (err < 0) {
        goto error_1;
    }

    err = sysfs_create_group(&adxl345->input->dev.kobj, &adxl345_attribute_group);
    if (err < 0) {
        goto error_2;
    }

	spi_setup(spi);

#ifdef ADXL345_HAS_EARLYSUSPEND
	adxl345->e_sus_fcn.suspend = adxl345_early_suspend;
	adxl345->e_sus_fcn.resume = adxl345_late_resume;
	register_early_suspend(&adxl345->e_sus_fcn);
#endif

if (misc_register(&adxl345_device))
	{
		DBG(KERN_ERR
		       "adxl345_probe: adxl345_device register failed\n");

		goto error_0;
	}

    return 0;

error_2:
    adxl345_input_fini(adxl345);
error_1:
	spi_set_drvdata(spi, NULL);
    kfree(adxl345);
error_0:
    return err;
}

static int adxl345_remove(struct spi_device *spi)
{
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    adxl345_set_enable(&spi->dev, 0);

    sysfs_remove_group(&adxl345->input->dev.kobj, &adxl345_attribute_group);
    adxl345_input_fini(adxl345);

    kfree(adxl345);

    return 0;
}

static int adxl345_suspend(struct spi_device *spi, pm_message_t mesg)
{
#ifdef ADXL345_HAS_EARLYSUSPEND
	return 0;
#else
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);

    mutex_lock(&adxl345->enable_mutex);

    if (adxl345_get_enable(&spi->dev)) {
        cancel_delayed_work_sync(&adxl345->work);
        adxl345_power_down(adxl345);
    }

#if DEBUG
    adxl345->suspend = 1;
#endif

    mutex_unlock(&adxl345->enable_mutex);
    return 0;
#endif
}

static int adxl345_resume(struct spi_device *spi)
{
#ifdef ADXL345_HAS_EARLYSUSPEND
    return 0;
#else
    struct adxl345_data *adxl345 = spi_get_drvdata(spi);
    int delay = atomic_read(&adxl345->delay);

    adxl345_hw_init(adxl345);
    adxl345_set_delay(&spi->dev, delay);

    mutex_lock(&adxl345->enable_mutex);

    if (adxl345_get_enable(&spi->dev)) {
        adxl345_power_up(adxl345);
        schedule_delayed_work(&adxl345->work, delay_to_jiffies(delay) + 1);
    }

#if DEBUG
    adxl345->suspend = 0;
#endif

    mutex_unlock(&adxl345->enable_mutex);
    return 0;
#endif
}


#ifdef ADXL345_HAS_EARLYSUSPEND
static void adxl345_early_suspend(struct early_suspend *h)
{
    struct adxl345_data *adxl345 = 
		container_of(h, struct adxl345_data, e_sus_fcn);
	struct spi_device * spi = adxl345->spi;

    mutex_lock(&adxl345->enable_mutex);

    if (adxl345_get_enable(&spi->dev)) {
        cancel_delayed_work_sync(&adxl345->work);
        adxl345_power_down(adxl345);
    }

    mutex_unlock(&adxl345->enable_mutex);

	return;
}

static void adxl345_late_resume(struct early_suspend *h)
{
    struct adxl345_data *adxl345 = 
		container_of(h, struct adxl345_data, e_sus_fcn);
	struct spi_device * spi = adxl345->spi;
    int delay = atomic_read(&adxl345->delay);

    adxl345_hw_init(adxl345);
    adxl345_set_delay(&spi->dev, delay);

    mutex_lock(&adxl345->enable_mutex);

    if (adxl345_get_enable(&spi->dev)) {
        adxl345_power_up(adxl345);
        schedule_delayed_work(&adxl345->work, delay_to_jiffies(delay) + 1);
    }

    mutex_unlock(&adxl345->enable_mutex);
	return;
}
#endif

void adxl345_stop_ap(void)
{
	if(atomic_read(&g_enable))
	{
		mutex_lock(&g_start_mutex);
		cancel_delayed_work_sync(&g_adxl345_data->work);
        adxl345_power_down(g_adxl345_data);
		mutex_unlock(&g_start_mutex);
	}

}

EXPORT_SYMBOL(adxl345_stop_ap);

void adxl345_start_ap(void)
{
	
	if (atomic_read(&g_enable)) 
	{
		int delay = atomic_read(&g_adxl345_data->delay);
		mutex_lock(&g_start_mutex);
		adxl345_hw_init(g_adxl345_data);
		adxl345_set_delay(&g_adxl345_data->spi->dev, delay);


		adxl345_power_up(g_adxl345_data);
		schedule_delayed_work(&g_adxl345_data->work, delay_to_jiffies(delay)+ 1);
		mutex_unlock(&g_start_mutex);
	}


}

EXPORT_SYMBOL(adxl345_start_ap);

static struct spi_driver adxl345_driver = {
    .driver = {
        .name = ADXL345_NAME,
        .owner = THIS_MODULE,
    },
    .probe = adxl345_probe,
    .remove = adxl345_remove,
    .suspend = adxl345_suspend,
    .resume = adxl345_resume,
};



static int __init adxl345_init(void)
{
	return spi_register_driver(&adxl345_driver);

}

static void __exit adxl345_exit(void)
{
	spi_unregister_driver(&adxl345_driver);
}

module_init(adxl345_init);
module_exit(adxl345_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_DESCRIPTION("ADXL345 accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(ADXL345_VERSION);
