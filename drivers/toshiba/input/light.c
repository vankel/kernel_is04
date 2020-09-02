/* 
  Light Sensor Driver

  Protocol driver for Light sensors.
  Copyiiright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>

/* for I2C remote mutex */
#include <linux/remote_spinlock.h>



#include <linux/moduleparam.h>

#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <mach/vreg.h>
#include "light.h"

#include "../arch/arm/mach-msm/proc_comm.h"

/* for I2C remote mutex */
#include "../arch/arm/mach-msm/smd_private.h"

#define LIGHT_USE_EARLY_SUSPEND	(1)

#if LIGHT_USE_EARLY_SUSPEND
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#endif
//Flags for parallel userspace and driver handling

/* for debugging */
#define DEBUG 0

/* for debugging */
//#define LIGHT_EXPORT_ADC_FUNCTION 

//Light Sensor
#define SENSOR_TYPE (5)

#if SENSOR_TYPE == 1
#define SENSOR_NAME "accelerometer"
#elif SENSOR_TYPE == 2
#define SENSOR_NAME "geomagnetic"
#elif SENSOR_TYPE == 3
#define SENSOR_NAME "orientation"
#elif SENSOR_TYPE == 4
#define SENSOR_NAME "gyroscope"
#elif SENSOR_TYPE == 5
#define SENSOR_NAME "light"
#elif SENSOR_TYPE == 6
#define SENSOR_NAME "pressure"
#elif SENSOR_TYPE == 7
#define SENSOR_NAME "temperature"
#elif SENSOR_TYPE == 8
#define SENSOR_NAME "proximity"
#endif

#define SENSOR_DEFAULT_DELAY            (1000)   /* 1000 ms */
#define SENSOR_LED_OFF_DELAY            (150)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)
#define MAX_LUX_ENTRIES					(225)
#define OUT_OF_SCOPE_SLOPE				(0)
#define NUM_AVG              			(1)
#define LIGHT_DEFAULT_VAL		600

#if DEBUG
#define LIGHT_DBG(x...)  printk(x)
#else
#define LIGHT_DBG(x...) 
#endif

#ifdef LIGHT_USE_I2C_INTERFACE
#define BKL_I2C_SLAVE (0xEC >> 1)
static unsigned char saved_parameter[6];
static struct i2c_adapter *i2c_tmdrv;
#endif
static struct input_dev *input_data = NULL;
#if LIGHT_USE_EARLY_SUSPEND
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend  e_sus_fcn;
void light_early_suspend(struct early_suspend *h);
void light_late_resume(struct early_suspend *h);
static int light_suspended = 0;
static int light_lux_adjust = 0;
static int light_adjust_max = 25;
#endif
#endif

/* for I2C remote mutex */
#define SMEM_SPINLOCK_I2C   "S:6"
static uint32_t *rmutex_i2c;
static remote_spinlock_t rslock_i2c;

static struct timer_list tmdrv_timer;
static struct work_struct g_proxi_work_data;
static unsigned int g_lux_val = 0;
static atomic_t g_total_lux_val;
static atomic_t g_total_adc_val;
static atomic_t g_led_on_off;
static atomic_t g_timer_is_pending;
static int g_ls_op_cnt = 0;
static int g_ls_probe_init = 0;
static int g_inside_sw = 0;



struct sensor_data {
    struct input_dev *input_device;
    struct mutex mutex;
    int enabled;
    int delay;
#if DEBUG
    int suspend;
#endif
};

static const unsigned int adc_to_lux[MAX_LUX_ENTRIES][2] = {
	/*ADC*/ /*LUX*/
	{ 0,	0   },
	{ 1,	0   },
	{ 2,	27  },
	{ 3,	40  },
	{ 4,	53  },
	{ 5,	67  },
	{ 6,	80  },
	{ 7,	93  },
	{ 8,	107 },
	{ 9,	120 },
	{10,	133 },
	{11,	147 },
	{12,	160 },
	{13,	173 },
	{14,	187 },
	{15,	200 },
	{16,	213 },
	{17,	227 },
	{18,	240 },
	{19,	253 },
	{20,	267 },
	{21,	280 },
	{22,	293 },
	{23,	307 },
	{24,	320 },
	{25,	333 },
	{26,	347 },
	{27,	360 },
	{28,	373 },
	{29,	387 },
	{30,	400 },
	{31,	413 },
	{32,	427 },
	{33,	440 },
	{34,	453 },
	{35,	467 },
	{36,	480 },
	{37,	493 },
	{38,	507 },
	{39,	520 },
	{40,	533 },
	{41,	547 },
	{42,	560 },
	{43,	573 },
	{44,	587 },
	{45,	600 },
	{46,	613 },
	{47,	627 },
	{48,	640 },
	{49,	653 },
	{50,	667 },
	{51,	680 },
	{52,	693 },
	{53,	707 },
	{54,	720 },
	{55,	733 },
	{56,	747 },
	{57,	760 },
	{58,	773 },
	{59,	787 },
	{60,	800 },
	{61,	813 },
	{62,	827 },
	{63,	840 },
	{64,	853 },
	{65,	867 },
	{66,	880 },
	{67,	893 },
	{68,	907 },
	{69,	920 },
	{70,	933 },
	{71,	947 },
	{72,	960 },
	{73,	973 },
	{74,	987 },
	{75,	1000},
	{76,	1013},
	{77,	1027},
	{78,	1040},
	{79,	1053},
	{80,	1067},
	{81,	1080},
	{82,	1093},
	{83,	1107},
	{84,	1120},
	{85,	1133},
	{86,	1147},
	{87,	1160},
	{88,	1173},
	{89,	1187},
	{90,	1200},
	{91,	1213},
	{92,	1227},
	{93,	1240},
	{94,	1253},
	{95,	1267},
	{96,	1280},
	{97,	1293},
	{98,	1307},
	{99,	1320},
	{100,	1333},
	{101,	1347},
	{102,	1360},
	{103,	1373},
	{104,	1387},
	{105,	1400},
	{106,	1413},
	{107,	1427},
	{108,	1440},
	{109,	1453},
	{110,	1467},
	{111,	1480},
	{112,	1493},
	{113,	1507},
	{114,	1520},
	{115,	1533},
	{116,	1547},
	{117,	1560},
	{118,	1573},
	{119,	1587},
	{120,	1600},
	{121,	1613},
	{122,	1627},
	{123,	1640},
	{124,	1653},
	{125,	1667},
	{126,	1680},
	{127,	1693},
	{128,	1707},
	{129,	1720},
	{130,	1733},
	{131,	1747},
	{132,	1760},
	{133,	1773},
	{134,	1787},
	{135,	1800},
	{136,	1813},
	{137,	1827},
	{138,	1840},
	{139,	1853},
	{140,	1867},
	{141,	1880},
	{142,	1893},
	{143,	1907},
	{144,	1920},
	{145,	1933},
	{146,	1947},
	{147,	1960},
	{148,	1973},
	{149,	1987},
	{150,	2000},
	{151,	2013},
	{152,	2027},
	{153,	2040},
	{154,	2053},
	{155,	2067},
	{156,	2080},
	{157,	2093},
	{158,	2107},
	{159,	2120},
	{160,	2133},
	{161,	2147},
	{162,	2160},
	{163,	2173},
	{164,	2187},
	{165,	2200},
	{166,	2213},
	{167,	2227},
	{168,	2240},
	{169,	2253},
	{170,	2267},
	{171,	2280},
	{172,	2293},
	{173,	2307},
	{174,	2320},
	{175,	2333},
	{176,	2347},
	{177,	2360},
	{178,	2373},
	{179,	2387},
	{180,	2400},
	{181,	2413},
	{182,	2427},
	{183,	2440},
	{184,	2453},
	{185,	2467},
	{186,	2480},
	{187,	2493},
	{188,	2507},
	{189,	2520},
	{190,	2533},
	{191,	2547},
	{192,	2560},
	{193,	2573},
	{194,	2587},
	{195,	2600},
	{196,	2613},
	{197,	2627},
	{198,	2640},
	{199,	2653},
	{200,	2667},
	{201,	2680},
	{202,	2693},
	{203,	2707},
	{204,	2720},
	{205,	2733},
	{206,	2747},
	{207,	2760},
	{208,	2773},
	{209,	2787},
	{210,	2800},
	{211,	2813},
	{212,	2827},
	{213,	2840},
	{214,	2853},
	{215,	2867},
	{216,	2880},
	{217,	2893},
	{218,	2907},
	{219,	2920},
	{220,	2933},
	{221,	2947},
	{222,	3000},
	{223,	3500},
	{224,	4000},
};

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;
static struct sensor_data *data = NULL;

/*
 *	Getting the LUX Value  from this Table. 
 *	Since the sensor control specification provided by Toshiba has some definite mapping 
 *	between hex ADC values and lux value, but the actual ADC values are sometimes coming 
 *	within that range (but not the exact value provided in the mapping) and sometimes 
 *	more than that, we have taken the ratio of generating the lux values from the ADC 
 *	values whenever the ADC value does not exactly match the table (provided in control 
 *	specification).e.g. When ADC value comes as 0x92 we return 2500 since we find a match
 *	in the table, but when it comes 0x93 we return 2793
 *
 */

unsigned int
get_lux_from_adc(unsigned int data)
{
	if (data  >= MAX_LUX_ENTRIES) 
	{
		return (adc_to_lux[MAX_LUX_ENTRIES - 1][1]);
	}
	else
	{
		return (adc_to_lux[data][1]);
	}
}

/* for I2C remote mutex */
static void i2c_rspin_lock(uint32_t *rmutex)
{
    int lock = 0;
    unsigned long flags;

    do {
        remote_spin_lock_irqsave(&rslock_i2c, flags);
        if (*rmutex == 0) {
            *rmutex = 1;
            lock = 1;
        }
        remote_spin_unlock_irqrestore(&rslock_i2c, flags);
        schedule();
    } while (!lock);
}

/* for I2C remote mutex */
static void i2c_rspin_unlock(uint32_t *rmutex)
{
    unsigned long flags;
    remote_spin_lock_irqsave(&rslock_i2c, flags);
    *rmutex = 0;
    remote_spin_unlock_irqrestore(&rslock_i2c, flags);
}

unsigned
get_proc_comm_data(void)
{
    unsigned data1, data2;
	int ret;
    data1 = 5; /* adc_read_lx */
    data2 = 0;

    /* waiting for I2C remote mutex */
    if(rmutex_i2c != NULL) {
        i2c_rspin_lock(rmutex_i2c);
        i2c_rspin_unlock(rmutex_i2c);
    }

    ret = msm_proc_comm(PCOM_OEM_008, &data1, &data2);
    /* LIGHT_DBG("[nak]get_proc_comm_data() - adc qsd %02x, ret=%d\n", data2,ret); */
    return data2;
}

/*
 * checks if the sensor has been opened already
 * and only if everything has been initialized
 * before, it schedules the bottom half.
 *
*/
static int
light_open_sensor(void)
{
   LIGHT_DBG("\n[LIGHT] OPEN called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
   mutex_lock(&data->mutex);
   if (0 >= g_ls_op_cnt)
   {
	   if (0 == g_inside_sw)
	   {
	   		schedule_work(&g_proxi_work_data);
	   }
   }
   g_ls_op_cnt ++;
   mutex_unlock(&data->mutex);
   return 0;
}
/*
 * checks if open count is >0 and only then
 * decrements the open counter
 * 
 *
*/
static int
light_close_sensor(void)
{
    LIGHT_DBG("\n[LIGHT] OPEN called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
	mutex_lock(&data->mutex);
	if (0 == g_ls_op_cnt) 
	{
		mutex_unlock(&data->mutex);
		return -1;
	}
	g_ls_op_cnt --;
	mutex_unlock(&data->mutex);
	return 0;
}

#ifdef LIGHT_USE_I2C_INTERFACE
/*
 * Does i2c write to the internal registers
 * of light sensor
 * 
 *
*/

static int 
tmdrv_i2c_write(uint16_t reg, uint8_t *data, uint32_t len)
{
  struct i2c_msg msg;
  u_int8_t buf[64];
  int ret = 0;

  msg.addr = BKL_I2C_SLAVE;
  msg.flags = 0;
  buf[0] = reg;
  memcpy(&buf[1],data,len);

  msg.buf  = buf;
  msg.len  = len + 1;
  ret = i2c_transfer(i2c_tmdrv, &msg, 1);
  if (ret < 0)
  {
    LIGHT_DBG("[tmdrv]i2c_transfer() falied!(%d) in %s()\n",ret , __func__);
  }
  return ret;
}



/*
 * Does i2c read from the internal registers
 * of light sensor
 * 
 *
*/

static int 
tmdrv_i2c_read(uint16_t reg, uint8_t *data, uint32_t len)
{
  struct i2c_msg msg[2];
  u_int8_t msgbuf[2];
  int ret = 0;

  memcpy(msgbuf, &reg, sizeof(reg));

  msg[0].addr  = BKL_I2C_SLAVE;
  msg[0].flags = 0;
  msg[0].buf   = msgbuf;
  msg[0].len   = 1;

  msg[1].addr  = BKL_I2C_SLAVE;
  msg[1].flags = I2C_M_RD;
  msg[1].buf   = data;
  msg[1].len   = len;

  ret = i2c_transfer(i2c_tmdrv, msg, 2);
  if (ret != 2)
  {
    printk("[tmdrv]i2c_transfer() falied!(%d) in %s()\n",ret , __func__);
  }
  return ret;
}

/*
 * Does i2c read / write to the internal registers
 * of light sensor to initialize the light sensor
 * 
 *
*/
static void 
light_i2c_set(void)
{
	unsigned char wbuf = 0;
	tmdrv_i2c_read(0x02, &saved_parameter[2], 1);
    LIGHT_DBG("[tmdrv]i2c_transfer()  data:%d\n",saved_parameter[2]);
	wbuf = (0x08 | saved_parameter[2]);
	tmdrv_i2c_write(0x02, &wbuf, 1);
}
#endif

#ifdef LIGHT_SYSFS
/* Sysfs interface */
static ssize_t
sensor_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int delay;

    mutex_lock(&data->mutex);

    delay = data->delay;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", delay);
}

static ssize_t
sensor_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);
    int enabled;

    if (value < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < value) {
        value = SENSOR_MAX_DELAY;
    }

    mutex_lock(&data->mutex);

    enabled = data->enabled;
    data->delay = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT, (enabled<<16) | value);

    mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
sensor_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int enabled;

    mutex_lock(&data->mutex);

    enabled = data->enabled;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", enabled);
}

static ssize_t
sensor_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);
    int delay;

    if (value != 0 && value != 1) {
        return count;
    }

    mutex_lock(&data->mutex);

    delay = data->delay;
    data->enabled = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT, (value<<16) | delay);

    mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
sensor_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    static int cnt = 1;

    input_report_abs(input_data, ABS_WAKE, cnt++);

    return count;
}

static ssize_t
sensor_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
#if SENSOR_TYPE <= 4
    int x, y, z;
#else
    int x;
#endif

    spin_lock_irqsave(&input_data->event_lock, flags);

    x = input_data->abs[ABS_X];
#if SENSOR_TYPE <= 4
    y = input_data->abs[ABS_Y];
    z = input_data->abs[ABS_Z];
#endif

    spin_unlock_irqrestore(&input_data->event_lock, flags);

#if SENSOR_TYPE <= 4
    return sprintf(buf, "%d %d %d\n", x, y, z);
#else
    return sprintf(buf, "%d\n", x);
#endif
}

static ssize_t
sensor_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int status;

    spin_lock_irqsave(&input_data->event_lock, flags);

    status = input_data->abs[ABS_STATUS];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}

#if DEBUG
static int sensor_suspend(struct platform_device *pdev, pm_message_t state);
static int sensor_resume(struct platform_device *pdev);

static ssize_t 
sensor_debug_suspend_show(struct device *dev,
                          struct device_attribute *attr,
						  char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);

    return sprintf(buf, "%d\n", data->suspend);
}

static ssize_t 
sensor_debug_suspend_store(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{
    unsigned long suspend = simple_strtoul(buf, NULL, 10);

    if (suspend) {
        pm_message_t msg;
        sensor_suspend(sensor_pdev, msg);
    } else {
        sensor_resume(sensor_pdev);
    }

    return count;
}
#endif /* DEBUG */

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_delay_show, sensor_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_enable_show, sensor_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
        NULL, sensor_wake_store);
static DEVICE_ATTR(data, S_IRUGO, sensor_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, sensor_status_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_suspend, S_IRUGO|S_IWUSR,
                   sensor_debug_suspend_show, sensor_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *sensor_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
#if DEBUG
    &dev_attr_debug_suspend.attr,
#endif /* DEBUG */
    NULL
};

static struct attribute_group sensor_attribute_group = {
    .attrs = sensor_attributes
};

#endif

static int
sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
    /* implement suspend of the sensor */
//    LIGHT_DBG("%s: sensor_suspend\n", SENSOR_NAME);

  //  if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
        /* suspend gyroscope */
  //  }
  //  else if (strcmp(SENSOR_NAME, "light") == 0) {
        /* suspstatic light */
 //   }
  //  else if (strcmp(SENSOR_NAME, "pressure") == 0) {
        /* suspend pressure */
   // }
   // else if (strcmp(SENSOR_NAME, "temperature") == 0) {
        /* suspend temperature */
 //   }
  //  else if (strcmp(SENSOR_NAME, "proximity") == 0) {
        /* suspend proximity */
  //  }

#if DEBUG
  //  {
    //    struct sensor_data *data = input_get_drvdata(this_data);
      //  data->suspend = 1;
   // }
#endif /* DEBUG */

    return 0;
}

static int
sensor_resume(struct platform_device *pdev)
{
    /* implement resume of the sensor */
//    LIGHT_DBG("%s: sensor_resume\n", SENSOR_NAME);

  //  if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
        /* resume gyroscope */
  //  }
  //  else if (strcmp(SENSOR_NAME, "light") == 0) {
        /* resume light */
  //  }
  //  else if (strcmp(SENSOR_NAME, "pressure") == 0) {
        /* resume pressure */
  //  }
  //  else if (strcmp(SENSOR_NAME, "temperature") == 0) {
        /* resume temperature */
  //  }
  //  else if (strcmp(SENSOR_NAME, "proximity") == 0) {
        /* resume proximity */
   // }

#if DEBUG
   // {
     //   struct sensor_data *data = input_get_drvdata(this_data);
       // data->suspend = 0;
   // }
#endif /* DEBUG */

    return 0;
}




/*LED driver to set 1 for ON and 0 for OFF*/
void led_notify_light(int on_off)
{
		static int prev_status = 3;


        atomic_set(&g_led_on_off, on_off);

        if(prev_status == atomic_read(&g_led_on_off))
        {
                return;
        }
        else
        {

            if(!atomic_read(&g_led_on_off))
			{
				mod_timer(&tmdrv_timer, jiffies + msecs_to_jiffies(SENSOR_LED_OFF_DELAY));
				atomic_set(&g_timer_is_pending,1);
			}
        }
		prev_status = atomic_read(&g_led_on_off);

}

EXPORT_SYMBOL(led_notify_light);




/*
 * This is the bottom half which fires
 * at regular intervals and reads the ADC
 * value using msm_proc_comm and then converting
 *it to lux, finally syncing it to input subsystem
*/
static void 
light_work_bh(struct work_struct *work)
{

	unsigned data2;
	unsigned int conv_lux = 0;
	int i=0;
	int lux_val=0;

	mutex_lock(&data->mutex);
	g_inside_sw = 1;
	mutex_unlock(&data->mutex);
#ifdef LIGHT_USE_I2C_INTERFACE	
	light_i2c_set();
#endif
#if LIGHT_USE_EARLY_SUSPEND
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (0 == light_suspended) 
#endif
#endif
	{
		if(!atomic_read(&g_led_on_off)) /* LED is OFF, read value from ADC */
		{
			for(i = 0; i < NUM_AVG; i++)
			{
				data2  = get_proc_comm_data();
				conv_lux = get_lux_from_adc(data2);
				LIGHT_DBG("[LIGHT] Hardware ADC values %d LUX Value : %d\n",data2,conv_lux);
				g_lux_val += conv_lux;
			}
			atomic_set(&g_total_adc_val, data2);
			atomic_set(&g_total_lux_val,(g_lux_val / NUM_AVG));

		}

		lux_val = atomic_read(&g_total_lux_val);
		g_lux_val = 0;
		LIGHT_DBG("[LIGHT] light_work_bh is called=== Lux = %d\n",lux_val);
		if ((light_lux_adjust < light_adjust_max) || (atomic_read(&g_led_on_off)))
		{
			static int last_data = 0;
			if (light_lux_adjust == 0) 
			{
				last_data = lux_val;
			}
			else
			{
				if (lux_val == last_data) 
				{
					if (lux_val > 0) 
					{
						lux_val --;
					}
					else 
					{
						lux_val ++;
					}
				}
				last_data = lux_val;
			}
			light_lux_adjust ++;
		}
		input_report_abs(input_data, ABS_X, lux_val);
		input_sync(input_data);
	}
//	tmdrv_timer.expires = jiffies + msecs_to_jiffies(SENSOR_DEFAULT_DELAY);

	mutex_lock(&data->mutex);
	if (g_ls_op_cnt > 0)
	{
		if(atomic_read(&g_timer_is_pending))
		{
			mod_timer(&tmdrv_timer,jiffies + msecs_to_jiffies(SENSOR_LED_OFF_DELAY));
		}
		else
		{
			mod_timer(&tmdrv_timer,jiffies + msecs_to_jiffies(SENSOR_DEFAULT_DELAY));
		}
	}
	g_inside_sw = 0;
	mutex_unlock(&data->mutex);
}

static void 
light_timer_func(unsigned long ptr)
{
	LIGHT_DBG("[LIGHT] schedule work is called===\n");
	atomic_set(&g_timer_is_pending,0);
	schedule_work(&g_proxi_work_data);
}

#ifdef LIGHT_EXPORT_ADC_FUNCTION

/*
 * Open routine and exported to global kernel 
 * space for use by other external drivers
 * 
 *
*/
int 
illumi_sensor_open (void)
{
   LIGHT_DBG("[LIGHT] in OPEN called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
   if (0 == g_ls_probe_init)
   {
   	   LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
	   return -1;
   }
   light_open_sensor();
   return 0;
}
EXPORT_SYMBOL(illumi_sensor_open);

/*
 * Close routine and exported to global kernel 
 * space for use by other external drivers
 * 
 *
*/
int 
illumi_sensor_close (void)
{
	if (0 == g_ls_probe_init)
	{
   	    LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
		return -1;
	}
	LIGHT_DBG("\n[LIGHT] in CLOSE called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
	return light_close_sensor();
}
EXPORT_SYMBOL(illumi_sensor_close);

/*
 * Read routine and exported to global kernel 
 * space for use by other external drivers
 * 
 *
*/
int illumi_sensor_read(int *buf)
{
	if (0 == g_ls_probe_init)
	{
   	    LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
		return -1;
	}
	mutex_lock(&data->mutex);
	if (g_ls_op_cnt <= 0)
	{
   	    LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
		return -1;
	}
	mutex_unlock(&data->mutex);
	if ( buf )
	{
	/*	*buf = atomic_read(&g_total_lux_val); */
		*buf = atomic_read(&g_total_adc_val);
	}
	return 0;
}
EXPORT_SYMBOL(illumi_sensor_read);

#endif

#if LIGHT_USE_EARLY_SUSPEND
#ifdef CONFIG_HAS_EARLYSUSPEND
void light_early_suspend(struct early_suspend *h)
{
	light_suspended = 1;
	LIGHT_DBG("[LIGHT] light_early_suspend!!!!\n");
}
void light_late_resume(struct early_suspend *h)
{
	light_suspended = 0;
	light_lux_adjust = 0;
	light_adjust_max = 10;
	LIGHT_DBG("[LIGHT] light_late_resume \n");
}

#endif
#endif
/*
 * open routine for input 
 * susbsystem
 * 
 *
*/
static int 
input_open (struct input_dev *dev)
{
	if (0 == g_ls_probe_init)
	{
   	    LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
		return -1;
	}

	LIGHT_DBG("\n[LIGHT] in OPEN called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
	light_open_sensor();
	return 0;
}

/*
 * close routine for input 
 * susbsystem
 * 
 *
*/
static void 
input_close (struct input_dev *dev)
{
	if (0 == g_ls_probe_init)
	{
   	    LIGHT_DBG("[LIGHT] Failed  [%s]:[%d]===\n", __FUNCTION__,__LINE__);
		return;
	}

	LIGHT_DBG("\n[LIGHT] in CLOSE called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);
	light_close_sensor();
	return;
}


/*
 * sensor probe routine : does all sensor 
 * initialization and input subsystem creation
 * 
 *
*/
static int
sensor_probe(struct platform_device *pdev)
{
	int input_registered = 0;
#ifdef LIGHT_SYSFS	
	int sysfs_created = 0;
#endif
	int rt;

	LIGHT_DBG("\n[LIGHT] in PROBE called  [%s] start, line [%d]===\n", __FUNCTION__,__LINE__);

	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	
	data->enabled = 1;
	data->delay = SENSOR_DEFAULT_DELAY;

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		printk(KERN_ERR
			   "sensor_probe: Failed to allocate input_data device\n");
		goto err;
	}
	data->input_device = input_data;

	set_bit(EV_ABS, input_data->evbit);
	set_bit(ABS_X,     input_data->absbit);
	input_set_abs_params(input_data, ABS_X, -1872, 1872, 0, 0);

#if SENSOR_TYPE <= 4
	input_set_capability(input_data, EV_ABS, ABS_Y);
	input_set_capability(input_data, EV_ABS, ABS_Z);
#endif

	input_data->name = SENSOR_NAME;
	input_data->open = input_open;
	input_data->close = input_close;

	rt = input_register_device(input_data);
	if (rt) {
		printk(KERN_ERR
			   "sensor_probe: Unable to register input_data device: %s\n",
			   input_data->name);
		goto err;
	}
	input_registered = 1;
#ifdef LIGHT_SYSFS
	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		printk(KERN_ERR
			   "sensor_probe: sysfs_create_group failed[%s]\n",
			   input_data->name);
		goto err;
	}
	sysfs_created = 1;
#endif	
	mutex_init(&data->mutex);
	this_data = input_data;
#ifdef LIGHT_USE_I2C_INTERFACE
	/* Get the I2C Adaptor */
	i2c_tmdrv = i2c_get_adapter(0);
#endif
	//initialize work queue
	INIT_WORK(&g_proxi_work_data, light_work_bh);
	setup_timer(&tmdrv_timer, light_timer_func, 0);
	g_ls_probe_init = 1;
#if LIGHT_USE_EARLY_SUSPEND
#ifdef CONFIG_HAS_EARLYSUSPEND
	e_sus_fcn.suspend = light_early_suspend;
	e_sus_fcn.resume = light_late_resume;
	register_early_suspend(&e_sus_fcn);
#endif
#endif

	return 0;

	err:
	if (data != NULL) {
		printk(KERN_CRIT "sensor_probe: failed to initilie the light driver\n");
		if (input_data != NULL) {
#ifdef LIGHT_SYSFS			
			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
						&sensor_attribute_group);
			}
#endif			
			if (input_registered) {
				input_unregister_device(input_data);
			}
			else {
				input_free_device(input_data);
			}
			input_data = NULL;
		}
		kfree(data);
	}

	return rt;
}

static int
sensor_remove(struct platform_device *pdev)
{
    struct sensor_data *data;
	g_ls_probe_init = 0;
    if (this_data != NULL) {
        data = input_get_drvdata(this_data);
#ifdef LIGHT_SYSFS		
        sysfs_remove_group(&this_data->dev.kobj,
                &sensor_attribute_group);
#endif		
        input_unregister_device(this_data);
        if (data != NULL) {
            kfree(data);
        }
    }

    return 0;
}



/*
 * Module init and exit
 */
static struct platform_driver sensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME,
        .owner  = THIS_MODULE,
    },
};

/*
 * sensor module initialization
 * 
 * 
 *
*/
static int __init sensor_init(void)
{
	atomic_set(&g_total_lux_val,LIGHT_DEFAULT_VAL);
	atomic_set(&g_total_adc_val,0);
	atomic_set(&g_led_on_off,0);
	atomic_set(&g_timer_is_pending,0);
	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		return -1;
	} 
	platform_driver_register(&sensor_driver);

	/* for I2C remote mutex */
	rmutex_i2c = (uint32_t*)smem_alloc(SMEM_I2C_MUTEX, 8);
	if (rmutex_i2c != NULL) {
		if (remote_spin_lock_init(&rslock_i2c, SMEM_SPINLOCK_I2C) != 0)
			rmutex_i2c = NULL;
	}

	return 0;
}
module_init(sensor_init);

/*
 * exit sensor module
 * 
 * 
 *
*/
static void __exit sensor_exit(void)
{
    platform_driver_unregister(&sensor_driver);
    platform_device_unregister(sensor_pdev);
}
module_exit(sensor_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");
