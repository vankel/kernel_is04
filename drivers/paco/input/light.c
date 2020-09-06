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
/*
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

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#endif
//Flags for parallel userspace and driver handling

/* for debugging */
#define DEBUG 0

//Light Sensor
#define SENSOR_NAME "light"

//changing as per spec
#define SENSOR_DEFAULT_DELAY            (1000)   /* 1000 ms */
#define SENSOR_LED_OFF_DELAY            (150)   /* 150 ms */
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

static struct input_dev *input_data = NULL;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend  e_sus_fcn;
void light_early_suspend(struct early_suspend *h);
void light_late_resume(struct early_suspend *h);
static int light_suspended = 0;
static int light_lux_adjust = 0;
static int light_adjust_max = 25;
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

static int suspend(void);
static int resume(void);

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
	{0x0,0},
	{0x1,0},
	{0x2,30},
	{0x3,60},
	{0x4,80},
	{0x5,100},
	{0x6,120},
	{0x7,140},
	{0x8,160},
	{0x9,180},
	{0x0A,200},
	{0x0B,220},
	{0x0C,240},
	{0x0D,260},
	{0x0E,280},
	{0x0F,300},
	{0x10,320},
	{0x11,340},
	{0x12,360},
	{0x13,380},
	{0x14,400},
	{0x15,420},
	{0x16,440},
	{0x17,460},
	{0x18,480},
	{0x19,500},
	{0x1A,520},
	{0x1B,540},
	{0x1C,560},
	{0x1D,580},
	{0x1E,600},
	{0x1F,620},
	{0x20,640},
	{0x21,660},
	{0x22,680},
	{0x23,700},
	{0x24,720},
	{0x25,740},
	{0x26,760},
	{0x27,780},
	{0x28,800},
	{0x29,820},
	{0x2A,840},
	{0x2B,860},
	{0x2C,880},
	{0x2D,900},
	{0x2E,920},
	{0x2F,940},
	{0x30,960},
	{0x31,980},
	{0x32,1000},
	{0x33,1019},
	{0x34,1038},
	{0x35,1058},
	{0x36,1077},
	{0x37,1096},
	{0x38,1115},
	{0x39,1135},
	{0x3A,1154},
	{0x3B,1173},
	{0x3C,1192},
	{0x3D,1212},
	{0x3E,1231},
	{0x3F,1250},
	{0x40,1269},
	{0x41,1288},
	{0x42,1308},
	{0x43,1327},
	{0x44,1346},
	{0x45,1365},
	{0x46,1385},
	{0x47,1404},
	{0x48,1423},
	{0x49,1442},
	{0x4A,1462},
	{0x4B,1481},
	{0x4C,1500},
	{0x4D,1519},
	{0x4E,1538},
	{0x4F,1558},
	{0x50,1577},
	{0x51,1596},
	{0x52,1615},
	{0x53,1635},
	{0x54,1654},
	{0x55,1673},
	{0x56,1692},
	{0x57,1712},
	{0x58,1731},
	{0x59,1750},
	{0x5A,1769},
	{0x5B,1788},
	{0x5C,1808},
	{0x5D,1827},
	{0x5E,1846},
	{0x5F,1865},
	{0x60,1885},
	{0x61,1904},
	{0x62,1923},
	{0x63,1942},
	{0x64,1962},
	{0x65,1981},
	{0x66,2000},
	{0x67,2019},
	{0x68,2038},
	{0x69,2058},
	{0x6A,2077},
	{0x6B,2096},
	{0x6C,2115},
	{0x6D,2135},
	{0x6E,2154},
	{0x6F,2173},
	{0x70,2192},
	{0x71,2212},
	{0x72,2231},
	{0x73,2250},
	{0x74,2269},
	{0x75,2288},
	{0x76,2308},
	{0x77,2327},
	{0x78,2346},
	{0x79,2365},
	{0x7A,2385},
	{0x7B,2404},
	{0x7C,2423},
	{0x7D,2442},
	{0x7E,2462},
	{0x7F,2481},
	{0x80,2500},
	{0x81,2519},
	{0x82,2538},
	{0x83,2558},
	{0x84,2577},
	{0x85,2596},
	{0x86,2615},
	{0x87,2635},
	{0x88,2654},
	{0x89,2673},
	{0x8A,2692},
	{0x8B,2712},
	{0x8C,2731},
	{0x8D,2750},
	{0x8E,2769},
	{0x8F,2788},
	{0x90,2808},
	{0x91,2827},
	{0x92,2846},
	{0x93,2865},
	{0x94,2885},
	{0x95,2904},
	{0x96,2923},
	{0x97,2942},
	{0x98,2962},
	{0x99,2981},
	{0x9A,3000},
	{0x9B,3023},
	{0x9C,3047},
	{0x9D,3070},
	{0x9E,3093},
	{0x9F,3116},
	{0xA0,3140},
	{0xA1,3163},
	{0xA2,3186},
	{0xA3,3209},
	{0xA4,3233},
	{0xA5,3256},
	{0xA6,3279},
	{0xA7,3302},
	{0xA8,3326},
	{0xA9,3349},
	{0xAA,3372},
	{0xAB,3395},
	{0xAC,3419},
	{0xAD,3442},
	{0xAE,3465},
	{0xAF,3488},
	{0xB0,3512},
	{0xB1,3535},
	{0xB2,3558},
	{0xB3,3581},
	{0xB4,3605},
	{0xB5,3628},
	{0xB6,3651},
	{0xB7,3674},
	{0xB8,3698},
	{0xB9,3721},
	{0xBA,3744},
	{0xBB,3767},
	{0xBC,3791},
	{0xBD,3814},
	{0xBE,3837},
	{0xBF,3860},
	{0xC0,3884},
	{0xC1,3907},
	{0xC2,3930},
	{0xC3,3953},
	{0xC4,3977},
	{0xC5,4000},
	{0xC6,4040},
	{0xC7,4080},
	{0xC8,4120},
	{0xC9,4160},
	{0xCA,4200},
	{0xCB,4240},
	{0xCC,4280},
	{0xCD,4320},
	{0xCE,4360},
	{0xCF,4400},
	{0xD0,4440},
	{0xD1,4480},
	{0xD2,4520},
	{0xD3,4560},
	{0xD4,4600},
	{0xD5,4640},
	{0xD6,4680},
	{0xD7,4720},
	{0xD8,4760},
	{0xD9,4800},
	{0xDA,4840},
	{0xDB,4880},
	{0xDC,4920},
	{0xDD,4960},
	{0xDE,5000},
	{0xDF,6000},
	{0xE0,7000},

};

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;
static struct sensor_data *data = NULL;

static int
suspend(void)
{
    /* implement suspend of the sensor */
//    printk(KERN_DEBUG "%s: suspend\n", SENSOR_NAME);

//    if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
        /* suspend gyroscope */
//    }
//    else if (strcmp(SENSOR_NAME, "light") == 0) {
        /* suspend light */
//    }
//    else if (strcmp(SENSOR_NAME, "pressure") == 0) {
        /* suspend pressure */
//    }
//    else if (strcmp(SENSOR_NAME, "temperature") == 0) {
        /* suspend temperature */
//    }
//    else if (strcmp(SENSOR_NAME, "proximity") == 0) {
        /* suspend proximity */
//    }

#if DEBUG
//    {
//        struct sensor_data *data = input_get_drvdata(this_data);
//        data->suspend = 1;
//    }
#endif /* DEBUG */

	return 0;
}

static int
resume(void)
{
    /* implement resume of the sensor */
//    printk(KERN_DEBUG "%s: resume\n", SENSOR_NAME);

//    if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
        /* resume gyroscope */
//    }
//    else if (strcmp(SENSOR_NAME, "light") == 0) {
        /* resume light */
//    }
//    else if (strcmp(SENSOR_NAME, "pressure") == 0) {
        /* resume pressure */
//    }
//    else if (strcmp(SENSOR_NAME, "temperature") == 0) {
        /* resume temperature */
//    }
//    else if (strcmp(SENSOR_NAME, "proximity") == 0) {
        /* resume proximity */
//    }

#if DEBUG
//    {
//        struct sensor_data *data = input_get_drvdata(this_data);
//        data->suspend = 0;
//    }
#endif /* DEBUG */

    return 0;
}

/*
 *	Getting the LUX Value  from this Table. 
 *	Since the sensor control specification has some definite mapping 
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
    data1 = 5; //adc_read_lx
    data2 = 0;

    /* waiting for I2C remote mutex */
    if(rmutex_i2c != NULL) {
        i2c_rspin_lock(rmutex_i2c);
        i2c_rspin_unlock(rmutex_i2c);
    }

    ret = msm_proc_comm(PCOM_OEM_008, &data1, &data2);
    //LIGHT_DBG("[nak]get_proc_comm_data() - adc qsd %02x, ret=%d\n", data2,ret);
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

    if (value < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < value) {
        value = SENSOR_MAX_DELAY;
    }

    mutex_lock(&data->mutex);

    data->delay = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT, (data->enabled<<16) | value);

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

    if (value != 0 && value != 1) {
        return count;
    }

    mutex_lock(&data->mutex);

    if (data->enabled && !value) {
        suspend();
    }
    if (!data->enabled && value) {
        resume();
    }
    data->enabled = value;

    input_report_abs(input_data, ABS_CONTROL_REPORT, (value<<16) | data->delay);

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
    int x;

    spin_lock_irqsave(&input_data->event_lock, flags);

    x = input_data->abs[ABS_X];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", x);
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

static ssize_t 
sensor_debug_suspend_show(struct device *dev,
                          struct device_attribute *attr,
						  char *buf)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
    return sprintf(buf, "%d\n", light_suspended);
#else
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);

    return sprintf(buf, "%d\n", data->suspend);
#endif
}

static ssize_t 
sensor_debug_suspend_store(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);

    if (value) {
#ifdef CONFIG_HAS_EARLYSUSPEND
        light_early_suspend( NULL ); 	/* Pass dummy struct addr */
#else
        suspend();
#endif
    } else {
#ifdef CONFIG_HAS_EARLYSUSPEND
        light_late_resume( NULL );	/* Pass dummy struct addr */
#else
        resume();
#endif
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
//    struct sensor_data *data = input_get_drvdata(this_data);
    int rt = 0;

//    mutex_lock(&data->mutex);

//    if (data->enabled) {
//        rt = suspend();
//    }

//    mutex_unlock(&data->mutex);

    return rt;
}

static int
sensor_resume(struct platform_device *pdev)
{
//    struct sensor_data *data = input_get_drvdata(this_data);
    int rt = 0;

//    mutex_lock(&data->mutex);

//    if (data->enabled) {
//        rt = resume();
//    }

//    mutex_unlock(&data->mutex);

    return rt;
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
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (0 == light_suspended) 
#endif
	{
		if(!atomic_read(&g_led_on_off)) //LED is OFF, read value from ADC
		{
			for(i = 0; i < NUM_AVG; i++)
			{
				data2  = get_proc_comm_data();
				conv_lux = get_lux_from_adc(data2);
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
	//initialize work queue
	INIT_WORK(&g_proxi_work_data, light_work_bh);
	setup_timer(&tmdrv_timer, light_timer_func, 0);
	g_ls_probe_init = 1;
#ifdef CONFIG_HAS_EARLYSUSPEND
	e_sus_fcn.suspend = light_early_suspend;
	e_sus_fcn.resume = light_late_resume;
	register_early_suspend(&e_sus_fcn);
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
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.2.0");
