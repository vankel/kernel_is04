/*
  Proximty Sensor Driver
  Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA */


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/kthread.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include "proximity.h"
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/i2c.h>
#include <mach/pmic.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <sub_pmic.h>

/* for fops dev */
#define PROX_DEV 0

/* for debugging */
#define DEBUG 0

#if DEBUG
#define PROX_DBG(x...)  printk(x)
#else
#define PROX_DBG(x...)  
#endif

#define PROXIMITY_SYSFS 1

#define SENSOR_TYPE (8)

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

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)




/*===========================================================================

  GLOBAL VARIABLES

  ===========================================================================*/
#define PROX_SLAVE_ADDRESS      (0xC8 >> 1)

#define READ_SIZE				2


#define PRODRV_GPIO_N        94


#define	INT_CLEAR				0x80


#define	PROX_REG				0x00
#define	PROX_1					0x01
#define	PROX_VO_DET				0x01


#define	GAIN_REG				0x01
#define	GAIN_REG_INIT			0x00
#define	GAIN_LED				0x08


#define	HYS_REG					0x02
#define HYS_REG_INIT			0x00
#define HYS_HYSF				0x01
#define HYS_HYSF_1				0x02
#define HYS_HYSF_2				0x04
#define HYS_HYSF_3				0x08

#define	HYS_HYSC				0x20
#define	HYS_HYSC_1				0x40
//enabled to check if the spec is correct
//#define	HYS_HYSC_1				0x20
#define	HYS_HYSD				0x80


#define	CYCLE_REG				0x03
#define	CYCLE_REG_INIT			0x00
#define	CYCLE_OSC				0x04
#define	CYCLE_CYCL				0x08
#define	CYCLE_CYCL_1			0x10
#define	CYCLE_CYCL_2			0x20
#define	CYCLE_256MS				0x2C    /*K01_PROX_001*/



#define	OPMOD_REG				0x04
#define	OPMOD_REG_INIT			0x00
#define	OPMOD_SSD				0x01
#define	OPMOD_VCON				0x02
#define	OPMOD_ASD				0x10

#define	CON_REG					0x06
#define	CON_REG_INIT			0x00
#define	CON_OCON				0x08
#define	CON_OCON_1				0x10

#define	CON_REG_INT_CLEAR		0x86	/*10000110 --- top 1 bit is for intclear and last 3 bits are for register */


#define	PROX_ERROR_SUCCESS				0x00
extern int msm_prox_read_nvitem(unsigned int id);
struct i2c_adapter *i2c_proximity;


atomic_t g_wake;
atomic_t g_suspend_off;

///////////////////////////////////////////
struct sensor_data {
	struct mutex mutex;
	int enabled;
	int delay;
#if DEBUG
	int suspend;
#endif
};

static atomic_t prox_state; //0= WAKEUP 1= SUSPEND 2 = Wait i2c wakeup


/*===========================================================================

  LOCAL VARIABLES

  ===========================================================================*/

static int proximity_major = 0;
module_param(proximity_major, int, 0);
#if PROX_DEV
static struct class* proximity_class;
#endif

static unsigned int g_proxi_irq;
static struct work_struct g_proxi_work_data;
static unsigned char g_prox_sense_a;
static unsigned char g_prox_sense_b;

#define  NV_TSB_TOP_ITEMS_I  10000
#define  NV_PROX_SENSE_A_I   (NV_TSB_TOP_ITEMS_I+106)
#define  NV_PROX_SENSE_B_I   (NV_TSB_TOP_ITEMS_I+107)

#define PROX_SENS_MAX_NUM 31
#define PROX_SENS_A_DEFAULT 12
#define PROX_SENS_B_DEFAULT 21
#define PROX_TEST_SENS_NUM 32
unsigned char prox_sensitivity_table[PROX_TEST_SENS_NUM] = {
	0x04,	/*  0 :0.25 */
	0x05,	/*  1 :0.31 */
	0x06,	/*  2 :0.38 */
	0x07,	/*  3 :0.44 */
	0x00,	/*  4 :0.50 */
	0x01,	/*  5 :0.56 */
	0x25,	/*  6 :0.63 */
	0x0D,	/*  7 :0.69 */
	0x26,	/*  8 :0.75 */
	0x0F,	/*  9 :0.81 */
	0x27,	/*  10:0.88 */
	0x09,	/*  11:0.94 */
	0x20,	/*  12:1.00 */
	0x0B,	/*  13:1.06 */
	0x21,	/*  14:1.13 */
	0x45,	/*  15:1.25 */
	0x2D,	/*  16:1.38 */
	0x46,	/*  17:1.50 */
	0x2F,	/*  18:1.63 */
	0x47,	/*  19:1.75 */
	0x29,	/*  20:1.88 */
	0x40,	/*  21:2.00 */
	0x2B,	/*  22:2.13 */
	0x41,	/*  23:2.25 */
	0x4C,	/*  24:2.50 */
	0x4D,	/*  25:2.75 */
	0x4E,	/*  26:3.00 */
	0x4F,	/*  27:3.25 */
	0x48,	/*  28:3.50 */
	0x49,	/*  29:3.75 */
	0x4A,	/*  30:4.00 */
	0x4B,	/*  31:4.25 */
};
#define PROX_SENS_DETECT_OFFSET 7
#define PROX_SENS_A_MAX_NUM 23
unsigned char prox_sens_a_b_table[PROX_SENS_A_MAX_NUM] = {
/*     a            b   */
	/* 0 :0.25 */	4,	/* 0.50 */
	/* 1 :0.31 */	6,	/* 0.63 */
	/* 2 :0.38 */	8,	/* 0.75 */
	/* 3 :0.44 */	10,	/* 0.88 */
	/* 4 :0.50 */	12,	/* 1.00 */
	/* 5 :0.56 */	14,	/* 1.13 */
	/* 6 :0.63 */	15,	/* 1.25 */
	/* 7 :0.69 */	16,	/* 1.38 */
	/* 8 :0.75 */	17,	/* 1.50 */
	/* 9 :0.81 */	18,	/* 1.63 */
	/* 10:0.88 */	19,	/* 1.75 */
	/* 11:0.94 */	20,	/* 1.88 */
	/* 12:1.00 */	21,	/* 2.00 */
	/* 13:1.06 */	22,	/* 2.13 */
	/* 14:1.13 */	23,	/* 2.25 */
	/* 15:1.25 */	24,	/* 2.50 */
	/* 16:1.38 */	25,	/* 2.75 */
	/* 17:1.50 */	26,	/* 3.00 */
	/* 18:1.63 */	27,	/* 3.25 */
	/* 19:1.75 */	28,	/* 3.50 */
	/* 20:1.88 */	29,	/* 3.75 */
	/* 21:2.00 */	30,	/* 4.00 */
	/* 22:2.13 */	31,	/* 4.25 */
};
/*===========================================================================

  LOCAL FUNCTION PROTOTYPES

  ===========================================================================*/
static int prodrv_sns_reg_init(void);

static int prodrv_sns_init(void);
static int  prodrv_sns_i2c_write( unsigned char reg, unsigned char data);
static int prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len);
static void prodrv_sns_ssd(void);
int prodrv_sns_ON(void);
void prodrv_sns_OFF(void);
static int prox_request_irqs(void);

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;
static struct input_dev *input_data = NULL;

#ifdef PROXIMITY_SYSFS
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
	int value = simple_strtoul(buf, NULL, 10);
	int delay;
        PROX_DBG("\n<PROX> sensor_enable_store called, value [%d]====\n",value);

	if (value != 0 && value != 1) {
		return count;
	}

        if(value) /*if it has been set to 1, call is incoming*/
        atomic_set(&g_wake,1);
        else
        atomic_set(&g_wake,0);

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
		struct device_attribute *attr, char *buf)
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
static DEVICE_ATTR(enable, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP,
		sensor_enable_show, sensor_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		NULL, sensor_wake_store);
static DEVICE_ATTR(data, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP, sensor_data_show, NULL);
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
	PROX_DBG("%s: sensor_suspend\n", SENSOR_NAME);
	/*if call is not incoming, go to suspend */
	if(!atomic_read(&g_wake))
	{
	PROX_DBG("\n====<PROX> no incoming call\n");
		prodrv_sns_OFF();
		atomic_set(&g_suspend_off,1);
	}



	if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
		/* suspend gyroscope */
	}
	else if (strcmp(SENSOR_NAME, "light") == 0) {
		/* suspend light */
	}
	else if (strcmp(SENSOR_NAME, "pressure") == 0) {
		/* suspend pressure */
	}
	else if (strcmp(SENSOR_NAME, "temperature") == 0) {
		/* suspend temperature */
	}
	else if (strcmp(SENSOR_NAME, "proximity") == 0) {
		atomic_set(&prox_state,1);
		/* suspend proximity */
	}

#if DEBUG
	{
		struct sensor_data *data = input_get_drvdata(this_data);
		data->suspend = 1;
	}
#endif /* DEBUG */

	return 0;
}

static int
sensor_resume(struct platform_device *pdev)
{
	/* implement resume of the sensor */
	PROX_DBG("%s: sensor_resume\n", SENSOR_NAME);
if(atomic_read(&g_suspend_off))
	{
		prodrv_sns_ON();
		atomic_set(&g_suspend_off,0);
	}
	//interrupt clearing
	prodrv_sns_i2c_write( CON_REG | INT_CLEAR/*CON_REG_INT_CLEAR*/, CON_REG_INIT);
	if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
		/* resume gyroscope */
	}
	else if (strcmp(SENSOR_NAME, "light") == 0) {
		/* resume light */
	}
	else if (strcmp(SENSOR_NAME, "pressure") == 0) {
		/* resume pressure */
	}
	else if (strcmp(SENSOR_NAME, "temperature") == 0) {
		/* resume temperature */
	}
	else if (strcmp(SENSOR_NAME, "proximity") == 0) {
		/* resume proximity */

		if(atomic_read(&prox_state) == 2) //wait i2c wakeup
        {
			schedule_work(&g_proxi_work_data);
		}
		else
		{
			atomic_set(&prox_state,0);
		}
	}

#if DEBUG
	{
		struct sensor_data *data = input_get_drvdata(this_data);
		data->suspend = 0;
	}
#endif /* DEBUG */

	return 0;
}

static int
sensor_probe(struct platform_device *pdev)
{
	struct sensor_data *data = NULL;
	int input_registered = 0;
#ifdef PROXIMITY_SYSFS		
	int  sysfs_created = 0;
#endif
	int rt;

	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	data->enabled = 0;
	data->delay = SENSOR_DEFAULT_DELAY;

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		PROX_DBG(KERN_ERR
				"sensor_probe: Failed to allocate input_data device\n");
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);
#if SENSOR_TYPE <= 4
	input_set_capability(input_data, EV_ABS, ABS_Y);
	input_set_capability(input_data, EV_ABS, ABS_Z);
#endif
	input_data->name = SENSOR_NAME;
	rt = input_register_device(input_data);
	if (rt) {
		PROX_DBG(KERN_ERR
				"sensor_probe: Unable to register input_data device: %s\n",
				input_data->name);
		goto err;
	}
	input_registered = 1;
#ifdef PROXIMITY_SYSFS
	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		PROX_DBG(KERN_ERR
				"sensor_probe: sysfs_create_group failed[%s]\n",
				input_data->name);
		goto err;
	}
	sysfs_created = 1;
#endif	
	mutex_init(&data->mutex);
	this_data = input_data;
	input_report_abs(input_data, ABS_X, 1);
	input_sync(input_data);    
	prox_request_irqs();
	return 0;

err:
	if (data != NULL) {
		PROX_DBG(KERN_ERR
				"sensor_probe: Error input_data = NULL");
		if (input_data != NULL) {
#ifdef PROXIMITY_SYSFS			
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

	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
#ifdef PROXIMITY_SYSFS		
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

#if PROX_DEV

/*fops open to enable access via /dev/proximity*/
int
proximity_open(struct inode* inode,struct file* file)
{
	PROX_DBG("proximity: proximity_open called\n");

	return 0;
}

/*fops read to enable access via /dev/proximity*/
static int
proximity_read(struct file * file, char * buff, size_t count, loff_t *pos)
{
	unsigned data1=0;

	data1 = gpio_get_value(37);
	buff[0] = data1;
	PROX_DBG(KERN_CRIT "\n[PROXIMITY] in proximity_read--value read is [0x%x]===\n", data1);
	return 0;
}

/*fops release to enable access via /dev/proximity*/
int
proximity_release(struct inode *inode, struct file *file)
{
	PROX_DBG("proximity: proximity_release called\n");
	return 0;
}

/*fops ioctl to enable access via /dev/proximity*/
static long
proximity_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	switch (cmd)
	{
		case IOCTL_PRODRV_ON:
			prodrv_sns_ON();
			break;

		case IOCTL_PRODRV_OFF:
			prodrv_sns_OFF();
			break;

		default:
			break;
	}

	return 0;
}

static struct file_operations proximity_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = proximity_read,
	//  .write = proximity_write,
	.open = proximity_open,
	.unlocked_ioctl = proximity_ioctl,
	.release = proximity_release,
};
#endif

/*powerup the PMIC, configure GPIO 37,
set GPIO to NO_PULL, make it as input pin.
Give sleep of 10 millisecs, call registry init
*/

int
prodrv_sns_ON(void)
{
	int ok_flag = TRUE;
	
	PROX_DBG("\n<PROX> prodrv_sns_ON called====\n");
	sub_pmic_ldo_ctrl(SUB_PMIC_LDO4,SUB_PMIC_LDO_ON);
	
	gpio_configure(37,  0x40000000 | GPIOF_INPUT);
	//added to set it to no pull as per spec
	gpio_tlmm_config(GPIO_CFG(37, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);

	msleep(10);

	ok_flag = prodrv_sns_init();
	if(ok_flag!=TRUE)
	{
		gpio_configure(37, GPIOF_DRIVE_OUTPUT);
		//sub_pmic_ldo_ctrl(SUB_PMIC_LDO4,SUB_PMIC_LDO_OFF);

	}
	return ok_flag;
}


/*power-down the device
configure GPIO 37 to out*/

void
prodrv_sns_OFF(void)
{
	PROX_DBG("\n<PROX> prodrv_sns_OFF called====\n");
	gpio_configure(37, GPIOF_DRIVE_OUTPUT);
	gpio_tlmm_config(GPIO_CFG(37, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_DISABLE);
	prodrv_sns_ssd();/*K01_PROX_002*/

	sub_pmic_ldo_ctrl(SUB_PMIC_LDO4,SUB_PMIC_LDO_OFF);

}

/*Initialize all registry values of the proximity
sensor as per specifications, use i2c write to write
register values*/

static int
prodrv_sns_reg_init(void)
{
	int        init_sts;
	init_sts = prodrv_sns_i2c_write( GAIN_REG, GAIN_LED );
	PROX_DBG("prodrv_sns_reg_init init_sts1 = %d\n", init_sts);
	init_sts &= prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
	PROX_DBG("prodrv_sns_reg_init init_sts2 = %d\n", init_sts);
	//enabled as per spec
	init_sts &= prodrv_sns_i2c_write( CYCLE_REG, CYCLE_OSC);
	//	init_sts &= prodrv_sns_i2c_write( CYCLE_REG, CYCLE_256MS);
	PROX_DBG("prodrv_sns_reg_init init_sts3 = %d\n", init_sts);
	init_sts &= prodrv_sns_i2c_write( OPMOD_REG, OPMOD_SSD | OPMOD_VCON);
	PROX_DBG("prodrv_sns_reg_init init_sts4 = %d\n", init_sts);

	//enabling intr as per spec
	init_sts &= prodrv_sns_i2c_write( CON_REG, CON_REG_INIT);


	return init_sts;

}

/*wrapper for the register initialization*/

static int
prodrv_sns_init(void)
{
	int prodrv_init_st;
	prodrv_init_st = prodrv_sns_reg_init();
	if(prodrv_init_st < 0)
	{
		PROX_DBG("prodrv_sns_reg_init falied!\n");
		return FALSE;
	}
	return TRUE;
}



/*change the register values so that it is ok to
shutdown*/

static void
prodrv_sns_ssd(void)
{
	int sts = FALSE;
	sts = prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
	sts &= prodrv_sns_i2c_write( OPMOD_REG, OPMOD_VCON);
	sts &= prodrv_sns_i2c_write( CON_REG, CON_OCON | CON_OCON_1);
	if(sts != TRUE)
	{
		PROX_DBG("Prodrv SSD SET ERROR !!\n");
	}
}


/*does i2c write to the proximity sensor*/
static int
prodrv_sns_i2c_write(unsigned char reg, unsigned char data)
{
	struct i2c_msg msg;
	u_int8_t buf[64];
	int ret = 0;

	msg.addr = PROX_SLAVE_ADDRESS;
	msg.flags = 0;
	buf[0] = reg;
	buf[1] = data;

	msg.buf  = buf;
	msg.len  = 2;
	ret = i2c_transfer(i2c_proximity, &msg, 1);
	if (ret < 0)
	{
		PROX_DBG("[proximity] prodrv_sns_i2c_write -> i2c_transfer() falied!(%d) in %s()\n",ret , __func__);
	}
	else
	{
		PROX_DBG("[proximity] prodrv_sns_i2c_write -> i2c_transfer() SUCCESS!(%d) in %s()\n",ret , __func__);
	}

	return ret;
}



/*does i2c read from the proximity sensor*/
static int
prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len)
{
	struct i2c_msg msg[2];
	u_int8_t msgbuf[2];
	int ret = 0;


	memcpy(msgbuf, &reg, sizeof(reg));

	msg[0].addr  = PROX_SLAVE_ADDRESS;
	msg[0].flags = 0;
	msg[0].buf   = msgbuf;
	msg[0].len   = 1;

	msg[1].addr  = PROX_SLAVE_ADDRESS;
	msg[1].flags = I2C_M_RD;
	msg[1].buf   = data;
	msg[1].len   = len;

	ret = i2c_transfer(i2c_proximity, msg, 2);
	if (ret != 2)
	{
		PROX_DBG("[proximity]i2c_transfer() falied!(%d) in %s()\n",ret , __func__);
	}
	else
	{
		PROX_DBG("[proximity]i2c_transfer() SUCCESS!(%d) in %s()\n",ret , __func__);
	}

	return ret;
}


#if PROX_DEV
/*
 * Set up the cdev structure for a device.
 */
static void
proximity_setup_cdev(struct cdev *dev, int minor,
		struct file_operations *fops)
{
	int err, devno = MKDEV(proximity_major, minor);

	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add (dev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		PROX_DBG (KERN_NOTICE "Error %d adding rfs%d", err, minor);

	if (IS_ERR(device_create(proximity_class, NULL, devno, NULL, "proximity")))
		PROX_DBG(KERN_ERR "can't create device\n");
}
#endif

//bottom half of interrupt handling
static void
proxi_work_bh(struct work_struct *work)
{
	int init_sts;

	unsigned char data1[2];
	unsigned char proxy_val = 0;

  if(atomic_read(&prox_state) != 1)
  {
	PROX_DBG("\n<PROX>====proxi_work_bh != 1===\n");

	prodrv_sns_i2c_read(PROX_REG, data1, 2);
	if(data1[1] & 1)
	{
		//changing as per new spec change

		//init_sts = prodrv_sns_i2c_write( GAIN_REG, GAIN_LED );
		//check if it is 01 or 10 as per spec
		//init_sts &= prodrv_sns_i2c_write( HYS_REG, HYS_HYSC_1);
		init_sts = prodrv_sns_i2c_write( HYS_REG, g_prox_sense_a);
		proxy_val = 0;

	}
	else
	{
		init_sts = prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
		proxy_val = 1;
	}

	init_sts = prodrv_sns_i2c_write( CON_REG, CON_OCON | CON_OCON_1);

	//interrupt clearing
	init_sts = prodrv_sns_i2c_write( CON_REG | INT_CLEAR/*CON_REG_INT_CLEAR*/, CON_REG_INIT);

	enable_irq(g_proxi_irq);

	input_report_abs(input_data, ABS_X, proxy_val);
	input_sync(input_data);    

  }
  else
  {
	PROX_DBG("\n<PROX>====proxi_work_bh = 2 ===\n");
	atomic_set(&prox_state,2);
  }

}

//irq handler
static irqreturn_t
prox_irq_handler(int irq, void *p)
{
PROX_DBG("\n<PROX>===prox_irq_handler==\n");
	disable_irq(g_proxi_irq);
	schedule_work(&g_proxi_work_data);
	return IRQ_HANDLED;
}


//irq request
static int
prox_request_irqs(void)
{
	int err;
	unsigned long req_flags = /*IRQF_TRIGGER_RISING | */IRQF_TRIGGER_FALLING;

	PROX_DBG(KERN_CRIT "\n=======<PROX> prox_request_irqs called=======\n");
	err = g_proxi_irq = gpio_to_irq(37);
	if (err < 0)
		PROX_DBG(KERN_CRIT "gpio_to_irq() FAILED");
	err = request_irq(g_proxi_irq, prox_irq_handler,
			req_flags, "gpio_keys", NULL);
	if (err) {
		PROX_DBG(KERN_CRIT "===<PROX>==gpio_event_input_request_irqs: request_irq failed for input 37, irq %d\n",g_proxi_irq);
		return err;
	}
	return 0;

}







#if PROX_DEV
#define MAX_proximity_DEV 1
static struct cdev proximityDevs[MAX_proximity_DEV];
#endif

/*does sensor initialization*/
static int __init sensor_init(void)
{
	int rc;
	signed char level_a;
	signed char level_b;
#if PROX_DEV
	int result = 0;
	dev_t dev = MKDEV(proximity_major, 0);


	proximity_class = class_create(THIS_MODULE, "proximity_dev");
	if(IS_ERR(proximity_class))
	{
		return PTR_ERR(proximity_class);
	}

	/* Figure out our device number. */
	if (proximity_major)
	{
		result = register_chrdev_region(dev, 2, "proximity_dev");
	}
	else
	{
		result = alloc_chrdev_region(&dev, 0, 2, "proximity_dev");
		proximity_major = MAJOR(dev);
	}
	if (result < 0) {
		PROX_DBG(KERN_WARNING "proximity: unable to get major %d\n", proximity_major);

		return result;
	}
	if (proximity_major == 0)
		proximity_major = result;
#endif

	i2c_proximity = i2c_get_adapter(0);

	INIT_WORK(&g_proxi_work_data, proxi_work_bh);
	atomic_set(&g_wake,0);
	atomic_set(&g_suspend_off,0);
	atomic_set(&prox_state,0);

#if PROX_DEV
	/* Now set up two cdevis. */
	proximity_setup_cdev(proximityDevs, 0, &proximity_fops);
#endif
	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		return -1;
	}
    g_prox_sense_a = prox_sensitivity_table[PROX_SENS_A_DEFAULT];
    g_prox_sense_b = prox_sensitivity_table[PROX_SENS_B_DEFAULT];
    
    level_a = msm_prox_read_nvitem(NV_PROX_SENSE_A_I);
    if((level_a >= 0) && (level_a < PROX_SENS_MAX_NUM))
    {
        level_b = msm_prox_read_nvitem(NV_PROX_SENSE_B_I);
        if((level_b >= 0 ) && (level_b < PROX_SENS_MAX_NUM) && (level_a < level_b))
        {
            /* NV value access success. */
            g_prox_sense_a = prox_sensitivity_table[level_a];
            g_prox_sense_b = prox_sensitivity_table[level_b];
            PROX_DBG(KERN_INFO "proximity: sense_a = %d:sense_b = %d\n", g_prox_sense_a,g_prox_sense_b);
        }
    }

	prodrv_sns_ON();
	return platform_driver_register(&sensor_driver);
}
module_init(sensor_init);

/*does sensor shurdown*/
static void __exit sensor_exit(void)
{
	prodrv_sns_OFF();
	platform_driver_unregister(&sensor_driver);
	platform_device_unregister(sensor_pdev);
}
module_exit(sensor_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");
