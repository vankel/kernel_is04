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
 * MA  02110-1301, USA. */

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


/*===========================================================================

  MACROS

  ===========================================================================*/
/* for debugging */
#define DEBUG 0

#if DEBUG
#define PROX_DBG(x...)  printk(x)
#else
#define PROX_DBG(x...)  
#endif

/* for GPIO initialize */
#define SENSOR_NAME "proximity"
#define SENSOR_DEFAULT_DELAY	(200)   /* 200 ms */
#define SENSOR_MAX_DELAY		(2000)  /* 2000 ms */
#define ABS_STATUS				(ABS_BRAKE)
#define ABS_WAKE				(ABS_MISC)
#define ABS_CONTROL_REPORT		(ABS_THROTTLE)
#define PROX_GPIO				37
#define WAKE_ON					1

/* for I2C */
#define SNS_STANDBY_TIME		10

#define I2C_WRITE_BUFSIZE		64
#define READ_SIZE				2

#define I2C_TRANS_WRITE			1
#define I2C_TARNS_READ			2

#define I2C_READ_POS			1

#define PROX_SLAVE_ADDRESS	  	(0xC8 >> 1)

#define GPIOF_ENABLE_WAKE		0x40000000

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
#define	HYS_HYSD				0x80

#define	CYCLE_REG				0x03
#define	CYCLE_REG_INIT			0x00
#define	CYCLE_OSC				0x04
#define	CYCLE_CYCL				0x08
#define	CYCLE_CYCL_1			0x10
#define	CYCLE_CYCL_2			0x20
#define	CYCLE_256MS				0x2C	/*K01_PROX_001*/

#define	OPMOD_REG				0x04
#define	OPMOD_REG_INIT			0x00
#define	OPMOD_SSD				0x01
#define	OPMOD_VCON				0x02
#define	OPMOD_ASD				0x10

#define	CON_REG					0x06
#define	CON_REG_INIT			0x00
#define	CON_OCON				0x08
#define	CON_OCON_1				0x10

#define PROX_REG_NEAR			1
#define PROX_REG_FAR			0

/* for notification */
#define PROX_VAL_NEAR			0
#define PROX_VAL_FAR			1

/* for sensor status */
#define PROX_STATE_WAKE			0
#define PROX_STATE_SUSPEND		1
#define PROX_STATE_WAIT_I2C		2

/* for sensor settings */
#define NV_OEM_TOP_ITEMS_I		10000
#define NV_PROX_SENSE_A_I		(NV_OEM_TOP_ITEMS_I+106)
#define NV_PROX_SENSE_B_I		(NV_OEM_TOP_ITEMS_I+107)

#define PROX_SENS_MAX_NUM		31
#define PROX_SENS_A_DEFAULT		12
#define PROX_SENS_B_DEFAULT		21
#define PROX_TEST_SENS_NUM		32

/* for sensor delay */
#define PROX_DETECT_NON			0
#define PROX_DETECTING			1
#define PROX_DETECTED			2

/*===========================================================================

  GLOBAL VARIABLES

  ===========================================================================*/

extern int msm_prox_read_nvitem(unsigned int id);
struct i2c_adapter *i2c_proximity;

static atomic_t g_prox_state;
static atomic_t g_wake;
static atomic_t g_suspend_off;

static unsigned int g_proxi_irq;
static struct work_struct g_proxi_work_data;
static unsigned char g_prox_sense_a;
static unsigned char g_prox_sense_b;

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

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

static int prox_wait = 150;

/*===========================================================================

  STRUCTS

  ===========================================================================*/
struct sensor_data {
	struct mutex mutex;
	int enabled;
	int delay;
};

struct prox_detect_state {
	unsigned char proxy_val;
	unsigned char proxy_state;
};
static struct prox_detect_state prox_detect;
static struct timer_list prox_timer;

/*===========================================================================

  LOCAL FUNCTION PROTOTYPES

  ===========================================================================*/
static int prodrv_sns_reg_init(void);
static int  prodrv_sns_i2c_write( unsigned char reg, unsigned char data);
static int prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len);
static void prodrv_sns_ssd(void);
int prodrv_sns_ON(void);
void prodrv_sns_OFF(void);
static int prodrv_sns_request_irqs(void);

/*===========================================================================

  LOCAL FUNCTIONS

  ===========================================================================*/
/* Sysfs interface */
static ssize_t
prodrv_sysfs_delay_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int delay;
	
	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_show=====\n");
	
	if(NULL == data) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_show NULL=====\n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	delay = data->delay;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", delay);
}

static ssize_t
prodrv_sysfs_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int value = simple_strtoul(buf, NULL, 10);
	int enabled;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_store=====\n");

	if(NULL == data) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_delay_store NULL=====\n");
		return 0;
	}
	
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
prodrv_sysfs_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int enabled;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_show=====\n");
	
	if(NULL == data) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_show NULL=====\n");
		return 0;
	}
	
	mutex_lock(&data->mutex);

	enabled = data->enabled;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t
prodrv_sysfs_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_store=====\n");
	
	if ((value != 0) && (value != 1)) {
		PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_store return err=====\n");
		return count;
	}

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_enable_store value:%d=====\n", value);
	if(value) {
		/*if it has been set to 1, call is incoming*/
		atomic_set(&g_wake,TRUE);
	}
	else {
		atomic_set(&g_wake,FALSE);
	}
		
	return count;
}

static ssize_t
prodrv_sysfs_wake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_wake_store=====\n");

	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

static ssize_t
prodrv_sysfs_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int x;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_data_show=====\n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

	x = input_data->abs[ABS_X];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_data_show:%d=====\n",x);
	
	return sprintf(buf, "%d\n", x);
}

static ssize_t
prodrv_sysfs_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	unsigned long flags;
	int status;

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_status_show=====\n");
	
	spin_lock_irqsave(&input_data->event_lock, flags);

	status = input_data->abs[ABS_STATUS];

	spin_unlock_irqrestore(&input_data->event_lock, flags);

	PROX_DBG("=====<PROXIMITY> prodrv_sysfs_status_show:%d=====\n",status);
	
	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_delay_show, prodrv_sysfs_delay_store);
static DEVICE_ATTR(enable, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP,
		prodrv_sysfs_enable_show, prodrv_sysfs_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		NULL, prodrv_sysfs_wake_store);
static DEVICE_ATTR(data, S_IWUGO|S_IRUGO|S_IWUSR|S_IWGRP, prodrv_sysfs_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, prodrv_sysfs_status_show, NULL);

static struct attribute *sensor_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group sensor_attribute_group = {
	.attrs = sensor_attributes
};

static int
prodrv_sns_suspend(struct platform_device *pdev, pm_message_t state)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_suspend=====\n");
	
	/* implement suspend of the sensor */
	/*if call is not incoming, go to suspend */
	if(!atomic_read(&g_wake)) {
		prodrv_sns_OFF();
		atomic_set(&g_suspend_off,TRUE);
	}

	atomic_set(&g_prox_state,PROX_STATE_SUSPEND);

	return 0;
}

static int
prodrv_sns_resume(struct platform_device *pdev)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_resume=====\n");
	
	/* implement resume of the sensor */
	if(atomic_read(&g_suspend_off)) {
		prodrv_sns_ON();
		//interrupt clearing
		prodrv_sns_i2c_write( CON_REG | INT_CLEAR, CON_REG_INIT);

		atomic_set(&g_suspend_off,FALSE);
	}

	if(atomic_read(&g_prox_state) == PROX_STATE_WAIT_I2C) {
		schedule_work(&g_proxi_work_data);
	}
	else {
		atomic_set(&g_prox_state,PROX_STATE_WAKE);
	}
	return 0;
}

static int
prodrv_sns_probe(struct platform_device *pdev)
{
	struct sensor_data *data = NULL;
	static struct input_dev *input_data = NULL;
	int input_registered = 0;
	int  sysfs_created = 0;
	int rt;

	PROX_DBG("=====<PROXIMITY> prodrv_sns_probe=====\n");

#ifndef CONFIG_GPIOLIB
#else
	gpio_request(PROX_GPIO, SENSOR_NAME);
#endif
	prodrv_sns_ON();
	
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
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_X);

	input_data->name = SENSOR_NAME;
	rt = input_register_device(input_data);
	if (rt) {
		goto err;
	}
	input_registered = 1;
	rt = sysfs_create_group(&input_data->dev.kobj,
			&sensor_attribute_group);
	if (rt) {
		goto err;
	}
	sysfs_created = 1;
	mutex_init(&data->mutex);
	this_data = input_data;
	input_report_abs(input_data, ABS_X, PROX_VAL_FAR);
	input_sync(input_data);	
	prodrv_sns_request_irqs();
	return 0;

err:
	if (data != NULL) {
		if (input_data != NULL) {
			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
						&sensor_attribute_group);
			}
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
prodrv_sns_remove(struct platform_device *pdev)
{
	struct sensor_data *data;

	PROX_DBG("=====<PROXIMITY> prodrv_sns_remove=====\n");
	
	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		sysfs_remove_group(&this_data->dev.kobj,
				&sensor_attribute_group);
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
	.probe	  = prodrv_sns_probe,
	.remove	 = prodrv_sns_remove,
	.suspend	= prodrv_sns_suspend,
	.resume	 = prodrv_sns_resume,
	.driver = {
		.name   = SENSOR_NAME,
		.owner  = THIS_MODULE,
	},
};

/*powerup the PMIC, configure GPIO 37,
set GPIO to NO_PULL, make it as input pin.
Give sleep of 10 millisecs, call registry init
*/
int
prodrv_sns_ON(void)
{
	int ok_flag = TRUE;

	PROX_DBG("=====<PROXIMITY> prodrv_sns_ON=====\n");

	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;

	sub_pmic_ldo_ctrl(SUB_PMIC_LDO4,SUB_PMIC_LDO_ON);

#ifndef CONFIG_GPIOLIB
	gpio_configure(PROX_GPIO, GPIOF_ENABLE_WAKE | GPIOF_INPUT);
#else
	gpio_direction_input(PROX_GPIO);
#endif
	
	//added to set it to no pull as per spec
	gpio_tlmm_config(GPIO_CFG(PROX_GPIO, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);

	msleep(SNS_STANDBY_TIME);

	ok_flag = prodrv_sns_reg_init();
	if(ok_flag!=TRUE) {
#ifndef CONFIG_GPIOLIB
		gpio_configure(PROX_GPIO, GPIOF_DRIVE_OUTPUT);
#else
		gpio_direction_output(PROX_GPIO, 1);
#endif
	}
	return ok_flag;
}

/*power-down the device
configure GPIO 37 to out*/
void
prodrv_sns_OFF(void)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_OFF=====\n");

#ifndef CONFIG_GPIOLIB
	gpio_configure(PROX_GPIO, GPIOF_DRIVE_OUTPUT);
#else
	gpio_direction_output(PROX_GPIO, 1);
#endif
	
	gpio_tlmm_config(GPIO_CFG(PROX_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_DISABLE);

	prodrv_sns_ssd();

	sub_pmic_ldo_ctrl(SUB_PMIC_LDO4,SUB_PMIC_LDO_OFF);

	prox_detect.proxy_val = PROX_VAL_NEAR;
	prox_detect.proxy_state = PROX_DETECT_NON;
	
	del_timer(&prox_timer);
}

/*Initialize all registry values of the proximity
sensor as per specifications, use i2c write to write
register values*/
static int
prodrv_sns_reg_init(void)
{
	int		init_sts;
	init_sts = prodrv_sns_i2c_write( GAIN_REG, GAIN_LED );
	
	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
	}
	
	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( CYCLE_REG, CYCLE_OSC);
	}

	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( OPMOD_REG, OPMOD_SSD | OPMOD_VCON);
	}

	if(0 <= init_sts) {
		init_sts &= prodrv_sns_i2c_write( CON_REG, CON_REG_INIT);
	}
	
	if(0 > init_sts) {
		PROX_DBG("=====<PROXIMITY> prodrv_sns_reg_init FALSE=====\n");
		return FALSE;
	}
	return TRUE;
}

/*change the register values so that it is ok to
shutdown*/
static void
prodrv_sns_ssd(void)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_ssd=====\n");

	prodrv_sns_i2c_write( OPMOD_REG, OPMOD_VCON);
	prodrv_sns_i2c_write( CON_REG, CON_OCON | CON_OCON_1);
}

/*does i2c write to the proximity sensor*/
static int
prodrv_sns_i2c_write(unsigned char reg, unsigned char data)
{
	struct i2c_msg msg;
	u_int8_t buf[I2C_WRITE_BUFSIZE];
	int ret = 0;

	msg.addr = PROX_SLAVE_ADDRESS;
	msg.flags = 0;
	buf[0] = reg;
	buf[1] = data;

	msg.buf  = buf;
	msg.len  = 2;

	ret = i2c_transfer(i2c_proximity, &msg, I2C_TRANS_WRITE);

	return ret;
}

/*does i2c read from the proximity sensor*/
static int
prodrv_sns_i2c_read(unsigned short reg, unsigned char *data, uint32_t len)
{
	struct i2c_msg msg[READ_SIZE];
	u_int8_t msgbuf[READ_SIZE];
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

	ret = i2c_transfer(i2c_proximity, msg, I2C_TARNS_READ);

	return ret;
}

void prodrv_sns_report_abs(unsigned long data)
{
	struct prox_detect_state *state = (struct prox_detect_state *)data;
 
	if(state->proxy_val == PROX_VAL_NEAR) {
		state->proxy_state = PROX_DETECTED;
		PROX_DBG("=====<PROXIMITY> proxi_work_bh NEAR 2 delay:%d=====\n", prox_wait);
		input_report_abs(this_data, ABS_X, state->proxy_val);
		input_sync(this_data);	
	}
}

//bottom half of interrupt handling
static void
prodrv_sns_work_bh(struct work_struct *work)
{
	int init_sts;

	unsigned char data1[READ_SIZE];

	PROX_DBG("=====<PROXIMITY> proxi_work_bh=====\n");
	
	if(atomic_read(&g_prox_state) != PROX_STATE_SUSPEND) {
		prodrv_sns_i2c_read(PROX_REG, data1, READ_SIZE);

		if(data1[I2C_READ_POS] & PROX_REG_NEAR) {
			//changing as per new spec change
			//check if it is 01 or 10 as per spec
			init_sts = prodrv_sns_i2c_write( HYS_REG, g_prox_sense_a);
			PROX_DBG("=====<PROXIMITY> proxi_work_bh NEAR 1=====\n");

			// save state
			prox_detect.proxy_val = PROX_VAL_NEAR;
			prox_detect.proxy_state = PROX_DETECTING;
	
			// start timer
			init_timer(&prox_timer);
			prox_timer.function = prodrv_sns_report_abs;
			prox_timer.expires = jiffies + msecs_to_jiffies(prox_wait); //ms
			prox_timer.data = (unsigned long)&prox_detect;
			add_timer(&prox_timer);
		}
		else {
			init_sts = prodrv_sns_i2c_write( HYS_REG, g_prox_sense_b);
			PROX_DBG("=====<PROXIMITY> proxi_work_bh FAR 1=====\n");

			// save state
			prox_detect.proxy_val = PROX_VAL_FAR;
			
			if(prox_detect.proxy_state == PROX_DETECTING) {
				// If something leaves from the sensor within "prox_wait" ms,
				// the proximity driver cancel the timer. And it do NOT inform detecting.
				del_timer(&prox_timer);
				PROX_DBG("=====<PROXIMITY> proxi_work_bh FAR TimerDelete=====\n");
			}
			else {
				PROX_DBG("=====<PROXIMITY> proxi_work_bh FAR Notice=====\n");
				input_report_abs(this_data, ABS_X, prox_detect.proxy_val);
				input_sync(this_data);
			}

			prox_detect.proxy_state = PROX_DETECT_NON;
		}

		init_sts = prodrv_sns_i2c_write( CON_REG, CON_OCON | CON_OCON_1);

		//interrupt clearing
		init_sts = prodrv_sns_i2c_write( CON_REG | INT_CLEAR, CON_REG_INIT);

		enable_irq(g_proxi_irq);
	}
  	else {
		PROX_DBG("=====<PROXIMITY> proxi_work_bh I2C Wait=====\n");
		atomic_set(&g_prox_state,PROX_STATE_WAIT_I2C);
  	}
}

//irq handler
static irqreturn_t
prodrv_sns_irq_handler(int irq, void *p)
{
	disable_irq_nosync(g_proxi_irq);
	schedule_work(&g_proxi_work_data);
	return IRQ_HANDLED;
}

//irq request
static int
prodrv_sns_request_irqs(void)
{
	int err;
	unsigned long req_flags = IRQF_TRIGGER_FALLING;

	err = g_proxi_irq = gpio_to_irq(PROX_GPIO);
	err = request_irq(g_proxi_irq, prodrv_sns_irq_handler,
			req_flags, SENSOR_NAME, NULL);
	if (err) {
		return err;
	}
	set_irq_wake(g_proxi_irq, WAKE_ON);
	return 0;
}

/*does sensor initialization*/
static int __init prodrv_sns_init(void)
{
	signed char level_a;
	signed char level_b;

	PROX_DBG("=====<PROXIMITY> prodrv_sns_init=====\n");
	
	i2c_proximity = i2c_get_adapter(0);

	INIT_WORK(&g_proxi_work_data, prodrv_sns_work_bh);
	atomic_set(&g_wake,FALSE);
	atomic_set(&g_suspend_off,FALSE);
	atomic_set(&g_prox_state,PROX_STATE_WAKE);

	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev)) {
		return -1;
	}
	g_prox_sense_a = prox_sensitivity_table[PROX_SENS_A_DEFAULT];
	g_prox_sense_b = prox_sensitivity_table[PROX_SENS_B_DEFAULT];
	
	level_a = msm_prox_read_nvitem(NV_PROX_SENSE_A_I);
	if((level_a >= 0) && (level_a < PROX_SENS_MAX_NUM)) {
		level_b = msm_prox_read_nvitem(NV_PROX_SENSE_B_I);
		if((level_b >= 0 ) && (level_b < PROX_SENS_MAX_NUM) && (level_a < level_b)) {
			/* NV value access success. */
			g_prox_sense_a = prox_sensitivity_table[level_a];
			g_prox_sense_b = prox_sensitivity_table[level_b];
		}
	}

	PROX_DBG("=====<PROXIMITY> sensor a:%d=====\n", g_prox_sense_a);
	PROX_DBG("=====<PROXIMITY> sensor b:%d=====\n", g_prox_sense_b);
	
	return platform_driver_register(&sensor_driver);
}
module_init(prodrv_sns_init);

/*does sensor shutdown*/
static void __exit prodrv_sns_exit(void)
{
	PROX_DBG("=====<PROXIMITY> prodrv_sns_exit=====\n");
	
	prodrv_sns_OFF();
	platform_driver_unregister(&sensor_driver);
	platform_device_unregister(sensor_pdev);
}
module_exit(prodrv_sns_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");
