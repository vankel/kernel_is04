/*
 * Certain software is contributed or developed by 
 * FUJITSU TOSHIBA MOBILE COMMUNICATIONS LIMITED.
 *
 * COPYRIGHT(C) FUJITSU TOSHIBA MOBILE COMMUNICATIONS LIMITED 2011
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by FSF, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This code is based on android.c.
 * The original copyright and notice are described below.
 */
/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
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



#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include <linux/usb/android.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/cryptohash.h>
#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
#include "u_ether.h"
#endif

#include "f_mass_storage.h"
#include "f_adb.h"
#include "u_serial.h"
#ifdef CONFIG_USB_ANDROID_DIAG
#include "f_diag.h"
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
#include "f_rmnet.h"
#endif

#include "gadget_chips.h"


#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

int udc_online_complete = 0;




#define TSB_PRODUCT_ID      0x0D8D
#define LSM_PRODUCT_ID      0x0D8C

#if 0
static u16 product_id;
#else
static u16 product_id = TSB_PRODUCT_ID;
#endif

static int android_set_pid(const char *val, struct kernel_param *kp);
static int android_get_pid(char *buffer, struct kernel_param *kp);
module_param_call(product_id, android_set_pid, android_get_pid,
					&product_id, 0664);
MODULE_PARM_DESC(product_id, "USB device product id");


#define MAX_SERIAL_LEN 256
static char serial_number[MAX_SERIAL_LEN] = "1234567890ABCDEF";

static char global_serial_number[MAX_SERIAL_LEN] = {0};

static struct kparam_string kps = {
	.string			= serial_number,
	.maxlen			= MAX_SERIAL_LEN,
};
static int android_set_sn(const char *kmessage, struct kernel_param *kp);
module_param_call(serial_number, android_set_sn, param_get_string,
						&kps, 0664);
MODULE_PARM_DESC(serial_number, "SerialNumber string");


static unsigned int  boot_set_pid = 0;


#define ENABLE_PWSTRING_IF
#define NV_GPRS_ANITE_GCF_I     947
#define NV_OPEFLAG_I            10035

#define PID__ADB_MSC____________    0x0D85
#define PID__ADB_MSC_____MDM____    0x9002
#define PID__ADB_MSC_DIG_____GPS    0x9003
#define PID__ADB_MSC_DIG_MDM_GPS    0x9018

#define PID__ADB_MSC_DIG_MDM_GPS_LSM    0x9018



static int android_set_serialnumber(const char *kmessage);

extern int msm_usb_read_nvitem(unsigned int id, unsigned short *data);
extern int msm_usb_write_nvitem(unsigned int id, unsigned short *data);
extern int store_state;

#ifdef ENABLE_PWSTRING_IF
static char password_string[MAX_SERIAL_LEN] = {0};

static unsigned int  desired_password[5] = {0x4d347823,0x685235e1,0x99803e04,0xcd56a201,0x5c5ec4f2};

static unsigned int  output_password[5] = {0}; 

static struct kparam_string kps_pass = {
        .string                 = password_string,
        .maxlen                 = MAX_SERIAL_LEN,
};

static int android_set_password(const char *kmessage, struct kernel_param *kp);
static int android_get_password(unsigned int *buffer, struct kernel_param *kp);

module_param_call(password_string, android_set_password, android_get_password,
                        &kps_pass, 0664);

MODULE_PARM_DESC(password_string, "PasswordNumber string");
#endif


extern int for_serial_notification(void);


static const char longname[] = "Gadget Android";
#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
static u8 hostaddr[ETH_ALEN];
#endif


#define VENDOR_ID		0x18D1

struct android_dev {
	struct usb_gadget *gadget;
	struct usb_composite_dev *cdev;

	int version;

	int adb_enabled;
	struct mutex lock;
	struct android_usb_platform_data *pdata;
	unsigned long functions;
};

static struct android_dev *_android_dev;



#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2


static struct usb_string strings_dev[] = {

	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",	
	{  }			
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

#define android_func_attr(function, index)				\
static ssize_t  show_##function(struct device *dev,			\
		struct device_attribute *attr, char *buf)		\
{									\
									\
	unsigned long n = _android_dev->functions;			\
	int val = 0;							\
									\
	while (n) {							\
		if ((n & 0x0F) == index)				\
			val = 1;					\
		n = n >> 4;						\
	}								\
	return sprintf(buf, "%d\n", val);				\
									\
}									\
									\
static DEVICE_ATTR(function, S_IRUGO, show_##function, NULL);

android_func_attr(adb, ANDROID_ADB);
android_func_attr(mass_storage, ANDROID_MSC);
android_func_attr(acm_modem, ANDROID_ACM_MODEM);
android_func_attr(acm_nmea, ANDROID_ACM_NMEA);
android_func_attr(diag, ANDROID_DIAG);
android_func_attr(modem, ANDROID_GENERIC_MODEM);
android_func_attr(nmea, ANDROID_GENERIC_NMEA);
android_func_attr(cdc_ecm, ANDROID_CDC_ECM);
android_func_attr(rmnet, ANDROID_RMNET);
android_func_attr(rndis, ANDROID_RNDIS);

android_func_attr(atport, ANDROID_GENERIC_ATPORT);


static struct attribute *android_func_attrs[] = {
	&dev_attr_adb.attr,
	&dev_attr_mass_storage.attr,
	&dev_attr_acm_modem.attr,
	&dev_attr_acm_nmea.attr,
	&dev_attr_diag.attr,
	&dev_attr_modem.attr,
	&dev_attr_nmea.attr,
	&dev_attr_cdc_ecm.attr,
	&dev_attr_rmnet.attr,
	&dev_attr_rndis.attr,

	&dev_attr_atport.attr,

	NULL,
};

static struct attribute_group android_func_attr_grp = {
	.name  = "functions",
	.attrs = android_func_attrs,
};

static int  android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = -EINVAL;
	unsigned long n;
	pr_debug("android_bind_config c = 0x%x dev->cdev=0x%x\n",
		(unsigned int) c, (unsigned int) dev->cdev);
	n = dev->functions;
	while (n) {
		switch (n & 0x0F) {
		case ANDROID_ADB:
			ret = adb_function_add(dev->cdev, c);
			if (ret)
				return ret;
			break;
		case ANDROID_MSC:
			ret = mass_storage_function_add(dev->cdev, c);
			if (ret)
				return ret;
			break;
		case ANDROID_ACM_MODEM:
			ret = acm_bind_config(c, 0);
			if (ret)
				return ret;
			break;
		case ANDROID_ACM_NMEA:
			ret = acm_bind_config(c, 1);
			if (ret)
				return ret;
			break;
#ifdef CONFIG_USB_ANDROID_DIAG
		case ANDROID_DIAG:
			ret = diag_function_add(c, serial_number);
			if (ret)
				return ret;
			break;
#endif
#ifdef CONFIG_USB_F_SERIAL
		case ANDROID_GENERIC_MODEM:


			ret = gser_bind_config(c, 1);

			if (ret)
				return ret;
			break;
		case ANDROID_GENERIC_NMEA:


			ret = gser_bind_config(c, 2);

			if (ret)
				return ret;
			break;

		case ANDROID_GENERIC_ATPORT:
			ret = lismo_bind_config(c, 0);
			if (ret)
				return ret;
			break;

#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
		case ANDROID_CDC_ECM:
			ret = ecm_bind_config(c, hostaddr);
			if (ret)
				return ret;
			break;
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
		case ANDROID_RMNET:
			ret = rmnet_function_add(c);
			if (ret) {
				pr_err("failed to add rmnet function\n");
				return ret;
			}
			break;
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
		case ANDROID_RNDIS:
			ret = rndis_bind_config(c, hostaddr);
			if (ret)
				return ret;
			break;
#endif
		default:
			ret = -EINVAL;
			return ret;
		}
		n = n >> 4;
	}
	return ret;

}

static int get_num_of_serial_ports(void)
{
	struct android_dev *dev = _android_dev;
	unsigned long n = dev->functions;
	unsigned ports = 0;

	while (n) {
		switch (n & 0x0F) {
		case ANDROID_ACM_MODEM:
		case ANDROID_ACM_NMEA:
		case ANDROID_GENERIC_MODEM:
		case ANDROID_GENERIC_NMEA:

		case ANDROID_GENERIC_ATPORT:

			ports++;
		}
		n = n >> 4;
	}

	return ports;
}

static int is_iad_enabled(void)
{
	struct android_dev *dev = _android_dev;
	unsigned long n = dev->functions;

	while (n) {
		switch (n & 0x0F) {
		case ANDROID_ACM_MODEM:
		case ANDROID_ACM_NMEA:
#ifdef CONFIG_USB_ANDROID_RNDIS
		case ANDROID_RNDIS:
#endif
			return 1;
		}
		n = n >> 4;
	}

	return 0;
}

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.bConfigurationValue = 1,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_unbind(struct usb_composite_dev *cdev)
{
	if (get_num_of_serial_ports())
		gserial_cleanup();

	return 0;
}

static int  android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum;
	int			id;
	int			ret;
	int                     num_ports;

	pr_debug("android_bind\n");


	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	device_desc.idProduct = __constant_cpu_to_le16(product_id);

	if ((gadget->ops->wakeup) && (dev->functions != ANDROID_MSC))
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	else
		android_config_driver.bmAttributes &= ~USB_CONFIG_ATT_WAKEUP;

	if (dev->pdata->self_powered && !usb_gadget_set_selfpowered(gadget)) {
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_SELFPOWER;
		android_config_driver.bMaxPower	= 0x32; 
	}
	dev->cdev = cdev;
	pr_debug("android_bind assigned dev->cdev\n");
	dev->gadget = gadget;

	num_ports = get_num_of_serial_ports();
	if (num_ports) {
		ret = gserial_setup(cdev->gadget, num_ports);
		if (ret < 0)
			return ret;
	}


#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
	
	ret = gether_setup(cdev->gadget, hostaddr);
	if (ret && (ret != -EBUSY)) {
		gserial_cleanup();
		return ret;
	}
#endif

	
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		pr_err("usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	if (is_iad_enabled()) {
		device_desc.bDeviceClass         = USB_CLASS_MISC;
		device_desc.bDeviceSubClass      = 0x02;
		device_desc.bDeviceProtocol      = 0x01;
	} else {
		device_desc.bDeviceClass         = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass      = 0;
		device_desc.bDeviceProtocol      = 0;
	}
	pr_debug("android_bind done\n");
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.unbind		= android_unbind,
};

struct usb_composition *android_validate_product_id(unsigned short pid)
{
	struct android_dev *dev = _android_dev;
	struct usb_composition *fi;
	int i;

	for (i = 0; i < dev->pdata->num_compositions; i++) {
		fi = &dev->pdata->compositions[i];
		pr_debug("pid=0x%x apid=0x%x\n",
		       fi->product_id, fi->adb_product_id);
		if ((fi->product_id == pid) || (fi->adb_product_id == pid))
			return fi;
	}
	return NULL;
}

static int android_switch_composition(u16 pid)
{
	struct android_dev *dev = _android_dev;
	struct usb_composition *func;
	int ret;

	int len = 0;
	char temp_serial_number_default[MAX_SERIAL_LEN] = {0};
	char temp_serial_number[MAX_SERIAL_LEN + 1] = {0};
	strlcpy(temp_serial_number_default, global_serial_number, MAX_SERIAL_LEN);


	
	func = android_validate_product_id(pid);
	if (!func) {
		pr_err("%s: invalid product id %x\n", __func__, pid);
		return -EINVAL;
	}

	
	if (dev->adb_enabled) {
		product_id = func->adb_product_id;
		dev->functions = func->adb_functions;

        android_set_serialnumber(temp_serial_number_default);

	} else {
		product_id = func->product_id;
		dev->functions = func->functions;

        if (product_id == TSB_PRODUCT_ID) {
			strlcpy(temp_serial_number, global_serial_number, MAX_SERIAL_LEN);
			len = strlen(temp_serial_number);

			if (len <= (MAX_SERIAL_LEN - 2)) {
			    
				temp_serial_number[len] = '0';
				temp_serial_number[len + 1] = '\0';
			}
			android_set_serialnumber(temp_serial_number);
		}
		else {
            android_set_serialnumber(temp_serial_number_default);
		}

	}

	usb_composite_unregister(&android_usb_driver);
	ret = usb_composite_register(&android_usb_driver);

	return ret;
}

static ssize_t android_remote_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_gadget *gadget = _android_dev->gadget;

	if (!gadget)
		return -ENODEV;

	pr_debug("Calling remote wakeup....\n");
	usb_gadget_wakeup(gadget);

	return count;
}
static DEVICE_ATTR(remote_wakeup, S_IWUSR, 0, android_remote_wakeup);

static struct attribute *android_attrs[] = {
	&dev_attr_remote_wakeup.attr,
	NULL,
};

static struct attribute_group android_attr_grp = {
	.attrs = android_attrs,
};


#ifdef ENABLE_PWSTRING_IF
static void android_hash_password(char *kmessage)
{
	unsigned int hash[5], workspace[SHA_WORKSPACE_WORDS];
	char *local_buf[MAX_SERIAL_LEN] = {0};
	int i =0;
	strlcpy(local_buf, kmessage, MAX_SERIAL_LEN);
	local_buf[64] = '\0';
	sha_init(hash);
	sha_transform(hash, (unsigned char *)local_buf, workspace);
	for (i = 0; i < 5; i++)
	{
		output_password[i]= hash[i];
	}
}

static int android_set_password(const char *kmessage, struct kernel_param *kp)
{
	int len = 0;
	int i =0;
	if(kmessage)
		len = strlen(kmessage);
	else
	{
		for(i=0;i<5;i++)
		{
			output_password[i] = 0;
		}
		return 0;
	}
	
	if (len > 64) {
		return -ENOSPC;
	}
	android_hash_password(kmessage);
	return 0;
}
static int android_get_password(unsigned int *buffer, struct kernel_param *kp)
{
	int ret = 0;
	int i =0;
	for(i=0;i<5;i++)
	{
		buffer[i] = output_password[i];
		output_password[i] = 0;
	}
	return ret;
}
#endif 

static int android_set_nv(unsigned long pid, struct kernel_param *kp)
{
	int ret = 0;

	unsigned short nv_ope = 0;
	int usbsetting = 0;
#ifdef ENABLE_PWSTRING_IF
	unsigned int  buffer [5] = {0};	
	int loop =0;
#endif 

	switch (pid) {

#if 0
	case PID__ADB_MSC_DIG_MDM_GPS:
	case PID__ADB_MSC_DIG_____GPS:
	case PID__ADB_MSC_____MDM____:
	case TSB_PRODUCT_ID:
#else
	case PID__ADB_MSC_DIG_MDM_GPS_LSM:
#endif

		usbsetting = 1;
#ifdef ENABLE_PWSTRING_IF
		android_get_password(buffer, kp);
		for (loop =0; loop<5; loop ++) {
			if( buffer[loop] != desired_password[loop] )
				goto out;
		}
#endif 
		break;

	case TSB_PRODUCT_ID:
	case LSM_PRODUCT_ID:
		usbsetting = 1;
		break;

	default:	
		break;
	}
	
	if (usbsetting) {

#if 0
		if (msm_usb_read_nvitem(NV_GPRS_ANITE_GCF_I, &nv_gcf) != 0)
			goto out;
		
		
		if (pid == PID__ADB_MSC_DIG_MDM_GPS && nv_gcf == 1)
			nv_ope = 2;
		else if (pid == PID__ADB_MSC_DIG_____GPS && nv_gcf == 0)
			nv_ope = 2;
		else if (pid == PID__ADB_MSC_____MDM____ && nv_gcf == 1)
			nv_ope = 1;
		else if (pid == TSB_PRODUCT_ID && nv_gcf == 0)
			nv_ope = 1;
		else
			goto out;
#else
		if (pid == PID__ADB_MSC_DIG_MDM_GPS_LSM)
			nv_ope = 2;
		else if (pid == TSB_PRODUCT_ID)
			nv_ope = 0;
		#if 0	
		else if (pid == LSM_PRODUCT_ID)
			nv_ope = 1;
		#endif	
#endif

		
		if (msm_usb_write_nvitem(NV_OPEFLAG_I, &nv_ope) != 0)
			goto out;
	}
	
	ret = 1;	
out:
	return ret;
}

static unsigned long android_get_pid_from_nv(void)
{
	unsigned long pid = 0;

#if 0
	unsigned short nv_gcf;
	unsigned short nv_ope;
	
	if (msm_usb_read_nvitem(NV_GPRS_ANITE_GCF_I, &nv_gcf) != 0)
		goto out;
	
	if (msm_usb_read_nvitem(NV_OPEFLAG_I, &nv_ope) != 0)
		goto out;
	
	if (nv_gcf == 0 && nv_ope == 0x02)
		pid = PID__ADB_MSC_DIG_____GPS;
	else if (nv_gcf == 1 && nv_ope == 0x01)
		pid = PID__ADB_MSC_____MDM____;	
	else if (nv_gcf == 1 && nv_ope == 0x02)
		pid = PID__ADB_MSC_DIG_MDM_GPS;
	else if (nv_gcf == 0 && nv_ope == 0x01)
		pid = TSB_PRODUCT_ID;
#else
	unsigned short nv_ope;
	
	if (msm_usb_read_nvitem(NV_OPEFLAG_I, &nv_ope) != 0)
		goto out;
	
	if (nv_ope == 0x02)
		pid = PID__ADB_MSC_DIG_MDM_GPS_LSM;
	#if 0	
	else if (nv_ope == 0x01)
		pid = LSM_PRODUCT_ID;
	#endif	
#endif

	
out:
	return pid;
}


static int android_set_sn(const char *kmessage, struct kernel_param *kp)
{
	int len = strlen(kmessage);

	if (len >= MAX_SERIAL_LEN) {
		pr_err("serial number string too long\n");
		return -ENOSPC;
	}

	strlcpy(serial_number, kmessage, MAX_SERIAL_LEN);

	if (serial_number[len - 1] == '\n')
		serial_number[len - 1] = '\0';

	return 0;
}

static int android_set_pid(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned long tmp;
	unsigned long pid = 0;  

pr_info("[gadget/android.c] %s(%s)\n", __func__, val);

	ret = strict_strtoul(val, 16, &tmp);
	if (ret)
		goto out;


	if (!_android_dev) {
		product_id = tmp;
		goto out;
	}


	if (tmp == 0xffff) {
		tmp = product_id;
		if (!store_state)
			goto out;
	}
	if (!boot_set_pid) {
		pid = android_get_pid_from_nv();
		if (pid)
			tmp = pid;
		
		boot_set_pid = 1;
	}
	else {
		if (!android_set_nv(tmp, kp)) {
			printk(KERN_ERR,"%s:invalid combination %x\n",__func__,tmp);
			ret = -EINVAL;
			goto out;
		}
	}


	unsigned long timeout;
	timeout = jiffies + msecs_to_jiffies(700);
	while (udc_online_complete) {
		if (time_after(jiffies, timeout)) {
			pr_info("[gadget/android.c] %s()USER CALL WARNING!!\n", __func__);
			return -ENOSPC;
		}
		msleep(10);
	}

	mutex_lock(&_android_dev->lock);
	ret = android_switch_composition(tmp);
	mutex_unlock(&_android_dev->lock);
out:
	return ret;
}

static int android_get_pid(char *buffer, struct kernel_param *kp)
{
	int ret;

	mutex_lock(&_android_dev->lock);
	ret = sprintf(buffer, "%x", product_id);
	mutex_unlock(&_android_dev->lock);

	return ret;
}

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

pr_info("[gadget/android.c] %s()\n", __func__);

	unsigned long timeout;
	timeout = jiffies + msecs_to_jiffies(700);
	while (udc_online_complete) {
		if (time_after(jiffies, timeout)) {
			pr_info("[gadget/android.c] %s()USER CALL WARNING!!\n", __func__);
			return -ENOSPC;
		}
		msleep(10);
	}

	mutex_lock(&dev->lock);

	if (dev->adb_enabled)
		goto out;

	dev->adb_enabled = 1;
	pr_debug("enabling adb\n");
	if (product_id)
		ret = android_switch_composition(product_id);
out:
	mutex_unlock(&dev->lock);

	return ret;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

pr_info("[gadget/android.c] %s()\n", __func__);

	unsigned long timeout;
	timeout = jiffies + msecs_to_jiffies(700);
	while (udc_online_complete) {
		if (time_after(jiffies, timeout)) {
			pr_info("[gadget/android.c] %s()USER CALL WARNING!!\n", __func__);
			return -ENOSPC;
		}
		msleep(10);
	}


	mutex_lock(&dev->lock);

	if (!dev->adb_enabled)
		goto out;

	pr_debug("disabling adb\n");
	dev->adb_enabled = 0;
	if (product_id)
		ret = android_switch_composition(product_id);
out:
	mutex_unlock(&dev->lock);

	return ret;
}

static struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};


static int android_set_serialnumber(const char *kmessage)
{
	int len = strlen(kmessage);

	if (len >= MAX_SERIAL_LEN) {
		printk(KERN_ERR "serial number string too long\n");
		return -ENOSPC;
	}
	memset(serial_number,0,MAX_SERIAL_LEN);
	strlcpy(serial_number, kmessage, MAX_SERIAL_LEN);
	
	if (serial_number[len - 1] == '\n')
		serial_number[len - 1] = '\0';

	return 0;
}

static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int ret;

	pr_debug("android_probe pdata: %p\n", pdata);

	if (!pdata || !pdata->vendor_id || !pdata->product_name ||
		!pdata->manufacturer_name)
		return -ENODEV;

	device_desc.idVendor =	__constant_cpu_to_le16(pdata->vendor_id);
	dev->version = pdata->version;
	strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
	strings_dev[STRING_MANUFACTURER_IDX].s = pdata->manufacturer_name;
	android_set_serialnumber(pdata->product_serial_number); 
	strings_dev[STRING_SERIAL_IDX].s = serial_number;

	strlcpy(global_serial_number, serial_number, MAX_SERIAL_LEN);

	dev->pdata = pdata;

	ret = sysfs_create_group(&pdev->dev.kobj, &android_attr_grp);
	if (ret < 0) {
		pr_err("%s: Failed to create the sysfs entry \n", __func__);
		return ret;
	}
	ret = sysfs_create_group(&pdev->dev.kobj, &android_func_attr_grp);
	if (ret < 0) {
		pr_err("%s: Failed to create the functions sysfs entry \n",
				__func__);
		sysfs_remove_group(&pdev->dev.kobj, &android_attr_grp);
	}

	return ret;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;
	struct usb_composition *func;
	int ret;
	unsigned long pid = 0;  

    int len = 0;

	pr_debug("android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto out;
	}

	_android_dev = dev;
	mutex_init(&dev->lock);

	ret = adb_function_init();
	if (ret)
		goto free_dev;

	ret = platform_driver_register(&android_platform_driver);
	if (ret)
		goto adb_exit;

	ret = misc_register(&adb_enable_device);
	if (ret)
		goto pdrv_unregister;


	ret = for_serial_notification();
	if (ret != 0)
		goto misc_deregister;

	
	mutex_lock(&dev->lock);
	if (!product_id) {
		mutex_unlock(&dev->lock);
		ret = 0; 
		boot_set_pid = 0;   
		goto out;
	}


	pid = android_get_pid_from_nv();
	if (pid)
		product_id = pid;
    if (product_id == TSB_PRODUCT_ID) {
		len = strlen(serial_number);
        if (len <= (MAX_SERIAL_LEN - 2)) {
		    
		    serial_number[len] = '0';
		    serial_number[len + 1] = '\0';
		}
	}

	func = android_validate_product_id(product_id);
	if (!func) {
		mutex_unlock(&dev->lock);
		pr_err("%s: invalid product id\n", __func__);
		ret = -EINVAL;
		goto misc_deregister;
	}
	dev->functions = func->functions;

	ret = usb_composite_register(&android_usb_driver);
	if (ret) {
		mutex_unlock(&dev->lock);
		goto misc_deregister;
	}
	boot_set_pid = 1;   
	mutex_unlock(&dev->lock);

	return 0;

misc_deregister:
	misc_deregister(&adb_enable_device);
pdrv_unregister:
	platform_driver_unregister(&android_platform_driver);
adb_exit:
	adb_function_exit();
free_dev:
	kfree(dev);
out:
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
	gether_cleanup();
#endif

	usb_composite_unregister(&android_usb_driver);
	misc_deregister(&adb_enable_device);
	platform_driver_unregister(&android_platform_driver);
	adb_function_exit();
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
