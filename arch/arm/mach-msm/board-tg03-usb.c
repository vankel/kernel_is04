/*
  USB Driver

  COPYRIGHT(C) FUJITSU TOSHIBA MOBILE COMMUNICATIONS LIMITED 2011

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
/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include "devices.h"
#include "proc_comm.h"
#include "smd_private.h"
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif

#ifdef CONFIG_USB_ANDROID
/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		/* MSC */
		.product_id         = 0x0D8D,
		.functions	    = 0x02,
		.adb_product_id     = 0x0D8D,
		.adb_functions	    = 0x12
	},
	{
		/* LISMO */
		.product_id         = 0x0D8C,
		.functions          = 0xB,
		.adb_product_id     = 0x0D8C,
		.adb_functions      = 0x1B,
	},
#if defined(CONFIG_USB_ANDROID_DIAG) && defined(CONFIG_USB_F_SERIAL)
	{
		/* DIAG + NMEA + MSC */
		.product_id 		= 0x9000,
		.functions			= 0x274,
		.adb_product_id 	= 0x9003,
		.adb_functions		= 0x2714,
	},
	{
		/* MSC + MODEM */
		.product_id 		= 0x9001,
		.functions			= 0x62,
		.adb_product_id 	= 0x9002,
		.adb_functions		= 0x612,
	},
	{
		/* DIAG + MODEM + NMEA + MSC + LISMO */
		.product_id         = 0x9017,
		.functions	    = 0x2476B,
		.adb_product_id     = 0x9018,
		.adb_functions	    = 0x24761B,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		/* RNDIS */
		.product_id         = 0x0D8E,
		.functions	    = 0xA,
		.adb_product_id     = 0x0D8E,
		.adb_functions	    = 0x1A,
	},
#endif
};
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Toshiba",
	.product	= "Mass Storage",
	.release	= 0xFFFF,
};

struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &mass_storage_pdata,
	},
};
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0930,
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.product_name	= "Toshiba HSUSB Device",
	.manufacturer_name = "Toshiba Corporation",
};

struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static void msm_read_nv_imei()
{
	uint8_t *dataread = NULL;
	uint8_t i, digit, imei_bcd_len = 0;
	char imei_ascii[ ( MSM_NV_UE_IMEI_SIZE - 1 ) * 2 + 1 ] = {0};
	uint8_t nv_imei_read[MSM_NV_UE_IMEI_SIZE] = {0};
	int length = 0,j = 5; 
	char temp_arr[7] = {0};

	dataread = smem_alloc(SMEM_OEM_003,MSM_NV_UE_IMEI_SIZE);
	if(dataread == NULL)
	{
		memset(android_usb_pdata.product_serial_number,0,MSM_SERIAL_NUMBER_SIZE);
		strcpy(android_usb_pdata.product_serial_number,"TG03000000");
		return;
	}
	memcpy( nv_imei_read, dataread, MSM_NV_UE_IMEI_SIZE);

	imei_bcd_len = nv_imei_read[0];
	if ( imei_bcd_len == 0 )
	{
		memset(android_usb_pdata.product_serial_number,0,MSM_SERIAL_NUMBER_SIZE);
		strcpy(android_usb_pdata.product_serial_number,"TG03000000");
	}
	else if ( imei_bcd_len <= ( MSM_NV_UE_IMEI_SIZE - 1 ) )
	{
		memset( imei_ascii, 0, MSM_OTHER_IMEI_ASCII_MAX_LEN );

		for( i = 1; i <= imei_bcd_len; i++ )
		{
			digit = nv_imei_read[ i ] & 0x0F;
			if( ( digit <= 9 ) || ( i <= 1 ) )
			{
				imei_ascii[ ( i - 1 ) * 2 ] = digit + '0';
			}
			else
			{
				imei_ascii[ ( i - 1 ) * 2 ] = '\0';
				break;
			}
			digit = nv_imei_read[ i ] >> 4;
			if ( ( digit <= 9 ) || ( i <= 1 ) )
			{
				imei_ascii[ ( ( i - 1 ) * 2 ) + 1 ] = digit + '0';
				/* Make sure NULL terminated */
				imei_ascii[ ( ( i - 1 ) * 2 ) + 2 ] = '\0';
			}
			else
			{
				imei_ascii[ ( ( i - 1 ) * 2 ) + 1 ] = '\0';
				break;
			}
		} /* end for */
		printk( "IMEI value 0x%s read from NV\n", imei_ascii );
		memset(android_usb_pdata.product_serial_number,0,MSM_SERIAL_NUMBER_SIZE);
		strcpy(android_usb_pdata.product_serial_number,"TG03");
		length = strlen(imei_ascii);
		while(j>=0)
		{
			temp_arr[j--] = imei_ascii[length -2];
			length --;
		}
		temp_arr[6]='\0';
		strcat(android_usb_pdata.product_serial_number,temp_arr);
	}
	/* This is a invalid IMEI*/
	else
	{
		memset(android_usb_pdata.product_serial_number,0,MSM_SERIAL_NUMBER_SIZE);
		strcpy(android_usb_pdata.product_serial_number,"TG03000000");
	}
}

#define SERIAL_NUMBER_SIZE_NV	8
#define SERIAL_NUMBER_SIZE	11

static void msm_read_nv_serial(void)
{
	uint8_t *dataread = NULL;
	uint8_t nv_serial_read[12] = {0};
	uint8_t nv_serial_read_temp[8] = {0};
	uint8_t byte1;
	uint8_t byte2;
	int i;

	dataread = smem_alloc(SMEM_OEM_027,SERIAL_NUMBER_SIZE_NV);
	if(dataread == NULL)
	{
		memset(android_usb_pdata.product_serial_number,0,SERIAL_NUMBER_SIZE_NV);
		strcpy(android_usb_pdata.product_serial_number,"TG030000");   //need to get the string to add
		return;
	}
	else
	{
		memcpy(nv_serial_read_temp, dataread, SERIAL_NUMBER_SIZE_NV);
		memcpy(nv_serial_read, dataread, SERIAL_NUMBER_SIZE_NV);
		for (i=0;i<3;i++)
		{
			byte1 = nv_serial_read_temp[5+i];
			byte1 = byte1 >> 4;
			byte1 = byte1 & 0x0F;
			byte1 = byte1 + 0x30;

			byte2 = nv_serial_read_temp[5+i] & 0x0F;
			byte2 = byte2 + 0x30;
			nv_serial_read [4 + ((2 * i) + 1)] = byte1;
			nv_serial_read [5 + ((2 * i) + 1)] = byte2;
		}
		nv_serial_read[11] = '\0';
		memset(android_usb_pdata.product_serial_number,0,SERIAL_NUMBER_SIZE);
		strcpy(android_usb_pdata.product_serial_number,nv_serial_read);
	}
	return;
}

int msm_usb_get_serial_number(void)
{
	msm_read_nv_serial();
	return 1;
}
EXPORT_SYMBOL(msm_usb_get_serial_number);

int msm_usb_read_nvitem(unsigned int id, unsigned short *data)
{
	int rc;
	if(data == NULL)
		return -1;

	/* read NV*/
	rc = msm_proc_comm(PCOM_NV_READ, &id,(unsigned *)data);
	if(rc)
		printk("NV read error %d\n",rc);

	return rc;
}
EXPORT_SYMBOL(msm_usb_read_nvitem);

int msm_usb_write_nvitem(unsigned int id, unsigned short *data)
{
	int rc;
	if( data == NULL )
		return -1;

	/* write VN */
	rc = msm_proc_comm(PCOM_NV_WRITE, &id, (unsigned *)data);
	if( rc )
		printk("NV write error %d %d\n",rc,id);

	return rc;
}
EXPORT_SYMBOL(msm_usb_write_nvitem);

