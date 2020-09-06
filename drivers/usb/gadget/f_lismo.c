/*
  For Lismo USB serial function Driver

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
/*
 * f_lismo.c - USB serial function driver for KDDI lismo application
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>

#include "u_serial.h"
#include "gadget_chips.h"

extern void serial_notification_switch_on(void);
extern void serial_notification_switch_off(void);
extern int serial_online; 



struct lismo_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct f_gser {
	struct gserial			port;
	u8				data_id;
	u8				port_num;
	struct lismo_descs		fs;
	struct lismo_descs		hs;
	u8				online;
};

static inline struct f_gser *func_to_gser(struct usb_function *f)
{
	return container_of(f, struct f_gser, port.func);
}

/*-------------------------------------------------------------------------*/



static struct usb_cdc_header_desc header_desc = {
	.bLength            =	sizeof(struct usb_cdc_header_desc),
	.bDescriptorType    =	0x24,
	.bDescriptorSubType =	0x00,
	.bcdCDC             =	__constant_cpu_to_le16(0x0110),
};


static struct usb_interface_descriptor kmmo_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bNumEndpoints =	2,
	.bInterfaceClass =	0x02,  
	.bInterfaceSubClass =	0x0A,  
	.bInterfaceProtocol =	0x01,  
};


static struct usb_cdc_mdlm_desc mdlm_desc = {
	.bLength = sizeof(struct usb_cdc_mdlm_desc),
	.bDescriptorType = 0x24,
	.bDescriptorSubType = 0x12,
	.bcdVersion = __constant_cpu_to_le16(0x0100),
	.bGUID[0] = 0xC2,
	.bGUID[1] = 0x29,
	.bGUID[2] = 0x9F,
	.bGUID[3] = 0xCC,
	.bGUID[4] = 0xD4,
	.bGUID[5] = 0x89,
	.bGUID[6] = 0x40,
	.bGUID[7] = 0x66,
	.bGUID[8] = 0x89,
	.bGUID[9] = 0x2B,
	.bGUID[10] = 0x10,
	.bGUID[11] = 0xC3,
	.bGUID[12] = 0x41,
	.bGUID[13] = 0xDD,
	.bGUID[14] = 0x98,
	.bGUID[15] = 0xA9,
};

static struct usb_endpoint_descriptor lismo_fs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor lismo_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *lismo_fs_function[] = {
	(struct usb_descriptor_header *) &kmmo_interface_desc,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &mdlm_desc,
	(struct usb_descriptor_header *) &lismo_fs_in_desc,
	(struct usb_descriptor_header *) &lismo_fs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor lismo_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor lismo_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *lismo_hs_function[] = {
	(struct usb_descriptor_header *) &kmmo_interface_desc,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &mdlm_desc,
	(struct usb_descriptor_header *) &lismo_hs_in_desc,
	(struct usb_descriptor_header *) &lismo_hs_out_desc,
	NULL,
};



static struct usb_string lismo_string_defs[] = {
	[0].s = "Lismo Serial Gadget Port",
	{  } 
};

static struct usb_gadget_strings lismo_string_table = {
	.language =		0x0409,
	.strings =		lismo_string_defs,
};

static struct usb_gadget_strings *lismo_strings[] = {
	&lismo_string_table,
	NULL,
};

static int lismo_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_gser		 *gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;



	if (gser->port.in->driver_data) {
		DBG(cdev, "reset lismo ttyGS%d\n", gser->port_num);
		printk("KDDI lismo_set_alt reset lismo ttyGS%d\n", gser->port_num);
		gserial_disconnect(&gser->port);
	} else {
		DBG(cdev, "activate lismo ttyGS%d\n", gser->port_num);
		printk("KDDI lismo_set_alt activate lismo ttyGS%d\n", gser->port_num);
		gser->port.in_desc = ep_choose(cdev->gadget,
				gser->hs.in, gser->fs.in);
		gser->port.out_desc = ep_choose(cdev->gadget,
				gser->hs.out, gser->fs.out);
	}
	gserial_connect(&gser->port, gser->port_num);
	gser->online = 1;
#if 1
	serial_online=1;
	serial_notification_switch_on();
#endif
	return 0;
}

static void lismo_disable(struct usb_function *f)
{
	struct f_gser	         *gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

#if 1
	serial_online=0;
	serial_notification_switch_off();
#endif
	DBG(cdev, "lismo ttyGS%d deactivated\n", gser->port_num);
	printk("KDDI lismo_disable lismo ttyGS%d deactivated\n", gser->port_num);
	gserial_disconnect(&gser->port);
	gser->online = 0;
}

/*-------------------------------------------------------------------------*/



static int
lismo_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser            *gser = func_to_gser(f);
	int			 status;
	struct usb_ep		 *ep;


	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;
	kmmo_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;


	ep = usb_ep_autoconfig(cdev->gadget, &lismo_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;
	ep->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, &lismo_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;
	ep->driver_data = cdev;

	
	f->descriptors = usb_copy_descriptors(lismo_fs_function);

	gser->fs.in = usb_find_endpoint(lismo_fs_function,
			f->descriptors, &lismo_fs_in_desc);
	gser->fs.out = usb_find_endpoint(lismo_fs_function,
			f->descriptors, &lismo_fs_out_desc);


	if (gadget_is_dualspeed(c->cdev->gadget)) {
		lismo_hs_in_desc.bEndpointAddress =
				lismo_fs_in_desc.bEndpointAddress;
		lismo_hs_out_desc.bEndpointAddress =
				lismo_fs_out_desc.bEndpointAddress;

		
		f->hs_descriptors = usb_copy_descriptors(lismo_hs_function);

		gser->hs.in = usb_find_endpoint(lismo_hs_function,
				f->hs_descriptors, &lismo_hs_in_desc);
		gser->hs.out = usb_find_endpoint(lismo_hs_function,
				f->hs_descriptors, &lismo_hs_out_desc);
	}

	DBG(cdev, "lismo ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser->port.in->name, gser->port.out->name);
	printk( "KDDI lismo_bind lismo ttyGS%d: %s speed IN/%s OUT/%s\n",
                        gser->port_num,
                        gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
                        gser->port.in->name, gser->port.out->name);
#if 1
	serial_online=1;
#endif
	serial_notification_switch_on();
	return 0;

fail:

	if (gser->port.out)
		gser->port.out->driver_data = NULL;
	if (gser->port.in)
		gser->port.in->driver_data = NULL;


	printk("%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
lismo_unbind(struct usb_configuration *c, struct usb_function *f)
{

	serial_online=0;
	serial_notification_switch_off();

	
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
#if 0   
	serial_notification_switch_off();
#endif  
	kfree(func_to_gser(f));
}


int lismo_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_gser *gser;
	int		status;



	if (lismo_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		lismo_string_defs[0].id = status;
	}


	gser = kzalloc(sizeof *gser, GFP_KERNEL);
	if (!gser)
		return -ENOMEM;

	gser->port_num = port_num;

	gser->port.func.name = "gser";
	gser->port.func.strings = lismo_strings;
	gser->port.func.bind = lismo_bind;
	gser->port.func.unbind = lismo_unbind;
	gser->port.func.set_alt = lismo_set_alt;
	gser->port.func.disable = lismo_disable;
	status = usb_add_function(c, &gser->port.func);
	printk( "KDDI lismo_bind_config lismo ttyGS%d\n",gser->port_num);
	if (status)
		kfree(gser);
	return status;
}
