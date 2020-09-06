/*
  USB serial notification Driver

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

#include <linux/module.h>	
#include <linux/kernel.h>	
#include <linux/proc_fs.h>	
#include <asm/uaccess.h>	
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/workqueue.h>

#define PROCFS_MAX_SIZE		128
#define PROCFS_NAME 		"serial_noti"

#define DRIVER_NAME         "switch_GS0"
static struct switch_dev               *notification;

static int usb_status;
int serial_online;


extern int usb_charger_detection_state(void);
void serial_notification_switch_on(void);
void serial_notification_switch_off(void);
int for_serial_notification(void);

static struct proc_dir_entry *Our_Proc_File;


static char procfs_buffer[PROCFS_MAX_SIZE];


static unsigned long procfs_buffer_size = 0;


static int 
procfile_read(char *buffer,
		char **buffer_location,
		off_t offset, int buffer_length, int *eof, void *data)
{
	int ret=0;

	printk("Current status: %s\nwrite \"serial-on\" or \"serial-off\" to change high sped serial notification\n", (serial_online ? "online" : "offline"));
	
	return ret;
}

static void do_onchange_notification_tasklet(struct work_struct *work)
{
	if(usb_status == 1)
		switch_set_state(notification, 1);
}
DECLARE_WORK(serial_notification_onstatus_tasklet, do_onchange_notification_tasklet);

static void do_offchange_notification_tasklet(struct work_struct *work)
{
	switch_set_state(notification, 0);
}
DECLARE_WORK(serial_notification_offstatus_tasklet, do_offchange_notification_tasklet);

void serial_notification_switch_off()
{
	schedule_work(&serial_notification_offstatus_tasklet);
}

void serial_notification_switch_on()
{
	schedule_work(&serial_notification_onstatus_tasklet);
}

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s-%s\n", DRIVER_NAME, ((serial_online && usb_status) ? "online" : "offline"));
}

void gserial_datacable_connected(void);
void gserial_datacable_disconnected(void);

static void do_usbdisconnect_notification_tasklet(struct work_struct *work)
{
	usb_status=0;
	switch_set_state(notification, 0);
}

static void do_usbconnect_notification_tasklet(struct work_struct *work)
{
	int ac_charger=-1;
	

	ac_charger = usb_charger_detection_state();
	switch(ac_charger)
	{
		case 0: 
			
			usb_status=1;
			if(serial_online == 1)
				switch_set_state(notification, 1);
			else
				switch_set_state(notification, 0);
			break;
		case 1:
			
			usb_status=0;
			break;
		default:
			
			usb_status=0;
	}
}

DECLARE_WORK(usb_disconnect_tasklet, do_usbdisconnect_notification_tasklet);
DECLARE_WORK(usb_connect_tasklet, do_usbconnect_notification_tasklet);


void gserial_datacable_connected()
{
	schedule_work(&usb_connect_tasklet);
}


void gserial_datacable_disconnected()
{
	schedule_work(&usb_disconnect_tasklet);
}


static int procfile_write(struct file *file, const char *buffer, unsigned long count,
		void *data)
{
	
	procfs_buffer_size = count;
	if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
		procfs_buffer_size = PROCFS_MAX_SIZE;
	}

	
	if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
		return -EFAULT;
	}

	if(!strncmp(buffer,"on",2)){
		serial_online=1;
#if 0
		if(usb_status == 1){		
			serial_notification_switch_on();
		}else{
			serial_notification_switch_off();
		}
#endif
	} else if (!strncmp(buffer,"off",3)) {
		serial_online=0;

	}

	return procfs_buffer_size;
}


int for_serial_notification()
{

	int rc;
	
	Our_Proc_File = create_proc_entry(PROCFS_NAME, S_IRUGO | S_IWUSR | S_IWGRP, NULL);

	if (Our_Proc_File == NULL) {
		remove_proc_entry(PROCFS_NAME, NULL);
		return -ENOMEM;
	}
	
	
	notification = kzalloc(sizeof *notification, GFP_KERNEL);
	if (!notification)
		return -ENOMEM;
	
	notification->name = DRIVER_NAME;
	notification->print_name = print_switch_name;
	notification->print_state = print_switch_state;
	rc = switch_dev_register(notification);
	if (rc < 0){
		printk("Error:Failed to register with notification changing event: uevent\n");
		printk("Error:Please look at source code: kernel/drivers/usb/gadget/serial_notification.c\n");
		printk("Error:USB connection Notification cannot be seen in kernel socket on userlevel application\n");
		
		kfree(notification);
		remove_proc_entry(PROCFS_NAME, NULL);
		printk(KERN_ALERT "Error: Could not register %s\n", "switch_dev");
		return -ENOMEM;
	}


	Our_Proc_File->read_proc  = procfile_read;
	Our_Proc_File->write_proc = procfile_write;

	Our_Proc_File->uid 	  = 0;
	Our_Proc_File->gid 	  = 0;
	Our_Proc_File->size 	  = 37;

	return 0;	
}
#if 0

static void __exit notification_exit(void)
{
	switch_dev_unregister(notification);
	remove_proc_entry(PROCFS_NAME, NULL);
	printk(KERN_INFO "/proc/%s removed\n", PROCFS_NAME);
	kfree(notification);
}

module_init(notification_init);
module_exit(notification_exit);
#endif
