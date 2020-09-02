/*
  fcsmd Driver

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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/wakelock.h>

#include "smd_private.h"

MODULE_LICENSE("GPL");

#define DRIVER_NAME "fcsmd"
#define SMEM_OEM_023_SIZE 300
//#define FCSMD_DEBUG

static int fcsmd_dev_count = 1;
static int fcsmd_major = 0;
static int fcsmd_minor = 0;
module_param(fcsmd_major, int, 0);
static struct class *fcsmd_class = NULL;
static struct cdev fcsmd_cdev;
static char fcsmd_buf[SMEM_OEM_023_SIZE + 4];
struct wake_lock fcsmd_wakelock; /* prevent suspend state during felica transaction */

/*----------------------------------------------------------
 *	fcsmd_open
 *---------------------------------------------------------*/
static int fcsmd_open (struct inode *inode, struct file *filp)
{
	wake_lock(&fcsmd_wakelock);
	return 0;
}


/*----------------------------------------------------------
 *	fcsmd_write
 *---------------------------------------------------------*/
static int fcsmd_write(struct file * file, char * buff, size_t count, loff_t *pos)
{
	int ret = 0;

	unsigned char *smem_ptr=NULL;
	int i;

#ifdef FCSMD_DEBUG
	unsigned char *ptr = NULL;
	int byte = 0;
#endif

	if((file == NULL) || (buff == NULL) || (SMEM_OEM_023_SIZE < count) || (pos == NULL)){
		printk(KERN_ERR "fcsmd_write: parameter error \n");
		return -1;
	}

	ret = copy_from_user(fcsmd_buf, buff, count);
	if (ret) {
		printk(KERN_ERR "fcsmd_write: copy_from_user failed. ret=%d\n", ret);
		return -1;
	}

	smem_ptr = (unsigned char *)smem_alloc(SMEM_OEM_023, SMEM_OEM_023_SIZE); 
	if(smem_ptr == NULL){
		printk(KERN_ERR "fcsmd_write: failed to get SMEM_OEM_023=%x\n", SMEM_OEM_023);
		return -1;
	}

#ifdef FCSMD_DEBUG
	ptr = smem_ptr;
#endif

	for(i=0; i<count; i++){
		*smem_ptr++ = fcsmd_buf[i];
	}

#ifdef FCSMD_DEBUG
	for(i=0; i<count; i++){
		printk(KERN_INFO " %02x", *ptr++);
		if(byte==8){
			printk(KERN_INFO "\n");
			byte = 0;
		}
	}
	printk("\n");
#endif

	*pos += count;

	return count;
}
/*----------------------------------------------------------
 *	fcsmd_read
 *---------------------------------------------------------*/
static int fcsmd_read(struct file * file, char * buff, size_t count, loff_t *pos)
{
	int ret = 0;
	unsigned char *smem_ptr=NULL;

#ifdef FCSMD_DEBUG
	int i;
	int byte = 0;
	unsigned char *ptr = NULL;
#endif

	if((file == NULL) || (buff == NULL) || (SMEM_OEM_023_SIZE < count) || (pos == NULL)){
		printk(KERN_ERR "fcsmd_write: parameter error \n");
		return -1;
	}

	smem_ptr = (unsigned char *)smem_alloc(SMEM_OEM_023, SMEM_OEM_023_SIZE); 
	if(smem_ptr == NULL){
		printk(KERN_ERR "fcsmd_read: failed to get SMEM_OEM_023=%x\n", SMEM_OEM_023);
		return -1;
	}

#ifdef FCSMD_DEBUG
	ptr = smem_ptr;
	for(i=0; i<count; i++){
		printk(KERN_INFO " %02x", *ptr++);
		if(byte==8){
			printk(KERN_INFO "\n");
			byte = 0;
		}
	}
	printk("\n");
#endif

	ret = copy_to_user(buff, smem_ptr, count);
	if (ret) {
		printk(KERN_ERR "fcsmd_read: copy_to_user failed. ret=%d\n", ret);
		return -1;
	}
	*pos += count;

	return count;
}
/*----------------------------------------------------------
 *	fcsmd_release
 *---------------------------------------------------------*/
static int fcsmd_release(struct inode *inode, struct file *filp)
{
	wake_unlock(&fcsmd_wakelock);
	return 0;
}


/*
 * Our various sub-devices.
 */
static struct file_operations fcsmd_ops = {
	.owner   = THIS_MODULE,
	.open    = fcsmd_open,
	.release = fcsmd_release,
	.write   = fcsmd_write,
	.read    = fcsmd_read,
};

/*----------------------------------------------------------
 *	fcsmd_init
 *---------------------------------------------------------*/
static int fcsmd_init(void)
{
	dev_t dev = MKDEV(fcsmd_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
    struct class_device *class_dev = NULL;

	/*
	* register major number
	*/
	/* reserve major number */
	if (fcsmd_major) {
		alloc_ret = register_chrdev_region(dev, fcsmd_dev_count, DRIVER_NAME);
		if (alloc_ret < 0) {
			printk(KERN_ERR "fcsmd: unable to get major %d\n", fcsmd_major);
			goto error;
		}
		if (fcsmd_major == 0)
			fcsmd_major = alloc_ret;
	}
	else {
		alloc_ret = alloc_chrdev_region(&dev, fcsmd_minor, fcsmd_dev_count, DRIVER_NAME);
		if (alloc_ret) {
			printk(KERN_ERR "fcsmd: unable to get major \n");
			goto error;
		}
		fcsmd_major = MAJOR(dev);
	}

	/* register system call handler(fops) */
	cdev_init(&fcsmd_cdev, &fcsmd_ops);

	/* register to kernel */
	fcsmd_cdev.owner = THIS_MODULE;
	fcsmd_cdev.ops = &fcsmd_ops;
	cdev_err = cdev_add (&fcsmd_cdev, MKDEV(fcsmd_major, fcsmd_minor), fcsmd_dev_count);

	if (cdev_err) {
		goto error;
	}

	/* register class */
	fcsmd_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(fcsmd_class)) {
		goto error;
	}
	class_dev = device_create(fcsmd_class, NULL, MKDEV(fcsmd_major, fcsmd_minor), NULL, DRIVER_NAME);

	if (IS_ERR(class_dev))
		printk(KERN_ERR "fcsmd: can't create device\n");

	wake_lock_init(&fcsmd_wakelock, WAKE_LOCK_SUSPEND, "fcsmd");

	return 0;
  
error:
	if (cdev_err == 0)
		cdev_del(&fcsmd_cdev);
	
	if (alloc_ret == 0)
		unregister_chrdev_region(MKDEV(fcsmd_major, 0), fcsmd_dev_count);

	return -1;
}


/*----------------------------------------------------------
 *	fcsmd_exit
 *---------------------------------------------------------*/
static void fcsmd_exit(void)
{
	wake_lock_destroy(&fcsmd_wakelock);

	/* unregister class */
	device_destroy(fcsmd_class, MKDEV(fcsmd_major, 0));
	class_destroy(fcsmd_class);

	/* unregister device */
	cdev_del(&fcsmd_cdev);
	unregister_chrdev_region(MKDEV(fcsmd_major, 0), fcsmd_dev_count);
}

module_init(fcsmd_init);
module_exit(fcsmd_exit);
