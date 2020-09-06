/*
  stschk Driver

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

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/mmc/host.h>

#include "stschk.h"
#include "proc_comm.h"

#define SYSTEM_BLOCK_NAME "mtdblock1"

#define NV_TSB_TOP_ITEMS_I         10000
#define NV_BOOT_OPTIONS_I          (NV_TSB_TOP_ITEMS_I+96)
#define NV_SYSTEM_CHANGED_I        (NV_TSB_TOP_ITEMS_I+112)

static int stschk_major = 0;
static struct class* stschk_class;


int stschk_get_flag(int *val)
{
    int rc = 0;
    unsigned boot_option = 0;
    unsigned system_changed = 0;
    unsigned type_boot_option = NV_BOOT_OPTIONS_I;
    unsigned type_system_changed = NV_SYSTEM_CHANGED_I;

    rc = msm_proc_comm(PCOM_NV_READ, &type_boot_option, &boot_option);
    if(rc != 0){
        printk(KERN_ERR "[stschk]msm_proc_comm() falied! in %s()\n", __func__);
        return 1;
    }

    rc = msm_proc_comm(PCOM_NV_READ, &type_system_changed, &system_changed);
    if(rc != 0){
        printk(KERN_ERR "[stschk]msm_proc_comm() falied! in %s()\n", __func__);
        return 1;
    }
    *val = system_changed;
    return 0;
}
EXPORT_SYMBOL(stschk_get_flag);

int stschk_set_flag(int val)
{
    int rc = 0;
    unsigned boot_option = 0;
    unsigned type_boot_option = NV_BOOT_OPTIONS_I;
    unsigned type_system_changed = NV_SYSTEM_CHANGED_I;

    rc = msm_proc_comm(PCOM_NV_READ, &type_boot_option, &boot_option);
    if(rc != 0){
        printk(KERN_ERR "[stschk]msm_proc_comm() falied! in %s()\n", __func__);
        return 1;
    }

    if (boot_option != 0x20) {
        rc = msm_proc_comm(PCOM_NV_WRITE, &type_system_changed, &val);
        if(rc != 0){
            printk(KERN_ERR "[stschk]msm_proc_comm() falied! in %s()\n", __func__);
            return 1;
        }
    }

    return 0;
}
EXPORT_SYMBOL(stschk_set_flag);

int stschk_check_system_block(struct dentry *dentry)
{
    int rc = 0;
    char *superblock_name;
    unsigned system_changed = 0;
    unsigned type_system_changed = NV_SYSTEM_CHANGED_I;

    if (dentry == NULL) {
        return 0;
    }

    superblock_name = dentry->d_sb->s_id;

    if (!strcmp(superblock_name, SYSTEM_BLOCK_NAME)) {
        rc = msm_proc_comm(PCOM_NV_READ, &type_system_changed, &system_changed);
        if(rc != 0){
            printk(KERN_ERR "[stschk]msm_proc_comm() falied! in %s()\n", __func__);
            return 1;
        }

        if (system_changed != 1) {
            return stschk_set_flag(1);
        }
    }

    return 0;
}
EXPORT_SYMBOL(stschk_check_system_block);

static int stschk_get_state(unsigned long arg)
{
    int val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    err = stschk_get_flag(&val);
    if(err != 0){
        printk(KERN_ERR "stschk_get_state err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &val, sizeof(unsigned short)))
        return -EFAULT;

    return err;
}

static atomic_t ioctl_available = ATOMIC_INIT(1);

static int stschk_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int retval = 0;

    if(! atomic_dec_and_test(&ioctl_available))
    {
        atomic_inc(&ioctl_available);
        return -EBUSY;
    }

    switch(cmd){
        case STSCHK_GET_STATE:
            retval = stschk_get_state(arg);
            break;
        default:
            printk(KERN_ERR "stschk:invalid data\n");
            retval = -EINVAL;
            break;
    }

    atomic_inc(&ioctl_available);

    return retval;
}

static atomic_t open_available = ATOMIC_INIT(1);

static ssize_t stschk_open(struct inode *inode, struct file *file)
{
    if(! atomic_dec_and_test(&open_available))
    {
        atomic_inc(&open_available);
        return -EBUSY;
    }

    return 0;
}

static ssize_t stschk_close(struct inode *inode, struct file *file)
{
    atomic_inc(&open_available);

    return 0;
}

static struct file_operations stschk_fops = {
    .owner = THIS_MODULE,
    .ioctl = stschk_ioctl,
    .open = stschk_open,
    .release = stschk_close,
};

MODULE_LICENSE("GPL");

static struct cdev stschk_cdev;
static dev_t stschk_dev;

static int __init stschk_init(void)
{
    int result = 0;
    int dev_err = 0;
    dev_t dev;
    struct device *device;


    if (stschk_major) {
        result = register_chrdev_region(stschk_dev, 1, "stschk");
    }
    else {
        result = alloc_chrdev_region(&stschk_dev, 0, 1, "stschk");
        stschk_major = MAJOR(stschk_dev);
    }

    if (result < 0) {
        printk(KERN_WARNING "stschk: unable to get major %d\n", stschk_major);
        return result;
    }
    if (stschk_major == 0)
        stschk_major = result;

    cdev_init(&stschk_cdev,&stschk_fops);

    dev = MKDEV(stschk_major, 0);
    stschk_cdev.owner = THIS_MODULE;
    stschk_cdev.ops = &stschk_fops;
    dev_err = cdev_add(&stschk_cdev,dev,1);

    if (dev_err)
        printk (KERN_NOTICE "stschk_init:cdev_add err = %d \n", dev_err);

    stschk_class = class_create(THIS_MODULE, "stschk");
    if(IS_ERR(stschk_class)) {
        printk(KERN_ERR"stschk:class_create err \n");
        return PTR_ERR(stschk_class);
    }

    device = device_create(stschk_class, NULL, dev, NULL, "stschk");
    if (IS_ERR(device))
        printk(KERN_ERR "stschk: can't create device\n");

    return 0;
}

static void __exit stschk_exit(void)
{
    device_destroy(stschk_class, MKDEV(stschk_major, 0));
    class_destroy(stschk_class);
    cdev_del(&stschk_cdev);
    unregister_chrdev_region(MKDEV(stschk_major, 0), 1);
}

module_init(stschk_init);
module_exit(stschk_exit);
