/*
  sdrwctl Driver

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

#include "sdrwctl_ap.h"
#include "sdrwctl.h"

#define SDRWCTL_MAX_SIZE 64*1024
#define SDRWCTL_SEC_SIZE 512

static int sdrwctl_major = 0;
static int sdrwctl_minor = 0;
static struct class* sdrwctl_class;
static struct mmc_host *sd_host;
static struct sdrwctl_ops sd_ops;

static int sdrwctl_get_state(unsigned long arg)
{
    int val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    err = sd_ops.get_state(sd_host, &val);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_get_state err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &val, sizeof(unsigned short)))
        return -EFAULT;

    return err;
}
static int sdrwctl_get_info(unsigned long arg)
{
    struct sdrwctl_reg_info reg_info;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;
    err = sd_ops.get_info(sd_host, &reg_info);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_get_info err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &reg_info, sizeof(struct sdrwctl_reg_info)))
        return -EFAULT;

    return err;

}
static int sdrwctl_get_prostate(unsigned long arg)
{
    unsigned short val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    err = sd_ops.get_prostate(sd_host, &val);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_get_info err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &val, sizeof(unsigned short)))
        return -EFAULT;

    return err;
}
static int sdrwctl_set_state(unsigned long arg)
{
    unsigned short val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_READ, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;
    if(copy_from_user(&val, user_arg, sizeof(unsigned short)))
        return -EFAULT;

    err = sd_ops.set_state(sd_host, &val);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_set_state err = %d\n",err);
        return err;
    }

    return err;

}
static int sdrwctl_get_check(unsigned long arg)
{
    int val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    err = sd_ops.get_check(sd_host, &val);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_get_check err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &val, sizeof(unsigned short)))
        return -EFAULT;

    return err;
}
static int sdrwctl_write_reg(unsigned long arg)
{
    struct sdrwctl_reg_rw reg_write;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_READ, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    if(copy_from_user(&reg_write, user_arg, sizeof(struct sdrwctl_reg_rw)))
        return -EFAULT;

    err = sd_ops.write_reg(sd_host, &reg_write);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_write_reg err = %d\n",err);
        return err;
    }

    return err;
}
static int sdrwctl_read_reg(unsigned long arg)
{
    struct sdrwctl_reg_rw reg_read;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    if(copy_from_user(&reg_read, user_arg, sizeof(struct sdrwctl_reg_rw)))
        return -EFAULT;

    err = sd_ops.read_reg(sd_host, &reg_read);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_read_reg err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &reg_read, sizeof(struct sdrwctl_reg_rw)))
        return -EFAULT;

    return err;
}
static int sdrwctl_write_reg_mask(unsigned long arg)
{
    struct sdrwctl_reg_rw reg_write;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_READ, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;
    if(copy_from_user(&reg_write, user_arg, sizeof(struct sdrwctl_reg_rw)))
        return -EFAULT;

    err = sd_ops.write_reg_mas(sd_host, &reg_write);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_write_reg_mask err = %d\n",err);

    return err;
}
static int sdrwctl_write_reg_mul(unsigned long arg)
{
    struct sdrwctl_reg_rw_mul reg_write_mul;
    unsigned short *val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_READ, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    if(copy_from_user(&reg_write_mul, user_arg, sizeof(struct sdrwctl_reg_rw_mul)))
        return -EFAULT;

    if(reg_write_mul.cnt > SDRWCTL_MAX_SIZE)
    {
        printk(KERN_DEBUG "sdrwctl:sdrwctl_write_reg_mul err \n");
        return -EFAULT;
    }

    val = kmalloc(reg_write_mul.cnt, GFP_KERNEL);
    if(val == NULL)
    {
        printk(KERN_DEBUG "sdrwctl:kmalloc failed\n");
        return -EFAULT;
    }

    if(copy_from_user(val, reg_write_mul.val, reg_write_mul.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    reg_write_mul.val = val;
    err = sd_ops.write_reg_mul(sd_host, &reg_write_mul);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_write_reg err = %d\n",err);

    kfree(val);

    return err;
}
static int sdrwctl_read_reg_mul(unsigned long arg)
{
    struct sdrwctl_reg_rw_mul reg_read_mul;
    struct sdrwctl_reg_rw_mul out_reg_read;
    unsigned short *val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    if(copy_from_user(&reg_read_mul, user_arg, sizeof(struct sdrwctl_reg_rw_mul)))
        return -EFAULT;

    if(reg_read_mul.cnt > SDRWCTL_MAX_SIZE)
    {
        printk(KERN_DEBUG "sdrwctl:sdrwctl_read_reg_mul err \n");
        return -EFAULT;
    }

    val = kmalloc(reg_read_mul.cnt, GFP_KERNEL);
    if(val == NULL)
    {
        printk(KERN_DEBUG "sdrwctl:kmalloc failed\n");
        return -EFAULT;
    }

    if(copy_from_user(val, reg_read_mul.val, reg_read_mul.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    out_reg_read = reg_read_mul;
    out_reg_read.val = val;

    err = sd_ops.read_reg_mul(sd_host, &out_reg_read);
    if(err != 0){
        kfree(val);
        printk(KERN_ERR "sdrwctl_read_reg err = %d\n",err);
        return err;
    }

    if(copy_to_user(((struct sdrwctl_reg_rw_mul*)user_arg)->val, out_reg_read.val, out_reg_read.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    reg_read_mul.val = ((struct sdrwctl_reg_rw_mul*)user_arg)->val;

    if(copy_to_user(user_arg, &reg_read_mul, sizeof(struct sdrwctl_reg_rw_mul)))
    {
        kfree(val);
        return -EFAULT;
    }

    kfree(val);

    return err;
}
static int sdrwctl_get_ext(unsigned long arg)
{
    int val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    err = sd_ops.get_ext(sd_host, &val);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_get_ext err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &val, sizeof(unsigned short)))
        return -EFAULT;

    return err;
}

static int sdrwctl_write_secreg_mul(unsigned long arg)
{
    struct sdrwctl_reg_sec_rw reg_write_sec;
    unsigned short *val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_READ, user_arg, sizeof(long));
    if(!err){
        printk(KERN_DEBUG "write_secreg_mul access_ok err %d\n",err);
        return -EFAULT;
    }

    if(copy_from_user(&reg_write_sec, user_arg, sizeof(struct sdrwctl_reg_sec_rw)))
        return -EFAULT;

    if(reg_write_sec.cnt > SDRWCTL_MAX_SIZE)
    {
        printk(KERN_DEBUG "sdrwctl:sdrwctl_write_secreg_mul err \n");
        return -EFAULT;
    }

    val = kmalloc(reg_write_sec.cnt, GFP_KERNEL);
    if(val == NULL)
    {
        printk(KERN_DEBUG "sdrwctl:kmalloc failed\n");
        return -EFAULT;
    }

    if(copy_from_user(val, reg_write_sec.val, reg_write_sec.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    reg_write_sec.val = val;
    err = sd_ops.secure_write(sd_host, &reg_write_sec);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_write_reg err = %d\n",err);

    kfree(val);

    return err;
}
static int sdrwctl_read_secreg_mul(unsigned long arg)
{
    struct sdrwctl_reg_sec_rw reg_read_sec;
    struct sdrwctl_reg_sec_rw out_reg_read_sec;
    unsigned short *val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err){
        printk(KERN_DEBUG "read_secreg_mul access_ok err %d\n",err);
        return -EFAULT;
    }

    if(copy_from_user(&reg_read_sec, user_arg, sizeof(struct sdrwctl_reg_sec_rw)))
        return -EFAULT;

    if(reg_read_sec.cnt > SDRWCTL_MAX_SIZE)
    {
        printk(KERN_DEBUG "sdrwctl:sdrwctl_read_secreg_mul err \n");
        return -EFAULT;
    }

    val = kmalloc(reg_read_sec.cnt, GFP_KERNEL);
    if(val == NULL)
    {
        printk(KERN_DEBUG "sdrwctl:kmalloc failed\n");
        return -EFAULT;
    }

    if(copy_from_user(val, reg_read_sec.val, reg_read_sec.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    out_reg_read_sec = reg_read_sec;
    out_reg_read_sec.val = val;

    err = sd_ops.secure_read(sd_host, &out_reg_read_sec);
    if(err != 0){
        kfree(val);
        printk(KERN_ERR "sdrwctl_read_reg err = %d\n",err);
        return err;
    }

    if(copy_to_user(((struct sdrwctl_reg_sec_rw*)user_arg)->val, out_reg_read_sec.val, out_reg_read_sec.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    reg_read_sec.val = ((struct sdrwctl_reg_sec_rw*)user_arg)->val;

    if(copy_to_user(user_arg, &reg_read_sec, sizeof(struct sdrwctl_reg_sec_rw)))
    {
        kfree(val);
        return -EFAULT;
    }

    kfree(val);

    return err;
}
static int sdrwctl_trans_secreg_mul(unsigned long arg)
{
    struct sdrwctl_reg_sec_codec reg_trans_sec;
    struct sdrwctl_reg_sec_codec out_reg_trans_sec;
    unsigned short *val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err){
        printk(KERN_DEBUG "trans_secreg_mul access_ok err %d\n",err);
        return -EFAULT;
    }

    if(copy_from_user(&reg_trans_sec, user_arg, sizeof(struct sdrwctl_reg_sec_codec)))
        return -EFAULT;

    if(reg_trans_sec.cnt > SDRWCTL_MAX_SIZE)
    {
        printk(KERN_DEBUG "sdrwctl:sdrwctl_trans_secreg_mul err \n");
        return -EFAULT;
    }

    val = kmalloc(reg_trans_sec.cnt, GFP_KERNEL);
    if(val == NULL)
    {
        printk(KERN_DEBUG "sdrwctl:kmalloc failed\n");
        return -EFAULT;
    }

    if(copy_from_user(val, reg_trans_sec.val, reg_trans_sec.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    out_reg_trans_sec = reg_trans_sec;
    out_reg_trans_sec.val = val;

    err = sd_ops.secure_codec(sd_host, &out_reg_trans_sec);
    if(err != 0){
        kfree(val);
        printk(KERN_ERR "sdrwctl_trans_reg err = %d\n",err);
        return err;
    }

    if(copy_to_user(((struct sdrwctl_reg_sec_codec*)user_arg)->val, out_reg_trans_sec.val, out_reg_trans_sec.cnt))
    {
        kfree(val);
        return -EFAULT;
    }

    reg_trans_sec.val = ((struct sdrwctl_reg_sec_codec*)user_arg)->val;

    if(copy_to_user(user_arg, &reg_trans_sec, sizeof(struct sdrwctl_reg_sec_codec)))
    {
        kfree(val);
        return -EFAULT;
    }

    kfree(val);

    return err;
}
static int sdrwctl_get_check_clr(unsigned long arg)
{
    int err = 0;

    err = sd_ops.flg_init(sd_host);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_get_check_clr err = %d\n",err);

    return err;
}
static int sdrwctl_lock(unsigned long arg)
{
    int err = 0;

    err = sd_ops.lock(sd_host);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_lock err = %d\n",err);

    return err;
}
static int sdrwctl_unlock(unsigned long arg)
{
    int err = 0;

    err = sd_ops.unlock(sd_host);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_unlock err = %d\n",err);

    return err;
}
static int sdrwctl_cmd(unsigned long arg)
{
    int err = 0;
    void __user *user_arg = (void __user *)arg;
    struct sdrwctl_reg_cmd_exec cmd_exec;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    if(copy_from_user(&cmd_exec, user_arg, sizeof(struct sdrwctl_reg_cmd_exec)))
        return -EFAULT;

    err = sd_ops.cmd(sd_host, &cmd_exec);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_cmd cmd = %d\n",err);

    return err;
}
static int sdrwctl_initialize(unsigned long arg)
{
    int err = 0;

    err = sd_ops.init(sd_host);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_initialize err = %d\n",err);

    return err;
}

static int sdrwctl_deinitialize(unsigned long arg)
{
    int err = 0;

    err = sd_ops.deinit(sd_host);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_deinitialize err = %d\n",err);

    return err;
}
static int sdrwctl_get_bit(unsigned long arg)
{
    int val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    err = sd_ops.get_bit(sd_host, &val);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_get_bit err = %d\n",err);
        return err;
    }

    if(copy_to_user(user_arg, &val, sizeof(int)))
        return -EFAULT;

    return err;
}
static int sdrwctl_set_bit(unsigned long arg)
{
    int val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_READ, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;
    if(copy_from_user(&val, user_arg, sizeof(int)))
        return -EFAULT;

    err = sd_ops.put_bit(sd_host, &val);
    if(err != 0){
        printk(KERN_ERR "sdrwctl_set_bit err = %d\n",err);
        return err;
    }

    return err;
}
static int sdrwctl_write_reg_sec(unsigned long arg)
{
    struct sdrwctl_reg_rw_mul reg_write_mul;
    unsigned short *val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_READ, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    if(copy_from_user(&reg_write_mul, user_arg, sizeof(struct sdrwctl_reg_rw_mul)))
        return -EFAULT;

    val = kmalloc(SDRWCTL_SEC_SIZE, GFP_KERNEL);
    if(val == NULL)
    {
        printk(KERN_DEBUG "sdrwctl:kmalloc failed\n");
        return -EFAULT;
    }

    if(copy_from_user(val, reg_write_mul.val, SDRWCTL_SEC_SIZE))
    {
        kfree(val);
        return -EFAULT;
    }

    reg_write_mul.val = val;
    err = sd_ops.write_reg_sec(sd_host, &reg_write_mul);
    if(err != 0)
        printk(KERN_ERR "sdrwctl_write_reg err = %d\n",err);

    kfree(val);

    return err;
}
static int sdrwctl_read_reg_sec(unsigned long arg)
{
    struct sdrwctl_reg_rw_mul reg_read_mul;
    struct sdrwctl_reg_rw_mul out_reg_read;
    unsigned short *val;
    int err = 0;
    void __user *user_arg = (void __user *)arg;

    err = access_ok(VERIFY_WRITE, user_arg, sizeof(long));
    if(!err)
        return -EFAULT;

    if(copy_from_user(&reg_read_mul, user_arg, sizeof(struct sdrwctl_reg_rw_mul)))
        return -EFAULT;

    val = kmalloc(SDRWCTL_SEC_SIZE, GFP_KERNEL);
    if(val == NULL)
    {
        printk(KERN_DEBUG "sdrwctl:kmalloc failed\n");
        return -EFAULT;
    }

    if(copy_from_user(val, reg_read_mul.val, SDRWCTL_SEC_SIZE))
    {
        kfree(val);
        return -EFAULT;
    }

    out_reg_read = reg_read_mul;
    out_reg_read.val = val;

    err = sd_ops.read_reg_sec(sd_host, &out_reg_read);
    if(err != 0){
        kfree(val);
        printk(KERN_ERR "sdrwctl_read_reg_sec err = %d\n",err);
        return err;
    }

    if(copy_to_user(((struct sdrwctl_reg_rw_mul*)user_arg)->val, out_reg_read.val, SDRWCTL_SEC_SIZE))
    {
        kfree(val);
        return -EFAULT;
    }

    reg_read_mul.val = ((struct sdrwctl_reg_rw_mul*)user_arg)->val;

    if(copy_to_user(user_arg, &reg_read_mul, sizeof(struct sdrwctl_reg_rw_mul)))
    {
        kfree(val);
        return -EFAULT;
    }

    kfree(val);

    return err;
}
static atomic_t ioctl_available = ATOMIC_INIT(1);

static int sdrwctl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int retval = 0;

    if(! atomic_dec_and_test(&ioctl_available))
    {
        atomic_inc(&ioctl_available);
        return -EBUSY;
    }

    switch(cmd){
        case SDRWCTL_GET_STATE:
            retval = sdrwctl_get_state(arg);
            break;
        case SDRWCTL_GET_INFO:
            retval = sdrwctl_get_info(arg);
            break;
        case SDRWCTL_GET_PROSTATE:
            retval = sdrwctl_get_prostate(arg);
            break;
        case SDRWCTL_SET_STATE:
            retval = sdrwctl_set_state(arg);
            break;
        case SDRWCTL_GET_CHECK:
            retval = sdrwctl_get_check(arg);
            break;
        case SDRWCTL_WRITE_REG16:
            retval = sdrwctl_write_reg(arg);
            break;
        case SDRWCTL_READ_REG16:
            retval = sdrwctl_read_reg(arg);
            break;
        case SDRWCTL_WRITE_REG16_MASK:
            retval = sdrwctl_write_reg_mask(arg);
            break;
        case SDRWCTL_WRITE_REG16_MUL:
            retval = sdrwctl_write_reg_mul(arg);
            break;
        case SDRWCTL_READ_REG16_MUL:
            retval = sdrwctl_read_reg_mul(arg);
            break;
        case SDRWCTL_GET_EXT:
            retval = sdrwctl_get_ext(arg);
            break;
        case SDRWCTL_WRITE_SECREG16_MUL:
            retval = sdrwctl_write_secreg_mul(arg);
            break;
        case SDRWCTL_READ_SECREG16_MUL:
            retval = sdrwctl_read_secreg_mul(arg);
            break;
        case SDRWCTL_GET_CHECK_CLR:
            retval = sdrwctl_get_check_clr(arg);
            break;
        case SDRWCTL_LOCK:
            retval = sdrwctl_lock(arg);
            break;
        case SDRWCTL_UNLOCK:
            retval = sdrwctl_unlock(arg);
            break;
        case SDRWCTL_CMD:
            retval = sdrwctl_cmd(arg);
            break;
        case SDRWCTL_INIT:
            retval = sdrwctl_initialize(arg);
            break;
        case SDRWCTL_DEINIT:
            retval = sdrwctl_deinitialize(arg);
            break;
        case SDRWCTL_TRANS_SECREG16_MUL:
            retval = sdrwctl_trans_secreg_mul(arg);
            break;
        case SDRWCTL_GET_BIT:
            retval = sdrwctl_get_bit(arg);
            break;
        case SDRWCTL_SET_BIT:
            retval = sdrwctl_set_bit(arg);
            break;
        case SDRWCTL_WRITE_REG16_SEC:
            retval = sdrwctl_write_reg_sec(arg);
            break;
        case SDRWCTL_READ_REG16_SEC:
            retval = sdrwctl_read_reg_sec(arg);
            break;
        default:
            printk(KERN_ERR "sdrwctl:invalid data\n");
            retval = -EINVAL;
            break;
    }

    atomic_inc(&ioctl_available);

    return retval;
}

static atomic_t open_available = ATOMIC_INIT(1);

static ssize_t sdrwctl_open(struct inode *inode, struct file *file)
{
    if(! atomic_dec_and_test(&open_available))
    {
        atomic_inc(&open_available);
        return -EBUSY;
    }

    return 0;
}

static ssize_t sdrwctl_close(struct inode *inode, struct file *file)
{
    atomic_inc(&open_available);

    return 0;
}

int sdrwctl_reg_func(struct mmc_host *host, struct sdrwctl_ops *ops)
{
    sd_host = host;
    memcpy(&sd_ops, ops, sizeof(struct sdrwctl_ops));

    return 0;
}
EXPORT_SYMBOL(sdrwctl_reg_func);

int sdrwctl_rel_func(struct mmc_host *host)
{
    sd_host = NULL;

    return 0;
}
EXPORT_SYMBOL(sdrwctl_rel_func);

static struct file_operations sdrwctl_fops = {
    .owner = THIS_MODULE,
    .ioctl = sdrwctl_ioctl,
    .open = sdrwctl_open,
    .release = sdrwctl_close,
};

MODULE_LICENSE("GPL");

static struct cdev sdrwctl_cdev;
static dev_t sdrwct_dev;

static int __init sdrwctl_init(void)
{
    int result = 0;
    int dev_err = 0;
    dev_t dev;
        struct device *device;

    if (sdrwctl_major) {
        result = register_chrdev_region(sdrwct_dev, 1, "sdrwctl");
    }
    else {
        result = alloc_chrdev_region(&sdrwct_dev, 0, 1, "sdrwctl");
        sdrwctl_major = MAJOR(sdrwct_dev);
    }

    if (result < 0) {
        printk(KERN_WARNING "sdrwctl: unable to get major %d\n", sdrwctl_major);
        return result;
    }
    if (sdrwctl_major == 0)
        sdrwctl_major = result;

    cdev_init(&sdrwctl_cdev,&sdrwctl_fops);

    dev = MKDEV(sdrwctl_major, 0);
    sdrwctl_cdev.owner = THIS_MODULE;
    sdrwctl_cdev.ops = &sdrwctl_fops;
    dev_err = cdev_add(&sdrwctl_cdev,dev,1);

    if (dev_err)
        printk (KERN_NOTICE "sdrwctl_init:cdev_add err = %d \n", dev_err);

    sdrwctl_class = class_create(THIS_MODULE, "sdrwctl");
    if(IS_ERR(sdrwctl_class)) {
        printk(KERN_ERR"sdrwctl:class_create err \n");
        return PTR_ERR(sdrwctl_class);
    }

    device = device_create(sdrwctl_class, NULL, dev, NULL, "sdrwctl");
    if (IS_ERR(device))
        printk(KERN_ERR "sdrwctl: can't create device\n");

    return 0;
}

static void __exit sdrwctl_exit(void)
{
    device_destroy(sdrwctl_class, MKDEV(sdrwctl_major, 0));
    class_destroy(sdrwctl_class);
    cdev_del(&sdrwctl_cdev);
    unregister_chrdev_region(MKDEV(sdrwctl_major, 0), 1);
}

module_init(sdrwctl_init);
module_exit(sdrwctl_exit);
