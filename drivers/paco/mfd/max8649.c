/*
  max8649 Driver

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
#include <linux/i2c.h>
#include <linux/delay.h>
#include "smd_private.h"
#include "max8649.h"

/* MAX8649 Register */
#define MAX8649_REG_MODE0     0x00
#define MAX8649_REG_MODE1     0x01
#define MAX8649_REG_MODE2     0x02
#define MAX8649_REG_MODE3     0x03
#define MAX8649_REG_CONTROL   0x04
#define MAX8649_REG_SYNC      0x05
#define MAX8649_REG_RAMP      0x06
#define MAX8649_REG_CHIP_ID1  0x08
#define MAX8649_REG_CHIP_ID2  0x09

#define MAX8649_MIN_LEVEL     750     /* 750 mV */
#define MAX8649_MAX_LEVEL     1380    /* 750 mV */
#define MAX8649_STEPS         10      /* 10 mV steps */

#define MAX8952_MIN_LEVEL     770     /* 770 mV */
#define MAX8952_MAX_LEVEL     1400    /* 140 mV */

#define IC_MAX8649            1
#define IC_MAX8952            2

#define MAX8649_OUT_V_MASK    0x0000003F  /* Output voltage mask */

#define MAX8649_FORCED_PWM_MODE 0x80

static struct i2c_client *maxclient = NULL;
static int Minvolts = 0;
static int Maxvolts = 0;

/**
 * max8649_set_dcdc2_level -
* @mvolts: voltage(750mV - 1380mV)
 *
 * Returns negative errno, else zero on success.
 */
int max8649_set_dcdc2_level(int mvolts)
{
    u8 val;
    int ret;

    if (!maxclient){
        printk(KERN_ERR "MAX8649 client is NULL!!\n");
        return -ENODEV;
    }
    if (mvolts < Minvolts || mvolts > Maxvolts){
        printk(KERN_ERR "MAX8649 Invalid voltage(%d mV)!!\n", mvolts);
        return -EINVAL;
    }

    val = (u8)((((mvolts - Minvolts) + (MAX8649_STEPS - 1)) / MAX8649_STEPS) & MAX8649_OUT_V_MASK);

    val |= MAX8649_FORCED_PWM_MODE;

    ret = i2c_smbus_write_byte_data(maxclient, MAX8649_REG_MODE2, val);

    if(ret != 0){
        /* error */
        printk(KERN_ERR "MAX8649 i2c write error!!\n");
    }

    return ret;
}
EXPORT_SYMBOL(max8649_set_dcdc2_level);

/**
 * max8649_get_dcdc2_level -
* @mvolts: voltage(750mV - 1380mV)
 *
 * Returns negative errno, else zero on success.
 */
int max8649_get_dcdc2_level(int *mvolts)
{
    int val;
    int ret;

    if (!maxclient){
        printk(KERN_ERR "MAX8649 client is NULL!!\n");
        return -ENODEV;
    }

    if(!mvolts){
        printk(KERN_ERR "MAX8649 mvolts is NULL!!\n");
        return -EINVAL;
    }

    val = i2c_smbus_read_byte_data(maxclient, MAX8649_REG_MODE2);

    if(val < 0){
        /* error */
        printk(KERN_ERR "MAX8649 i2c read error!!\n");
        ret = val;
    }else{
        val &= MAX8649_OUT_V_MASK;
        *mvolts = (val * MAX8649_STEPS) + Minvolts;
        /* success */
        ret = 0;
    }

    return ret;
}
EXPORT_SYMBOL(max8649_get_dcdc2_level);

static void max8649_set_volts()
{
    Minvolts = MAX8649_MIN_LEVEL;
    Maxvolts = MAX8649_MAX_LEVEL;
}

static void max8952_set_volts()
{
    Minvolts = MAX8952_MIN_LEVEL;
    Maxvolts = MAX8952_MAX_LEVEL;
}

/**
 * max8649_probe - probe the max8649 driver
 * @client: Pointer to the i2c_client structure
 * @dev_id: Pointer to the i2c_device_id structure
 *
 * Returns negative errno, else zero on success.
 */
static int max8649_probe(struct i2c_client *client,
        const struct i2c_device_id *dev_id)
{
    unsigned char *ic_ver;
    
    if(!client){
        printk(KERN_ERR "MAX8649 Invalid i2c client\n");
        return -EINVAL;
    }

    if (!i2c_check_functionality(client->adapter,
                I2C_FUNC_SMBUS_BYTE_DATA)) {
        printk(KERN_ERR "MAX8649 does not support SMBUS_BYTE_DATA.\n");
        return -EINVAL;
    }

    maxclient = client;

    ic_ver = smem_alloc(SMEM_OEM_028,sizeof(char));
    if(ic_ver == NULL){
        printk(KERN_ERR "smem_alloc Err.\n");
        return -EINVAL;
    }
    else if(*ic_ver == IC_MAX8649){
        max8649_set_volts();
    }
    else if(*ic_ver == IC_MAX8952){
        max8952_set_volts();
    }
    else{
        printk(KERN_ERR "smem_alloc Err.\n");
        return -EINVAL;
	}

    return 0;
}

/**
 * max8649_remove - remove the max8649 driver
 * @client: Pointer to the i2c_client structure
 *
 * Returns negative errno, else zero on success.
 */
static int __devexit max8649_remove(struct i2c_client *client)
{
    maxclient = NULL;
    return 0;
}

static const struct i2c_device_id max8649_id[] = {
    { "max8649", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, max8649_id);

static struct i2c_driver max8649_driver = {
    .driver = {
        .name   = "max8649",
        .owner  = THIS_MODULE,
    },
    .probe  = max8649_probe,
    .remove = __devexit_p(max8649_remove),
    .id_table = max8649_id,
};


/**
 * max8649_init - initialize the max8649 driver
 *
 * Returns negative errno, else zero on success.
 */
static int __init max8649_init(void)
{
    return i2c_add_driver(&max8649_driver);
}


/**
 * max8649_exit - delete the max8649 driver
 *
 */
static void __exit max8649_exit(void)
{
    i2c_del_driver(&max8649_driver);
}

module_init(max8649_init);
module_exit(max8649_exit);


MODULE_AUTHOR("TOSHIBA");
MODULE_DESCRIPTION("TG03 max8649");
MODULE_LICENSE("GPL");
