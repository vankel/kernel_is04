/*
Capacitive touch panel Driver

Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>

#include "tg03_cap_i2c.h"
#include "tg03_captouch.h"
#include "tsb_model.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"

/*#define _debug_msg_*/
#define TS_DRIVER_NAME          "tg03_captouch"
#define MAX_HW_POINTNUM         10
#define MAX_INIT_WAIT           500
#define MULTI_READ_INT_WAIT     80
#define RESUME_ATCHCALSTHR_CHG_TIME 10

#define TCHINT_STS_NONE         0x00
#define TCHINT_STS_TCHEVT       0x01
#define TCHINT_STS_TCHBLOCK     0x02

#define VENDOR_CHECKSUM_SWC     0x8A
#define VENDOR_CHECKSUM_WTK     0xA0

#define NV_TSB_TOP_ITEMS_I      10000
#define NV_TOUCHPANEL_UD_I      (NV_TSB_TOP_ITEMS_I+108)

/* Device information structure */
typedef struct  {
    struct input_dev *input;
    struct timer_list timer;
    int irq;
    int gpio_int;
    int gpio_reset;
    uint8_t i2c_slave_address;
    int pressure_max;
    int max_point;
    hw_config_data  *default_config;
    struct cdev tg03cap_cdev;
    struct early_suspend tg03cap_esus;
    struct workqueue_struct *tg03cap_wq;
    struct work_struct tg03cap_wq_func;
    struct work_struct tg03cap_wq_resume_func;
    point_data point[MAX_HW_POINTNUM];
    int tap_now;
    int touch_report_id;
    int dev_id;
} tg03_ts_data;

/* Object table register structure */
typedef struct {
    uint8_t type;
    uint8_t start_lsb;
    uint8_t start_msb;
    uint8_t size;
    uint8_t instances;
    uint8_t num_of_report_id;
}element;

/* Information block register structure */
struct tg03cap_information_block{
    uint8_t family_id;
    uint8_t variant_id;
    uint8_t version;
    uint8_t build;
    uint8_t matrix_x_size;
    uint8_t matrix_y_size;
    uint8_t num_of_elements;
    element *elements;
    uint32_t checksum;
};

static struct tg03cap_information_block tg03cap_info;
static int ioctl_irq_sts = 0;
static int cdev_major = 0;
static struct class* udev_class;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tg03cap_early_suspend(struct early_suspend *h);
static void tg03cap_late_resume(struct early_suspend *h);
#endif

static int tg03cap_hw_reset( tg03_ts_data *ts )
{
    if ( ts == NULL){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }
    
    /* Reset HW */
    gpio_set_value( ts->gpio_reset, 0 );    /* Reset ON */
    mdelay(1);                              /* RESET pin must be asserted low for at least 90ns to cause a reset */
    gpio_set_value( ts->gpio_reset, 1 );    /* Reset Off*/
    mdelay(40);                             /* device takes 40ms before it is ready to start communication */

    /* Reset Point status */
    memset( ts->point, 0xFFFFFFFF, sizeof(point_data)*ts->max_point );
    ts->tap_now = 0x00;
    
    return 0;
}

static int tg03cap_i2c_init( void )
{
    bool retval;

    /* initialize i2c */
    retval = cap_i2c_init();
    if ( retval == 0 ){
        printk(KERN_ERR "%s :i2c init error\n", __func__ );
        return -EIO;
    }

    return 0;
}

static int tg03cap_i2c_read( tg03_ts_data *ts, uint8_t *buf, uint16_t addr, uint16_t length )
{
    I2C_INFO  i2c_info_data;
    bool      retval;
    
    if ( (ts == NULL) || (buf == NULL) || (length == 0) ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }
    
    i2c_info_data.slave_addr    = ts->i2c_slave_address;
    i2c_info_data.buf_ptr       = buf;
    i2c_info_data.len           = length;
    i2c_info_data.offset        = addr;
    
    retval = cap_i2c_read(&i2c_info_data);
    if( retval == 0 ) {
        printk(KERN_ERR "%s : i2c read error reg:%d\n", __func__, i2c_info_data.offset );
        tg03cap_hw_reset( ts );
        return -EIO;
    }
    
    return 0;
}

static int tg03cap_i2c_write( tg03_ts_data *ts, uint8_t *buf, uint16_t addr, uint16_t length )
{
    I2C_INFO  i2c_info_data;
    int       cnt;
    bool      retval;

    if ( (ts == NULL) || (buf == NULL) || (length == 0) ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }

    i2c_info_data.slave_addr    = ts->i2c_slave_address;
    i2c_info_data.buf_ptr       = buf;
    i2c_info_data.len           = length;
    i2c_info_data.offset        = addr;

    for( cnt = 0; cnt < 3; cnt++ ) {
        retval = cap_i2c_write(&i2c_info_data);
        if( retval == 0 ) {
            printk(KERN_ERR "%s : i2c write error reg:%d, try:%d\n", __func__, i2c_info_data.offset, cnt+1 );
        } else {
            break;
        }
    }
    
    if( retval == 0 ) {
        printk(KERN_ERR "%s : i2c write error reg:%d\n", __func__, i2c_info_data.offset );
        tg03cap_hw_reset( ts );
        return -EIO;
    }

    return 0;
}

static uint16_t get_object_address ( uint8_t id ) 
{
    int cnt;
    
    if  ( tg03cap_info.num_of_elements == 0 ) {
        return 0;
    }
    
    for ( cnt = 0 ; cnt < tg03cap_info.num_of_elements ; cnt ++ ){
        if ( tg03cap_info.elements[cnt].type == id ) {
            return (uint16_t)( ((tg03cap_info.elements[cnt].start_msb<<8) & 0xFF00) | (tg03cap_info.elements[cnt].start_lsb & 0xFF) );
        }
    }
    
    return 0;
}

static uint8_t get_object_size ( uint8_t id ) 
{
    int cnt;
    
    if  ( tg03cap_info.num_of_elements == 0 ) {
        return 0;
    }
    
    for ( cnt = 0 ; cnt < tg03cap_info.num_of_elements ; cnt ++ ){
        if ( tg03cap_info.elements[cnt].type == id ) {
            return tg03cap_info.elements[cnt].size + 1;
        }
    }

    return 0;
}


static uint8_t report_to_object_id ( uint8_t report_id )
{
    int     cnt;
    uint8_t search_id = 0;
    
    if  ( tg03cap_info.num_of_elements == 0 ) {
        return 0;
    }
    
    for ( cnt = 0 ; cnt < tg03cap_info.num_of_elements ; cnt ++ ){
        search_id += (tg03cap_info.elements[cnt].instances + 1) * (tg03cap_info.elements[cnt].num_of_report_id);
        if ( report_id <= search_id ){
            return tg03cap_info.elements[cnt].type;
        }
    }
    
    return 0;
}

static int check_config_crc( tg03_ts_data *ts, int check_crc )
{
    uint8_t     val ;
    uint32_t    get_crc = 0 ;
    char        read_buff[16];
    int         cnt;

    if ( ts == NULL ) {
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }
    
    if ( ts->irq ){
        disable_irq( ts->irq );
    }
    
    /* Start calibration for Get Message include NV-Memory CRC */
    val = 1;
    tg03cap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+2, 1 );
    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt += 10 ){
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            tg03cap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 5 );
            if ( (report_to_object_id(read_buff[0])==GEN_COMMANDPROCESSOR_T6) && (read_buff[1]&0x10) ){
                break;
            }
        }
        mdelay( 10 );
    }
    get_crc = ((read_buff[4] << 16)&0x00FF0000) | ((read_buff[3] << 8)&0x0000FF00) | read_buff[2];

    if ( ts->irq ){
        enable_irq( ts->irq );
    }

    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "%s : message wait timeout\n", __func__ );
        return -EIO;
    }

    printk(KERN_ERR "%s : HW-CRC = 0x%06x, Default-CRC = 0x%06x\n", __func__, get_crc, check_crc );
    /* Check CRC and config registers */
    if ( get_crc != check_crc ) {
        return 1;
    }
    
    return 0;
}

static int backup_config_registers( tg03_ts_data *ts )
{
    uint8_t val ;
    char    read_buff[16];
    int     retval = 0;
    int     cnt;

    printk(KERN_ERR "%s :[IN]\n", __func__ );
    
    if ( ts == NULL ) {
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }

    if ( ts->irq ){
        disable_irq( ts->irq );
    }

    val = 0x55;
    tg03cap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+1, 1 );
    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt += 10 ){
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            tg03cap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 5 );
            if ( (report_to_object_id(read_buff[0])==GEN_COMMANDPROCESSOR_T6) ){
                break;
            }
        }
        mdelay( 10 );
    }
    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "%s : backup command response timeout\n", __func__ );
    }
    printk( KERN_INFO "%s : send backup command \n", __func__ );

    /* Reset Device */
    val = 0x01;
    tg03cap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6), 1 );
    mdelay( 64 );
    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt += 10 ){
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            tg03cap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 5 );
            if ( (report_to_object_id(read_buff[0])==GEN_COMMANDPROCESSOR_T6) && (read_buff[1]&0x80) ){
                break;
            }
        }
        mdelay( 10 );
    }
    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "%s : reset command response timeout\n", __func__ );
        retval = -EIO;
    }else{
        retval = ((read_buff[4] << 16)&0x00FF0000) | ((read_buff[3] << 8)&0x0000FF00) | read_buff[2];
    }

    if ( ts->irq ){
        enable_irq( ts->irq );
    }
    return retval;

}

static int set_config_registers( tg03_ts_data *ts, hw_config_data *config_data )
{
    int retval ;
    
    printk(KERN_ERR "%s :[IN]\n", __func__ );
    
    if ( (ts==NULL) || (config_data == NULL) ) {
        printk(KERN_ERR "%s :parameter is NULL \n", __func__ );
        return -EINVAL;
    }

    /* write registers */
    retval = tg03cap_i2c_write( ts, config_data->config_t7, get_object_address(GEN_POWERCONFIG_T7),
                       get_object_size(GEN_POWERCONFIG_T7) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t8, get_object_address(GEN_ACQUIRECONFIG_T8),
                       get_object_size(GEN_ACQUIRECONFIG_T8) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t9, get_object_address(TOUCH_MULTITOUCHSCREEN_T9),
                       get_object_size(TOUCH_MULTITOUCHSCREEN_T9) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t15, get_object_address(TOUCH_KEYARRAY_T15),
                       get_object_size(TOUCH_KEYARRAY_T15) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t18, get_object_address(SPT_COMCONFIG_T18),
                       get_object_size(SPT_COMCONFIG_T18) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t19, get_object_address(SPT_GPIOPWM_T19),
                       get_object_size(SPT_GPIOPWM_T19) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t20, get_object_address(PROCI_GRIPFACESUPPRESSION_T20),
                       get_object_size(PROCI_GRIPFACESUPPRESSION_T20) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t22, get_object_address(PROCG_NOISESUPPRESSION_T22),
                       get_object_size(PROCG_NOISESUPPRESSION_T22) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t23, get_object_address(TOUCH_PROXIMITY_T23),
                       get_object_size(TOUCH_PROXIMITY_T23) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t24, get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24),
                       get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t25, get_object_address(SPT_SELFTEST_T25),
                       get_object_size(SPT_SELFTEST_T25) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t27, get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27),
                       get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27) );
    if ( retval != 0)   goto err_out;

    retval = tg03cap_i2c_write( ts, config_data->config_t28, get_object_address(SPT_CTECONFIG_T28),
                       get_object_size(SPT_CTECONFIG_T28) );
    if ( retval != 0)   goto err_out;

#ifdef _debug_msg_
    {
        char read_buff[256];
        int i, j;
        int id;
        
        for ( i = 0 ; i <  tg03cap_info.num_of_elements ; i ++ ) {
            id = tg03cap_info.elements[i].type;
            tg03cap_i2c_read( ts, read_buff, get_object_address( tg03cap_info.elements[i].type ), get_object_size( tg03cap_info.elements[i].type ) );
            printk(KERN_INFO "ObjectNo %02d Register values \n", tg03cap_info.elements[i].type );
            for ( j = 0 ; j < get_object_size(tg03cap_info.elements[i].type)  ; j += 10 ) {
                printk(KERN_ERR "  %03d :  %03d, %03d, %03d, %03d, %03d - %03d, %03d, %03d, %03d, %03d \n",
                    j, 
                    read_buff[j],   read_buff[j+1], read_buff[j+2], read_buff[j+3], read_buff[j+4],
                    read_buff[j+5], read_buff[j+6], read_buff[j+7], read_buff[j+8], read_buff[j+9]
                );
            }
        }
    }
#endif

    return 0;

err_out:
    printk(KERN_ERR "%s : i2c_write_err ret = %d \n", __func__, retval );
    return retval;
    
}

static int reset_configration( tg03_ts_data *ts )
{
    int     retval;
    
    printk(KERN_ERR "%s :[IN]\n", __func__ );

    /* Set config registers ( Excluding the UserData ) */
    retval = set_config_registers( ts, ts->default_config );
    if ( retval != 0 ){
        printk(KERN_ERR "%s : Write Config Error ret = %d \n", __func__, retval );
        return retval;
    }

    /* backup config registers */
    retval = backup_config_registers( ts );
    if (retval != ts->default_config->config_crc ){
        printk(KERN_ERR "%s : backup new crc is not mach def=0x%06x, ret=0x%06x\n", __func__, ts->default_config->config_crc, retval );
        return -EIO;
    }
    printk(KERN_ERR "%s : reset Configuration is success. New CRC is 0x%06x. \n", __func__, retval );

    return retval;

}

static int tg03cap_get_device_id( tg03_ts_data *ts, uint8_t *read_buff )
{
    int     retval;
    int     model_no;
    int     check_sum;
    unsigned nv_data1, nv_data2;
    
    
    if ( (ts==NULL) || (read_buff==NULL) ) {
        printk(KERN_ERR "%s :parameter is NULL \n", __func__ );
        return -EINVAL;
    }

    model_no = tsb_model_get_model_no();

    if ( model_no <= TSB_MODEL_NO_3 ){
        retval = tg03cap_i2c_read( ts, read_buff, get_object_address(SPT_USERDATA_T38), get_object_size(SPT_USERDATA_T38) );
        if ( retval != 0 ){
            printk(KERN_ERR "%s : user data read error retval = %d\n", __func__, retval );
            return -EIO;
        }
    }else{
        nv_data1 = NV_TOUCHPANEL_UD_I;
        retval = msm_proc_comm( PCOM_NV_READ, &nv_data1, &nv_data2 );
        if( retval != 0 ){
            printk( KERN_ERR "%s : read NV error retval = %d \n", __func__, retval );
        }
        printk(KERN_INFO "%s : nv_data1 = 0x%08x, nv_data2 = 0x%08x\n", __func__, nv_data1, nv_data2 );

        read_buff[0] = (nv_data2 & 0x000000FF);
        read_buff[1] = (nv_data2 & 0x0000FF00) >> 8;
        read_buff[2] = (nv_data2 & 0x00FF0000) >> 16;
        read_buff[3] = (nv_data2 & 0xFF000000) >> 24;
        read_buff[4] = (nv_data1 & 0x000000FF);
        read_buff[5] = (nv_data1 & 0x0000FF00) >> 8;
        read_buff[6] = (nv_data1 & 0x00FF0000) >> 16;
        read_buff[7] = (nv_data1 & 0xFF000000) >> 24;

    }

    switch ( model_no ){
        case TSB_MODEL_NO_0:
        case TSB_MODEL_NO_1:
        case TSB_MODEL_NO_11:
            if ( read_buff[0] == 0x01 ){
                retval = MODEL_NO_PR_WTK;
            } else {
                retval = MODEL_NO_1_SWC;
            }
            break;
        case TSB_MODEL_NO_2:
        case TSB_MODEL_NO_3:
            retval = MODEL_NO_PR_WTK;
            break;
        default:    /* TSB_MODEL_NO_PR and later */
            check_sum = read_buff[2] + read_buff[3];
            if ( check_sum == VENDOR_CHECKSUM_SWC ) {
                retval = MODEL_NO_PR_SWC;
            } else if ( check_sum == VENDOR_CHECKSUM_WTK ) {
                retval = MODEL_NO_PR_WTK;
            } else {
                printk( KERN_ERR "%s : touchPanel UserData Error. Data1 = 0x%08x, Data2 = 0x%08x \n", __func__, nv_data1, nv_data2 );
                retval = MODEL_NO_PR_NONE;
            }
            break;
    }

    return retval;
}

static int tg03cap_hardware_init( tg03_ts_data *ts, tg03_captouch_platfrom_data *pdata )
{
    uint8_t read_buff[512]  = {0};
    int     cnt             = 0;
    int     report_id       = 1;
    int     retval;

    if ( (ts == NULL) || (pdata == NULL) ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }

    /*----------------------------- Wait power on ----------------------------*/
    pdata->vreg_config(1);
    for ( cnt = 0 ; cnt <= MAX_INIT_WAIT ; cnt += 10 ){
        /* wait for enable interrupt pin */
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            /* do not read a Message register                           */
            /* Because it does not understand the offset at this timing.*/
            break;
        }
        mdelay( 10 );
    }
    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "[captouch] Warning : INT is not enable, power on after %d msec\n", MAX_INIT_WAIT );
    }


    /*----------------------- area regstry init --------------------------*/
    retval = tg03cap_i2c_read( ts, (uint8_t *)&tg03cap_info, 0, 7 );
    if ( retval != 0 ){
        printk(KERN_ERR "%s : Information block read error retval = %d\n", __func__, retval );
        return -EIO;
    }
    tg03cap_info.elements = kmalloc( tg03cap_info.num_of_elements*sizeof(element), GFP_KERNEL );
    if ( tg03cap_info.elements == NULL ) {
        printk(KERN_ERR "%s : element structure allocation error \n", __func__ );
        return -ENOMEM;
    }

#ifdef _debug_msg_
    printk(KERN_INFO "family_id       %d\n", tg03cap_info.family_id );
    printk(KERN_INFO "variant_id      %d\n", tg03cap_info.variant_id );
    printk(KERN_INFO "version         %d\n", tg03cap_info.version );
    printk(KERN_INFO "build           %d\n", tg03cap_info.build );
    printk(KERN_INFO "matrix_x_size   %d\n", tg03cap_info.matrix_x_size );
    printk(KERN_INFO "matrix_y_size   %d\n", tg03cap_info.matrix_y_size );
    printk(KERN_INFO "num_of_elements %d\n\n", tg03cap_info.num_of_elements );
#endif

    retval = tg03cap_i2c_read( ts, (uint8_t *)tg03cap_info.elements, 7, tg03cap_info.num_of_elements*sizeof(element) );
    if ( retval != 0 ){
        printk(KERN_ERR "%s : element block read error retval = %d\n", __func__, retval );
        return -EIO;
    }

#ifdef _debug_msg_
    {
        int i = 0;
        for ( i=0; i<tg03cap_info.num_of_elements; i++ ) {
            printk(KERN_INFO "type             %d\n", tg03cap_info.elements[i].type);
            printk(KERN_INFO "start_lsb        %d\n", tg03cap_info.elements[i].start_lsb);
            printk(KERN_INFO "start_msb        %d\n", tg03cap_info.elements[i].start_msb);
            printk(KERN_INFO "  -->StartOffsets 0x%x(%d)\n",
                ( ((tg03cap_info.elements[i].start_msb<<8) & 0xFF00) | (tg03cap_info.elements[i].start_lsb & 0xFF) ),
                ( ((tg03cap_info.elements[i].start_msb<<8) & 0xFF00) | (tg03cap_info.elements[i].start_lsb & 0xFF) )
            );
            printk(KERN_INFO "size-1           %d\n", tg03cap_info.elements[i].size);
            printk(KERN_INFO "instances-1      %d\n", tg03cap_info.elements[i].instances);
            printk(KERN_INFO "num_of_report_id %d\n\n", tg03cap_info.elements[i].num_of_report_id);
        }
    }
#endif

    for( cnt=0; cnt<tg03cap_info.num_of_elements; cnt++ ) {
        if( tg03cap_info.elements[cnt].type == TOUCH_MULTITOUCHSCREEN_T9 ) {
            printk(KERN_ERR "touch_report_id = %d\n", report_id);
            ts->touch_report_id = report_id;
            break;
        }
        report_id += (tg03cap_info.elements[cnt].instances + 1) * (tg03cap_info.elements[cnt].num_of_report_id);
    }
    
    retval = tg03cap_get_device_id( ts, read_buff );
    if ( retval < 0 ){
        printk(KERN_ERR "%s : get DeviceID error retval = %d\n", __func__, retval );
        return -EIO;
    }

    printk(KERN_INFO "CapHWCheck : DeviceID = %d \n", retval );
    ts->dev_id = retval;

    ts->default_config = &pdata[ts->dev_id].hw_config ;

    if ( read_buff[1] != 0x01 ){
        if ( check_config_crc( ts, ts->default_config->config_crc ) != 0 ){
            printk(KERN_ERR "%s : CRC is not match. reset Configrations \n", __func__ );
            reset_configration( ts );
        }
    }

    return 0;
}

static irqreturn_t tg03cap_interrupt( int irq, void *dev_id )
{
    tg03_ts_data   *ts = dev_id;

    disable_irq( ts->irq );
    queue_work( ts->tg03cap_wq, &ts->tg03cap_wq_func );

    return IRQ_HANDLED;

}

static void interrupt_work_func( struct work_struct *work )
{

    uint32_t    x, y;
    uint8_t     read_buff[64];
    int         tap_pos;
    int         tch_flg = 0;
    int         cnt;
    int         retval;
    tg03_ts_data *ts = container_of( work, tg03_ts_data, tg03cap_wq_func );

    /* Message Check */
    for ( cnt = 0 ; cnt < ts->max_point ; cnt ++ ){

        /* Check INT */
        if ( gpio_get_value( ts->gpio_int ) != 0x00 ){
            break;
        }

        /* read Message register */
        retval = tg03cap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 8 );
        if ( retval != 0 ){
            printk( KERN_ERR "%s : I2C read error. retval = %d\n", __func__, retval );
            tch_flg = TCHINT_STS_NONE;
            input_report_abs( ts->input, ABS_MT_TOUCH_MAJOR, 0 );
            input_sync( ts->input );
            break;
        }

        switch ( report_to_object_id(read_buff[0]) ){

            case TOUCH_MULTITOUCHSCREEN_T9:
                /* check validate point data */
                tap_pos = read_buff[0] - ts->touch_report_id;
                if ( tap_pos >= ts->max_point ){
                    printk(KERN_INFO "invalid TapID ID=%d, x=%d, y=%d, area=%d\n", tap_pos, ts->point[tap_pos].x, ts->point[tap_pos].y, ts->point[tap_pos].area );
                    break;
                }

                /* set new point data */
                if( read_buff[1] & 0x20 ) {
                    ts->point[tap_pos].x = 0xFFFFFFFF;
                    ts->point[tap_pos].y = 0xFFFFFFFF;
                    ts->point[tap_pos].area = 0xFFFFFFFF;
                    ts->point[tap_pos].delta = 0xFFFFFFFF;
                    ts->tap_now &= ~((int)(0x0001 << tap_pos));
#ifdef _debug_msg_
                    printk( KERN_ERR "%s : Pen Up!(%d)\n", __func__, tap_pos );
#endif
                }else if( read_buff[1] & 0x80 ) {
                    x = (uint32_t)( ((read_buff[2] << 8) | (read_buff[4] & 0xC0)) >> 6 );
                    y = (uint32_t)( ((read_buff[3] << 4) | (read_buff[4] & 0x0C)) >> 2 );

                    if( tsb_model_get_model_no() == TSB_MODEL_NO_0 ) {
                        ts->point[tap_pos].x = 1023-y;
                    } else {
                        ts->point[tap_pos].x = y;
                    }
                    ts->point[tap_pos].y = x;
                    ts->point[tap_pos].area = read_buff[5];
                    ts->point[tap_pos].delta = read_buff[6];

                    if ( read_buff[1] & 0x40 ){
#ifdef _debug_msg_
                        printk( KERN_ERR "%s : Pen Pressed!(%d)\n", __func__, tap_pos );
#endif
                        if ( ts->tap_now & (int)(0x0001 << tap_pos) ){
                            printk( KERN_ERR "************* %s : Pen Up Error tap_now = 0x%x, tap_pos = %d****************\n", __func__, ts->tap_now, tap_pos );
                            tch_flg = TCHINT_STS_NONE;
                        }
                    }
                    ts->tap_now |= (int)(0x0001 << tap_pos);
                }

                tch_flg |= TCHINT_STS_TCHEVT;
                break;

            case PROCI_GRIPFACESUPPRESSION_T20:
#ifdef _debug_msg_
                printk(KERN_INFO "%s : I got PROCI_GRIPFACESUPPRESSION_T20 Messages. 0x%02x\n", __func__, read_buff[1] );
#endif
               if ( read_buff[1] & 0x01 ){
                    tch_flg |= TCHINT_STS_TCHBLOCK;
                }
                break;

            case PROCG_NOISESUPPRESSION_T22:
#ifdef _debug_msg_
                printk(KERN_INFO "%s : I got PROCG_NOISESUPPRESSION_T22 Messages. 0x%02x, 0x%02x, 0x%02x\n", __func__, read_buff[1], read_buff[2], read_buff[3] );
#endif
                break;
            
            default:
#ifdef _debug_msg_
                printk(KERN_INFO "Type = %d, ReadData= 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n",
                    report_to_object_id(read_buff[0]),
                    read_buff[0], read_buff[1], read_buff[2], read_buff[3],
                    read_buff[4], read_buff[5], read_buff[6], read_buff[7], read_buff[8]
                );
#endif
                break;
        }
        udelay( MULTI_READ_INT_WAIT );
    }

    if ( tch_flg & TCHINT_STS_TCHBLOCK ){
#ifdef _debug_msg_
        printk(KERN_INFO "%s : TCHINT_STS_TCHBLOCK bit is ON. force PenUp. 0x%x\n", __func__, tch_flg );
#endif
        memset ( ts->point, 0xFF, sizeof(point_data)*ts->max_point );
        ts->tap_now = 0;
    }

    if ( tch_flg != TCHINT_STS_NONE ){
        if ( ts->tap_now == 0x00 ) {
            input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
            input_sync(ts->input);
        }else{
            for ( cnt = 0 ; cnt < ts->max_point ; cnt ++ ) {
#ifdef _debug_msg_
                printk(KERN_INFO "pen down ! ID=%d, x=%d, y=%d, area=%d\n", cnt, ts->point[cnt].x, ts->point[cnt].y, ts->point[cnt].area );
#endif
                if ( ts->tap_now & (int)(0x0001<<cnt) ) {
                    input_report_abs(ts->input, ABS_MT_TRACKING_ID, cnt);
                    input_report_abs(ts->input, ABS_MT_POSITION_X, ts->point[cnt].x);
                    input_report_abs(ts->input, ABS_MT_POSITION_Y, ts->point[cnt].y);
                    input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, ts->pressure_max);
                    input_mt_sync(ts->input);
                }
            }
        }
        input_sync(ts->input);
    }

    enable_irq(ts->irq);

    return;
}


static void resume_timeout_func( unsigned long ptr )
{
    tg03_ts_data *ts = (tg03_ts_data *)ptr;

    queue_work( ts->tg03cap_wq, &ts->tg03cap_wq_resume_func );

}

static void resume_work_func( struct work_struct *work )
{
    tg03_ts_data *ts = container_of( work, tg03_ts_data, tg03cap_wq_resume_func );
    char    val = 0x00;

    disable_irq( ts->irq );

    val = ts->default_config->config_t8[7];
    tg03cap_i2c_write( ts, &val, get_object_address(GEN_ACQUIRECONFIG_T8)+7, 1 );

    enable_irq(ts->irq);

}

static int tg03cap_cdev_open( struct inode *inode, struct file *file )
{
    tg03_ts_data *ts = container_of( inode->i_cdev, tg03_ts_data, tg03cap_cdev );
    
    printk( KERN_INFO "%s :[IN]\n", __func__ );
    file->private_data = ts;
    return 0;
}

static int tg03cap_cdev_release(struct inode *inode, struct file *file)
{
    printk( KERN_INFO "%s :[IN]\n", __func__ );

    return 0;
}

static int tg03cap_cdev_ioctl( struct inode    *inode,
                               struct file     *file,
                               unsigned int    cmd,
                               unsigned long   arg    )
{
    int             retval = 0;
    tg03_ts_data    *ts = (tg03_ts_data *)file->private_data;

    printk( KERN_INFO "%s(cmd=0x%08x) :[IN]\n", __func__, cmd );
    
    switch(cmd){
        case IOCTL_SET_DBGMODE:
            if ( ioctl_irq_sts == 0 ){
                free_irq(ts->irq, ts);
                ioctl_irq_sts = 1; 
            }else{
                retval = request_irq( ts->irq, tg03cap_interrupt, IRQF_TRIGGER_FALLING, "touchscreen", ts );
                ioctl_irq_sts = 0; 
            }
            break;

        default:
            printk( KERN_INFO "%s DBG : show All IOCTL Command code\n", __func__ );
            printk( KERN_INFO "  IOCTL_SET_DBGMODE = 0x%x\n", IOCTL_SET_DBGMODE );
            break;
    }
    printk( KERN_INFO "%s(cmd=0x%08x) :[OUT] retval = %d\n", __func__, cmd, retval );
    return retval;
}

static struct file_operations tg03cap_fops = {
    .owner   = THIS_MODULE,
	.release = tg03cap_cdev_release,
    .open    = tg03cap_cdev_open,
    .ioctl   = tg03cap_cdev_ioctl,
};

static int __devinit tg03cap_pdev_probe(struct platform_device *pdev)
{
    int result;
    struct input_dev *input_dev = 0;
    tg03_ts_data *ts;
    tg03_captouch_platfrom_data *pdata = pdev->dev.platform_data;
    int devno;
    dev_t dev = MKDEV(cdev_major, 0);

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    ts = kzalloc(sizeof(tg03_ts_data), GFP_KERNEL);
    if ( ts == NULL ) {
        printk( KERN_ERR"%s: alloc ts failed\n", __func__ );
        result = -ENOMEM;
        goto fail_alloc_ts;
    }

    /* Make charactor device    */
    udev_class = class_create(THIS_MODULE, "tg03_cap");
    if(IS_ERR(udev_class)) {
        result = PTR_ERR(udev_class);
        printk( KERN_ERR"%s: class_create failed result = %d\n", __func__, result );
        goto fail_class_create;
    }
  
    /* Figure out our device number. */
    result = alloc_chrdev_region(&dev, 0, 1, "tg03_cap");
    cdev_major = MAJOR(dev);
    if (result < 0) {
        printk( KERN_ERR"%s: alloc_chrdev_region failed result = %d\n", __func__, result );
        goto fail_cdev_region;
    }
    if (cdev_major == 0)
        cdev_major = result;
  
    /* Now set up two cdevs. */
    devno = MKDEV( cdev_major, 0 ); 
    cdev_init( &(ts->tg03cap_cdev), &tg03cap_fops );
    ts->tg03cap_cdev.owner = THIS_MODULE;
    ts->tg03cap_cdev.ops = &tg03cap_fops;
    result = cdev_add ( &(ts->tg03cap_cdev), devno, 1 );
    if(result){
        printk( KERN_ERR"%s: cdev_add failed result = %d\n", __func__, result );
        goto fail_cdev_add;
    }
    
    if ( IS_ERR(device_create(udev_class, NULL, devno, NULL, "tg03_cap")) ){
        printk(KERN_ERR "can't create device\n");
        goto fail_dev_create;
    }

    input_dev = input_allocate_device();
    if ( input_dev == NULL ) {
        result = -ENOMEM;
        printk( KERN_ERR"%s: input_allocate_device failed result = %d\n", __func__, result );
        goto fail_alloc_input;
    }
    input_dev->name = TS_DRIVER_NAME;
    input_dev->phys = "tg03cap_info/input0";
    input_dev->dev.parent = &pdev->dev;

    ts->max_point = pdata->max_point;
    ts->gpio_int = pdata->gpio_int;
    ts->gpio_reset = pdata->gpio_reset;
    ts->pressure_max = pdata->pressure_max;

    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

    /* finger position */
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, pdata->max_point-1, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, pdata->pressure_max, 0, 0);        /* Pressure */
#if 0
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 209, 0, 0);
#endif

    result = input_register_device(input_dev);
    if ( result ){
        printk( KERN_ERR"%s: input_register_device failed result = %d\n", __func__, result );
        goto fail_input_register;
    }
    
    ts->input = input_dev;
    ts->i2c_slave_address = pdata->touch_addr;

    result = tg03cap_i2c_init();
    if ( result ){
        printk( KERN_ERR"%s: i2c init failed result = %d\n", __func__, result );
        goto fail_i2c_init;
    }
    result = tg03cap_hardware_init( ts, pdata );
    if ( result ){
        printk( KERN_ERR"%s: hardware init failed result = %d\n", __func__, result );
        goto fail_hw_init;
    }

    /* workqueue init   */
    ts->tg03cap_wq = create_singlethread_workqueue( "tg03cap_wq" );
    if ( ts->tg03cap_wq == NULL ) {
        printk( KERN_ERR"%s: create workqueue failed\n", __func__ );
        result = -ENOMEM;
        goto fail_create_wq;
    }
    INIT_WORK( &ts->tg03cap_wq_func, interrupt_work_func );
    INIT_WORK( &ts->tg03cap_wq_resume_func, resume_work_func );

    /* irq init */
    ts->irq = MSM_GPIO_TO_INT( ts->gpio_int );
    result = request_irq(ts->irq, tg03cap_interrupt, IRQF_TRIGGER_FALLING, "touchscreen", ts);
    if (result){
        printk( KERN_ERR"%s: request_irq failed result = %d\n", __func__, result );
        goto fail_req_irq;
    }
    
    platform_set_drvdata( pdev, ts );

    setup_timer( &ts->timer, resume_timeout_func, (unsigned long)ts );

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->tg03cap_esus.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
    ts->tg03cap_esus.suspend = tg03cap_early_suspend;
    ts->tg03cap_esus.resume = tg03cap_late_resume;
    register_early_suspend( &ts->tg03cap_esus );
#endif

    return 0;

fail_req_irq:
    destroy_workqueue( ts->tg03cap_wq );
fail_create_wq:
fail_i2c_init:
fail_hw_init:
    input_unregister_device( input_dev );
    input_dev = NULL;
fail_input_register:
    input_free_device( input_dev );
fail_alloc_input:
    device_destroy( udev_class, MKDEV(cdev_major, 0) );
fail_dev_create:
    cdev_del( &(ts->tg03cap_cdev) );
fail_cdev_add:
fail_cdev_region:
    class_destroy( udev_class );
fail_class_create:
    kfree(ts);
fail_alloc_ts:
    printk(KERN_INFO "***** TOUCH_DEBUG : ERROR EXIT %s\n", __func__);
    return result;

}


static int __devexit tg03cap_pdev_remove(struct platform_device *pdev)
{
    tg03_ts_data *ts = platform_get_drvdata( pdev );
    tg03_captouch_platfrom_data *pdata = pdev->dev.platform_data;
    dev_t dev = MKDEV(cdev_major, 0);

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    free_irq(ts->irq, ts);
    pdata->vreg_config(0);

    destroy_workqueue( ts->tg03cap_wq );
    input_unregister_device( ts->input );
    input_free_device( ts->input );

    device_destroy( udev_class, MKDEV(cdev_major, 0) );
    class_destroy( udev_class );
    cdev_del( &(ts->tg03cap_cdev) );
    unregister_chrdev_region( dev, 1);
    
    platform_set_drvdata(pdev, NULL);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend( &ts->tg03cap_esus );
#endif
    kfree(ts);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void tg03cap_early_suspend( struct early_suspend *esus )
{

    tg03_ts_data *ts;
    char    val[2] = { 0x00, 0x00 };
    char    com_val = 0x02;
    int     ret;

    if ( esus == NULL ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return;
    }

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    ts = container_of( esus, tg03_ts_data, tg03cap_esus );

    tg03cap_i2c_write( ts, &com_val, get_object_address(SPT_COMCONFIG_T18)+1, 1 );
    mdelay(1);

    disable_irq( ts->irq );
    ret = cancel_work_sync( &ts->tg03cap_wq_func );
    if ( ret == 1 ){
        enable_irq( ts->irq );
    }

    del_timer_sync( &ts->timer );

    tg03cap_i2c_write( ts, val, get_object_address(GEN_POWERCONFIG_T7), 2 );

}

static void tg03cap_late_resume( struct early_suspend *esus )
{
    tg03_ts_data *ts;
    char val = 0x00;
    
    if ( esus == NULL ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return;
    }

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    ts = container_of( esus, tg03_ts_data, tg03cap_esus );

    tg03cap_hw_reset( ts );

    val = ts->default_config->resume_atchcalsthr;
    tg03cap_i2c_write( ts, &val, get_object_address(GEN_ACQUIRECONFIG_T8)+7, 1 );

    ts->timer.expires = jiffies + RESUME_ATCHCALSTHR_CHG_TIME * HZ;
    add_timer( &ts->timer );

    enable_irq( ts->irq );

}

#else   /* CONFIG_HAS_EARLYSUSPEND */

static int tg03cap_pdev_suspend(struct platform_device *pdev, pm_message_t state)
{
    tg03_ts_data *ts = platform_get_drvdata( pdev );
    char    val[2] = { 0x00, 0x00 };
    char    com_val = 0x02;
    int     ret;
    
    printk( KERN_INFO "%s :[IN]\n", __func__ );

    tg03cap_i2c_write( ts, &com_val, get_object_address(SPT_COMCONFIG_T18)+1, 1 );
    mdelay(1);

    disable_irq( ts->irq );
    ret = cancel_work_sync( &ts->tg03cap_wq_func );
    if ( ret == 1 ){
        enable_irq( ts->irq );
    }

    del_timer_sync( &ts->timer );

    tg03cap_i2c_write( ts, val, get_object_address(GEN_POWERCONFIG_T7), 2 );

    return 0;
}

static int tg03cap_pdev_resume(struct platform_device *pdev)
{
    tg03_ts_data *ts = platform_get_drvdata( pdev );
    char val = 0x00;

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    tg03cap_hw_reset( ts );

    val = ts->default_config->resume_atchcalsthr;
    tg03cap_i2c_write( ts, &val, get_object_address(GEN_ACQUIRECONFIG_T8)+7, 1 );

    ts->timer.expires = jiffies + RESUME_ATCHCALSTHR_CHG_TIME * HZ;
    add_timer( &ts->timer );

    enable_irq( ts->irq );

    return 0;
}

#endif

static struct platform_driver tg03cap_pdev = {
    .probe      = tg03cap_pdev_probe,
    .remove     = __devexit_p(tg03cap_pdev_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = tg03cap_pdev_suspend,
    .resume     = tg03cap_pdev_resume,
#endif
    .driver     = {
        .name = TS_DRIVER_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init tg03cap_init(void)
{
    int i = platform_driver_register( &tg03cap_pdev );
    return i;
}
module_init(tg03cap_init);

static void __exit tg03cap_exit(void)
{
    platform_driver_unregister( &tg03cap_pdev );
}
module_exit(tg03cap_exit);

MODULE_DESCRIPTION("TG03 Touch Screen driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tg03_captouch");

