/*
  sub_pmic Driver

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
/*============================================================================
INCLUDE FILES FOR MODULE
============================================================================*/
#include <linux/input.h>
#include <linux/remote_spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include "sub_pmic.h"
#include "subpm_tg03_ext.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "../../../arch/arm/mach-msm/smd_private.h"


/*===========================================================================
    DEFINE
============================================================================*/
#define SMEM_SPINLOCK_I2C   "S:6"

/*===========================================================================
    LOCAL FUNCTIONS PROTOTYPES
============================================================================*/


/*===========================================================================
    EXTERNAL FUNCTIONS PROTOTYPES
============================================================================*/
int sub_pmic_get_sd_detect(bool *sd_status);
int sub_pmic_ldo_ctrl(enum subpm_ldo_no ldo_no, enum subpm_ldo_status state);
int sub_pmic_reg_ctrl(enum subpm_reg_no reg_no, enum subpm_reg_status state);
int sub_pmic_uart_select(enum subpm_uart_kind kind, enum subpm_uart_status in, enum subpm_uart_status out);
int sub_pmic_irda_pwrctrl(enum subpm_irda_pwrctrl state);
int sub_pmic_dtv_reset(enum subpm_dtv_reset state);

int sub_pmic_get_ldo_status(enum subpm_ldo_no ldo_no, enum subpm_ldo_status *status);
int sub_pmic_get_reg_status(enum subpm_reg_no reg_no, enum subpm_reg_status *status);
int sub_pmic_get_uart_status(enum subpm_uart_kind kind, enum subpm_uart_status *in, enum subpm_uart_status *out);
int sub_pmic_get_gpio_status(enum subpm_gpio_no gpio_no, enum subpm_gpio_status *status);

/*===========================================================================
    LOCAL VALUE PROTOTYPES
============================================================================*/
/* Convertion table */
static struct conv_ldo_def {
    enum subpm_ldo_no aarm_ldo_def;
    subpm_ldo_type marm_ldo_def;
} ldo_def[SUB_PMIC_MAX_LDO] = {
    /* aARM def,    mARM def */
    {SUB_PMIC_LDO1, SUBPM_LDO_1},
    {SUB_PMIC_LDO2, SUBPM_LDO_2},
    {SUB_PMIC_LDO3, SUBPM_LDO_3},
    {SUB_PMIC_LDO4, SUBPM_LDO_4},
    {SUB_PMIC_LDO5, SUBPM_LDO_5},
    {SUB_PMIC_LDO6, SUBPM_LDO_6},
    {SUB_PMIC_LDO7, SUBPM_LDO_7},
    {SUB_PMIC_LDO8, SUBPM_LDO_8},
};

/* Convertion table */
static struct conv_reg_def {
    enum subpm_reg_no aarm_reg_def;
    subpm_reg_type marm_reg_def;
} reg_def[SUB_PMIC_MAX_REG] = {
    /* aARM def,    mARM def */
    {SUB_PMIC_REG1, SUBPM_REG_1},
    {SUB_PMIC_REG2, SUBPM_REG_2},
};

/* Convertion table */
static struct conv_uart_def {
    enum subpm_uart_kind aarm_uart_def;
    int marm_uart_def;
} uart_def[SUB_PMIC_MAX_UART] = {
    /* aARM def,    mARM def */
    {SUB_PMIC_LOG, SUBPM_UART_SW_LOG},
    {SUB_PMIC_IRDA, SUBPM_UART_SW_IRDA},
};

/* Convertion table */
static struct conv_gpio_def {
    enum subpm_gpio_no aarm_reg_def;
    subpm_gpio_type marm_reg_def;
} gpio_def[SUB_PMIC_MAX_GPIO] = {
    /* aARM def,    mARM def */
    {SUB_PMIC_GPKI00, SUBPM_GPIO_GPKI0},
    {SUB_PMIC_GPKI01, SUBPM_GPIO_GPKI1},
    {SUB_PMIC_GPKI02, SUBPM_GPIO_GPKI2},
    {SUB_PMIC_GPKI03, SUBPM_GPIO_GPKI3},
    {SUB_PMIC_GPKI04, SUBPM_GPIO_GPKI4},
    {SUB_PMIC_GPKI05, SUBPM_GPIO_GPKI5},
    {SUB_PMIC_GPKI06, SUBPM_GPIO_GPKI6},
    {SUB_PMIC_GPKI07, SUBPM_GPIO_GPKI7},
    {SUB_PMIC_GPKO00, SUBPM_GPIO_GPKO0},
    {SUB_PMIC_GPKO01, SUBPM_GPIO_GPKO1},
    {SUB_PMIC_GPKO02, SUBPM_GPIO_GPKO2},
    {SUB_PMIC_GPKO03, SUBPM_GPIO_GPKO3},
    {SUB_PMIC_GPKO04, SUBPM_GPIO_GPKO4},
    {SUB_PMIC_GPKO05, SUBPM_GPIO_GPKO5},
    {SUB_PMIC_GPKO06, SUBPM_GPIO_GPKO6},
    {SUB_PMIC_GPKO07, SUBPM_GPIO_GPKO7},
    {SUB_PMIC_GPKO08, SUBPM_GPIO_GPKO8},
    {SUB_PMIC_GPKO09, SUBPM_GPIO_GPKO9},
    {SUB_PMIC_GPKO10, SUBPM_GPIO_GPKO10},
    {SUB_PMIC_GPKO11, SUBPM_GPIO_GPKO11},
};

static uint32_t *rmutex_i2c;
static remote_spinlock_t rslock_i2c;

/*===========================================================================
    GLOBAL VALUE PROTOTYPES
============================================================================*/

static void i2c_rspin_lock(uint32_t *rmutex)
{
    int lock = 0;
    unsigned long flags = 0;
    do {
        remote_spin_lock_irqsave(&rslock_i2c, flags);
        if (*rmutex == 0) {
            *rmutex = 1;
            lock = 1;
        }
        remote_spin_unlock_irqrestore(&rslock_i2c, flags);
        schedule();
    } while (!lock);
}

static void i2c_rspin_unlock(uint32_t *rmutex)
{
    unsigned long flags = 0;
    remote_spin_lock_irqsave(&rslock_i2c, flags);
    *rmutex = 0;
    remote_spin_unlock_irqrestore(&rslock_i2c, flags);
}


/**
 * sub_pmic_get_sd_detect - get SD card status
 * @sd_status: TRUE:SD is inserted.  FALSE:SD is not inserted.
 *
 * Returns negative errno, else zero on success.
 */
int sub_pmic_get_sd_detect(bool *sd_status)
{
    int ret = SUB_PMIC_ERROR;
    int rc = 0;
    unsigned rd_data = 0;
    unsigned type = SUBPM_GPIO_SDDET;
    spinlock_t lock;
    unsigned long flag = 0;

    if(sd_status != NULL)
    {
        if(in_interrupt() != 0){
            printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
            return ret;
        }

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_015, &type, &rd_data);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{
            if(rd_data == SUBPM_GPIO_LO){
                *sd_status = TRUE;  /* SD detect */
            }else{
                *sd_status = FALSE; /* SD NOT detect */
            }
            ret = SUB_PMIC_SUCCESS;
        }
    }
    else
    {
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_get_sd_detect);


/**
 * sub_pmic_ldo_ctrl - request PMIC driver to turn on/off the LDO
 * @ldo_no: kind of LDO
 * @state: LDO ON/OFF
 *
 * Returns negative errno, else zero on success.
 */
int sub_pmic_ldo_ctrl(enum subpm_ldo_no ldo_no, enum subpm_ldo_status state)
{

    int ret = SUB_PMIC_ERROR;
    u32 i;
    unsigned subpm_ctrl;
    int rc;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    for(i = 0; i < SUB_PMIC_MAX_LDO; i++){
        if(ldo_def[i].aarm_ldo_def == ldo_no){
            break;
        }
    }

    if(i < SUB_PMIC_MAX_LDO){
        if(state == SUB_PMIC_LDO_ON){
            /* LDOx ON */
            subpm_ctrl = SUBPM_CTRL_ON;
        }else{
            /* LDOx OFF */
            subpm_ctrl = SUBPM_CTRL_OFF;
        }

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_011, (unsigned *)&ldo_def[i].marm_ldo_def, &subpm_ctrl);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{
            ret = SUB_PMIC_SUCCESS;
        }

    }else{
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_ldo_ctrl);


/**
 * sub_pmic_reg_ctrl - request PMIC driver to turn on/off the REG
 * @ldo_no: kind of REG
 * @state: REG ON/OFF
 *
 * Returns negative errno, else zero on success.
 */
int sub_pmic_reg_ctrl(enum subpm_reg_no reg_no, enum subpm_reg_status state)
{

    int ret = SUB_PMIC_ERROR;
    u32 i;
    unsigned subpm_ctrl;
    int rc;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    for(i = 0; i < SUB_PMIC_MAX_REG; i++){
        if(reg_def[i].aarm_reg_def == reg_no){
            break;
        }
    }

    if(i < SUB_PMIC_MAX_REG){
        if(state == SUB_PMIC_REG_ON){
            /* REGx ON */
            subpm_ctrl = SUBPM_CTRL_ON;
        }else{
            /* REGx OFF */
            subpm_ctrl = SUBPM_CTRL_OFF;
        }

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_012, (unsigned *)&reg_def[i].marm_reg_def, &subpm_ctrl);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{
            ret = SUB_PMIC_SUCCESS;
        }
    }else{
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_reg_ctrl);


/**
 * sub_pmic_reg_ctrl - request PMIC driver to switched the path of UART
 * @kind: kind of UART path
 *
 * Returns negative errno, else zero on success.
 */
int sub_pmic_uart_select(enum subpm_uart_kind kind, enum subpm_uart_status in, enum subpm_uart_status out)
{
    int ret = SUB_PMIC_ERROR;
    u32 i;
    int rc;
    subpm_uart_ctrl_s uart_ctrl;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    for(i = 0; i < SUB_PMIC_MAX_UART; i++){
        if(uart_def[i].aarm_uart_def == kind){
            break;
        }
    }

    if(i < SUB_PMIC_MAX_UART){
        if(in == SUB_PMIC_UART_ENABLE){
            uart_ctrl.input = SUBPM_CTRL_ON;
        }else{
            uart_ctrl.input = SUBPM_CTRL_OFF;
        }

        if(out == SUB_PMIC_UART_ENABLE){
            uart_ctrl.output = SUBPM_CTRL_ON;
        }else{
            uart_ctrl.output = SUBPM_CTRL_OFF;
        }

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_013, (unsigned *)&uart_def[i].marm_uart_def, (unsigned *)&uart_ctrl);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{
            ret = SUB_PMIC_SUCCESS;
        }
    }else{
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_uart_select);



/**
 * sub_pmic_irda_pwrctrl - turn on/off a IrDA power
 * @state: ON/OFF
 *
 * Returns negative errno, else zero on success.
 */
int sub_pmic_irda_pwrctrl(enum subpm_irda_pwrctrl state)
{

    int ret = SUB_PMIC_ERROR;
    int rc = 0;
    unsigned type = SUBPM_GPIO_IRDA;
    unsigned tmp_state;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    if(state == SUB_PMIC_IRDA_ON){
        tmp_state = SUBPM_GPIO_LO;
    }else{
        tmp_state = SUBPM_GPIO_HI;
    }


    /* waiting for I2C mutex */
    if(rmutex_i2c != NULL) {
        i2c_rspin_lock(rmutex_i2c);

        spin_lock_irqsave(&lock, flag);

        i2c_rspin_unlock(rmutex_i2c);
    }

    rc = msm_proc_comm(PCOM_OEM_014, &type, &tmp_state);

    if(rmutex_i2c != NULL) {
        spin_unlock_irqrestore(&lock, flag);
    }

    if(rc != 0){
        /* error */
        printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
    }else{
        ret = SUB_PMIC_SUCCESS;
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_irda_pwrctrl);



/**
 * sub_pmic_dtv_reset - open/reset dtv hw
 * @state: OPEN/RESET
 *
 * Returns negative errno, else zero on success.
 */
int sub_pmic_dtv_reset(enum subpm_dtv_reset state)
{

    int ret = SUB_PMIC_ERROR;
    int rc = 0;
    unsigned type = SUBPM_GPIO_DTV_RST;
    unsigned tmp_state;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    if(state == SUB_PMIC_DTV_RESET){
        tmp_state = SUBPM_GPIO_LO;
    }else{
        tmp_state = SUBPM_GPIO_HI;
    }

    /* waiting for I2C mutex */
    if(rmutex_i2c != NULL) {
        i2c_rspin_lock(rmutex_i2c);

        spin_lock_irqsave(&lock, flag);

        i2c_rspin_unlock(rmutex_i2c);
    }

    rc = msm_proc_comm(PCOM_OEM_014, &type, &tmp_state);

    if(rmutex_i2c != NULL) {
        spin_unlock_irqrestore(&lock, flag);
    }

    if(rc != 0){
        /* error */
        printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
    }else{
        ret = SUB_PMIC_SUCCESS;
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_dtv_reset);


int sub_pmic_get_ldo_status(enum subpm_ldo_no ldo_no, enum subpm_ldo_status *status)
{
    int ret = SUB_PMIC_ERROR;
    u32 i;
    unsigned tmp_status;
    int rc;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    for(i = 0; i < SUB_PMIC_MAX_LDO; i++){
        if(ldo_def[i].aarm_ldo_def == ldo_no){
            break;
        }
    }

    if(i < SUB_PMIC_MAX_LDO){

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_016, (unsigned *)&ldo_def[i].marm_ldo_def, &tmp_status);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{

            if(tmp_status == SUB_PMIC_LDO_ON){
                /* LDOx ON */
                *status = SUBPM_CTRL_ON;
            }else{
                /* LDOx OFF */
                *status = SUBPM_CTRL_OFF;
            }
            ret = SUB_PMIC_SUCCESS;
        }

    }else{
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_get_ldo_status);


int sub_pmic_get_reg_status(enum subpm_reg_no reg_no, enum subpm_reg_status *status)
{
    int ret = SUB_PMIC_ERROR;
    u32 i;
    unsigned tmp_status;
    int rc;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    for(i = 0; i < SUB_PMIC_MAX_REG; i++){
        if(reg_def[i].aarm_reg_def == reg_no){
            break;
        }
    }

    if(i < SUB_PMIC_MAX_REG){

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_017, (unsigned *)&reg_def[i].marm_reg_def, &tmp_status);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{

            if(tmp_status == SUB_PMIC_REG_ON){
                /* REGx ON */
                *status = SUBPM_CTRL_ON;
            }else{
                /* REGx OFF */
                *status = SUBPM_CTRL_OFF;
            }
            ret = SUB_PMIC_SUCCESS;
        }

    }else{
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_get_reg_status);


int sub_pmic_get_uart_status(enum subpm_uart_kind kind, enum subpm_uart_status *in, enum subpm_uart_status *out)
{
    int ret = SUB_PMIC_ERROR;
    u32 i;
    int rc;
    subpm_uart_ctrl_s uart_ctrl;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    for(i = 0; i < SUB_PMIC_MAX_UART; i++){
        if(uart_def[i].aarm_uart_def == kind){
            break;
        }
    }

    if(i < SUB_PMIC_MAX_UART){

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_018, (unsigned *)&uart_def[i].marm_uart_def, (unsigned *)&uart_ctrl);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{

            if(uart_ctrl.input == SUB_PMIC_UART_ENABLE){
                *in = SUBPM_CTRL_ON;
            }else{
                *in = SUBPM_CTRL_OFF;
            }

            if(uart_ctrl.output == SUB_PMIC_UART_ENABLE){
                *out = SUBPM_CTRL_ON;
            }else{
                *out = SUBPM_CTRL_OFF;
            }
            ret = SUB_PMIC_SUCCESS;
        }
    }else{
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_get_uart_status);


int sub_pmic_get_gpio_status(enum subpm_gpio_no gpio_no, enum subpm_gpio_status *status)
{
    int ret = SUB_PMIC_ERROR;
    u32 i;
    unsigned tmp_status;
    int rc;
    spinlock_t lock;
    unsigned long flag = 0;

    if(in_interrupt() != 0){
        printk(KERN_ERR "[sub_pmic]%s() called in interrupt context!!\n", __func__);
        return ret;
    }

    for(i = 0; i < SUB_PMIC_MAX_GPIO; i++){
        if(gpio_def[i].aarm_reg_def == gpio_no){
            break;
        }
    }

    if(i < SUB_PMIC_MAX_GPIO){

        /* waiting for I2C mutex */
        if(rmutex_i2c != NULL) {
            i2c_rspin_lock(rmutex_i2c);

            spin_lock_irqsave(&lock, flag);

            i2c_rspin_unlock(rmutex_i2c);
        }

        rc = msm_proc_comm(PCOM_OEM_019, (unsigned *)&gpio_def[i].marm_reg_def, &tmp_status);

        if(rmutex_i2c != NULL) {
            spin_unlock_irqrestore(&lock, flag);
        }

        if(rc != 0){
            /* error */
            printk(KERN_ERR "[sub_pmic]msm_proc_comm() falied! in %s()\n", __func__);
        }else{

            if(tmp_status == SUBPM_GPIO_HI){
                *status = SUB_PMIC_GPIO_HI;
            }else{
                *status = SUB_PMIC_GPIO_LO;
            }
            ret = SUB_PMIC_SUCCESS;
        }
    }else{
        /* error */
        printk(KERN_ERR "[sub_pmic]Invalid arg in %s()\n", __func__);
    }

    return ret;
}
EXPORT_SYMBOL(sub_pmic_get_gpio_status);

static int __init sub_pimic_init(void)
{
    rmutex_i2c = (uint32_t*)smem_alloc(SMEM_I2C_MUTEX, 8);

    if (rmutex_i2c != NULL) {
        if (remote_spin_lock_init(&rslock_i2c, SMEM_SPINLOCK_I2C) != 0)
            rmutex_i2c = NULL;
    }

    return 0;

}
module_init(sub_pimic_init);


MODULE_AUTHOR("TOSHIBA");
MODULE_DESCRIPTION("TG03 sub pmic");
MODULE_LICENSE("GPL");

