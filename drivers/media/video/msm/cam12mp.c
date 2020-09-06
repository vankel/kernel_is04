/*
  cam12mp Driver

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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include "sub_pmic.h"
#include "camsensor_gpio.h"
#include "dev_model.h"
#include "../../../arch/arm/mach-msm/smd_private.h"

#if 0
#define LOGI(fmt, args...)      printk(KERN_DEBUG "cam12mp: " fmt, ##args)
#else
#define LOGI(fmt, args...)      do{}while(0)
#endif
#define LOGE(fmt, args...)      printk(KERN_ERR "cam12mp: " fmt, ##args)

// GPIO
#define RESET                   0
#define H_STBY                  1
#define MCLK                    15
#define R_STBY                  151
#define CAM_18V                 29

/*===================================================================*
    LOCAL DECLARATIONS
 *===================================================================*/
struct cam12mp_ctrl {
    const struct msm_camera_sensor_info *sensordata;
    int model;
    int led;
};

DEFINE_MUTEX(cam12mp_mtx);
static struct cam12mp_ctrl *cam12mp_ctrl = NULL;
static struct spi_device *cam12mp_spi_dev = NULL;
static uint8_t *pDlMem = NULL;

/*===================================================================*
    EXTERNAL DECLARATIONS
 *===================================================================*/
extern int _I2C_LOG_;

//////////////////////////////////
// LED Control
//////////////////////////////////
static void cam12mp_led_control(int ctrl)
{
    switch (ctrl) {
    case LED_OFF:
        if(cam12mp_ctrl->model == DEV_MODEL_NO_0 || cam12mp_ctrl->model == DEV_MODEL_NO_1) {
            pmic_set_led_intensity(LED_LCD, 0);
            pmic_set_led_intensity(LED_KEYPAD, 0);
        } else {
            pmic_secure_mpp_config_i_sink(PM_MPP_13,cam12mp_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_DIS);
            pmic_secure_mpp_config_i_sink(PM_MPP_18,cam12mp_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_DIS);
        }
        if ( vreg_disable(vreg_get(NULL, "boost"))) // VREG_5V
            LOGE("%s: vreg_5Vdisable failed !\n", __func__);
        break;
    case LED_LOW:
    case LED_HIGH:
        if ( vreg_enable(vreg_get(NULL, "boost")))  // VREG_5V
            LOGE("%s: vreg_5V enable failed !\n", __func__);
        mdelay(1);

        if(cam12mp_ctrl->model == DEV_MODEL_NO_0 || cam12mp_ctrl->model == DEV_MODEL_NO_1) {
            pmic_set_led_intensity(LED_LCD, 1);
            pmic_set_led_intensity(LED_KEYPAD, 1);
        } else {
            cam12mp_ctrl->led = ctrl == LED_LOW ? PM_MPP__I_SINK__LEVEL_10mA :
                                                  PM_MPP__I_SINK__LEVEL_15mA;
            pmic_secure_mpp_config_i_sink(PM_MPP_13, cam12mp_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_ENA);
            pmic_secure_mpp_config_i_sink(PM_MPP_18, cam12mp_ctrl->led,
                                          PM_MPP__I_SINK__SWITCH_ENA);
        }
        break;
    }
}

////////////////////////////////
// SPI SYNC
////////////////////////////////
static int cam12mp_spi(uint8_t *ptx, uint8_t *prx, uint32_t len)
{
    struct spi_message  msg;
    struct spi_transfer xfer;
    int rc = 0;

    spi_message_init(&msg);
    memset((void*)&xfer, 0, sizeof(xfer));
    xfer.tx_buf = ptx;
    xfer.len = len;
    xfer.rx_buf = prx;
    xfer.bits_per_word = 16;
    xfer.speed_hz = 26330000;
    spi_message_add_tail(&xfer, &msg);
    rc = spi_sync(cam12mp_spi_dev, &msg);
    if (rc < 0) LOGE(" - spi_sync(): Send Error (%d)\n", rc);
    return rc;
}

////////////////////////////////
// DL
////////////////////////////////
static int cam12mp_dl(struct cfg_dl *dl)
{
    int i;
    uint8_t rx[2];
    uint8_t *p[4];

    if(!pDlMem) {
        LOGE(" -%s kzalloc() Failed!\n",__func__);
        return -ENOMEM;
    }

    p[0] = pDlMem;
    p[1] = p[0] + dl->len[0];
    p[2] = p[1] + dl->len[1];
    p[3] = p[2] + dl->len[2];

    for(i=0;i<4;++i) {
        if (copy_from_user(p[i], dl->dt[i], dl->len[i])) {
            LOGE(" * copy_from_user (%d)Error !\n", i);
            goto cam12mp_dl_exit1;
        }
        LOGI(" >> %03d.bin DL Start (Size:%d) >>\n", i, dl->len[i]);
        if (cam12mp_spi(p[i], NULL, dl->len[i])) {
            LOGE(" * %03d.bin SPI Send Error ! \n", i);
            goto cam12mp_dl_exit1;
        }
        if (!i) {
            mdelay(1);
            continue;
        }
        mdelay(5);
        if (cam12mp_spi(NULL, rx, sizeof(rx))) {
            LOGE(" * %03d.bin DL Receive Error ! \n", i);
            goto cam12mp_dl_exit1;
        }
        if (rx[0] != 0x07 || rx[1] != 0x01) {
            LOGE(" * %03d.bin DL Response Error ! %02X%02X\n", i, rx[0], rx[1]);
            goto cam12mp_dl_exit1;
        }
        LOGI(" << %03d.bin DL Complete. <<\n", i);
        mdelay(1);
    }
    mdelay(9);
    return 0;

cam12mp_dl_exit1:
    return -1;
}

////////////////////////////////
// Power ON
////////////////////////////////
static int cam12mp_sensor_poweron(void)
{
    struct vreg *vreg_gp1, *vreg_gp6;

    LOGI("+%s()\n", __func__);

    vreg_gp6 = vreg_get(NULL, "gp6");
    if (IS_ERR(vreg_gp6)) {
        LOGE(" - vreg_get(gp6) failed (%ld) !\n", PTR_ERR(vreg_gp6));
        return -1;
    }
    if (vreg_set_level(vreg_gp6, 1800)) {
        LOGE(" - vreg gp6 set level failed !\n");
        return -1;
    }
    vreg_gp1 = vreg_get(NULL, "gp1");
    if (IS_ERR(vreg_gp1)) {
        LOGE(" - vreg_get(gp1) failed (%ld)\n", PTR_ERR(vreg_gp1));
        return -1;
    }
    if (vreg_set_level(vreg_gp1, 2700)) {
        LOGE(" - vreg gp1 set level failed !\n");
        return -1;
    }

    sub_pmic_reg_ctrl(SUB_PMIC_REG2, SUB_PMIC_LDO_ON);
    mdelay(1);

    if (vreg_enable(vreg_gp6)) {
        LOGE(" - vreg gp6 enable failed !\n");
        goto _sensor_poweron_fail_1;
    }
    mdelay(1);

    sub_pmic_ldo_ctrl(SUB_PMIC_CAM12V, SUB_PMIC_LDO_ON);
    mdelay(1);

    gpio_set_value(CAM_18V, 1);
    mdelay(1);

    sub_pmic_ldo_ctrl(SUB_PMIC_CAM28V, SUB_PMIC_LDO_ON);
    mdelay(1);

    if (vreg_enable(vreg_gp1)) {
        LOGE(" - vreg gp1 enable failed !\n");
        goto _sensor_poweron_fail_2;
    }
    mdelay(1);

    gpio_tlmm_config(GPIO_CFG(MCLK, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), GPIO_ENABLE);
    msm_camio_clk_rate_set(9600000);
    msm_camio_camif_pad_reg_reset();

    gpio_set_value(H_STBY, 1);

    gpio_set_value(R_STBY, 1);
    mdelay(1);

    gpio_set_value(RESET, 1);
    mdelay(4);

    return 0;

_sensor_poweron_fail_2:
    sub_pmic_ldo_ctrl(SUB_PMIC_CAM28V, SUB_PMIC_LDO_OFF);
    gpio_set_value(CAM_18V, 0);
    sub_pmic_ldo_ctrl(SUB_PMIC_CAM12V, SUB_PMIC_LDO_OFF);
    vreg_disable(vreg_gp6);

_sensor_poweron_fail_1:
    sub_pmic_reg_ctrl(SUB_PMIC_REG2, SUB_PMIC_LDO_OFF);
    LOGI("-%s (fail.)\n", __func__);
    return -1;
}

////////////////////////////////
// Power OFF
////////////////////////////////
static void cam12mp_sensor_poweroff(void)
{
    LOGI("+%s()\n", __func__);

    gpio_set_value(RESET, 0);
    mdelay(1);

    gpio_tlmm_config(GPIO_CFG(MCLK, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
    mdelay(1);

    gpio_set_value(R_STBY, 0);

    gpio_set_value(H_STBY, 0);
    mdelay(1);

    pmic_vreg_pull_down_switch(ON_CMD, PM_VREG_PDOWN_GP1_ID);
    if ( vreg_disable(vreg_get(NULL, "gp1")))
            LOGE("%s: vreg disable failed !\n", __func__);
    mdelay(1);

    sub_pmic_ldo_ctrl(SUB_PMIC_CAM28V, SUB_PMIC_LDO_OFF);
    mdelay(1);

    gpio_set_value(CAM_18V, 0);
    mdelay(1);

    sub_pmic_ldo_ctrl(SUB_PMIC_CAM12V, SUB_PMIC_LDO_OFF);
    mdelay(1);

    pmic_vreg_pull_down_switch(ON_CMD, PM_VREG_PDOWN_GP6_ID);
    if (vreg_disable(vreg_get(NULL, "gp6")))
            LOGE("%s: vreg disable failed !\n", __func__);
    mdelay(1);

    sub_pmic_reg_ctrl(SUB_PMIC_REG2, SUB_PMIC_LDO_OFF);

    pmic_vreg_pull_down_switch(OFF_CMD, PM_VREG_PDOWN_GP1_ID);
    pmic_vreg_pull_down_switch(OFF_CMD, PM_VREG_PDOWN_GP6_ID);
    mdelay(10);
}

//=====================================================================
// Driver Function
//=====================================================================
//---------------------------------------------------------------------
// msm_open_control
//---------------------------------------------------------------------
int cam12mp_sensor_init(const struct msm_camera_sensor_info *data)
{
    cam12mp_ctrl = kzalloc(sizeof(struct cam12mp_ctrl), GFP_KERNEL);
    if (!cam12mp_ctrl) {
        LOGE(" -%s kzalloc() Failed!\n",__func__);
        return -ENOMEM;
    }

    if (data)
        cam12mp_ctrl->sensordata = data;

    cam12mp_ctrl->led = PM_MPP__I_SINK__LEVEL_10mA;
    cam12mp_ctrl->model = dev_model_get_model_no();

    // Sensor Power ON
    if (cam12mp_sensor_poweron() < 0) {
        kfree(cam12mp_ctrl);
        LOGI("-%s Failed.\n", __func__);
        return -1;
    }
    return 0;
}

//---------------------------------------------------------------------
// msm_ioctl_control()
//---------------------------------------------------------------------
int cam12mp_sensor_config(void __user *argp)
{
extern void adxl345_stop_ap(void);
extern void adxl345_start_ap(void);

    struct CameraSensorI2CCmdType   I2CCmd;
    struct sensor_cfg_data cfg;
    uint32_t *smem_ptr = NULL;
    int   rc = 0;

    if (copy_from_user(&cfg, (void *)argp, sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    mutex_lock(&cam12mp_mtx);
    switch (cfg.cfgtype) {
    case CFG_PWR_UP:
        adxl345_stop_ap();
        rc = cam12mp_dl(&cfg.cfg.dl);
        adxl345_start_ap();
        break;

    case CFG_GET_TEMP:
        smem_ptr = (uint32_t *)smem_alloc(SMEM_OEM_013, sizeof(int)); 
        if(smem_ptr == NULL){
            LOGE("+%s (CFG_GET_TEMP)\n", __func__);
            rc = -EINVAL;
        } else {
            cfg.cfg.temp = *smem_ptr;
            if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        }
        break;

    case CFG_SET_LED:
        cam12mp_led_control(cfg.cfg.led);
        break;

    case CFG_COMMAND:
        if (!cfg.cfg.cmd.txlen || cfg.cfg.cmd.txlen > 16 || cfg.cfg.cmd.rxlen > 64) {
            LOGI("+%s (%d)\n", __func__,cfg.cfgtype);
            rc = -EINVAL;
            break;
        }
        _I2C_LOG_ = cfg.rs;
        I2CCmd.slave_addr = 0x78;
        I2CCmd.pwdata     = cfg.cfg.cmd.tx;
        I2CCmd.wlen       = cfg.cfg.cmd.txlen;
        I2CCmd.prdata     = cfg.cfg.cmd.rx;
        I2CCmd.rlen       = cfg.cfg.cmd.rxlen;
        if (!cfg.cfg.cmd.rxlen)
            rc = camsensor_gpioi2c_write(&I2CCmd);
        else
            rc = camsensor_gpioi2c_read(&I2CCmd);

        if (!rc)
            if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        _I2C_LOG_ = 1;
        break;

    default:
        LOGI("+%s (%d)\n", __func__,cfg.cfgtype);
        rc = -EINVAL;
        break;
    }
    mutex_unlock(&cam12mp_mtx);

    if (rc) LOGI("-%s Done.(%d)\n", __func__, rc);
    return rc;
}

//---------------------------------------------------------------------
// msm_release_control()
//---------------------------------------------------------------------
int cam12mp_sensor_release(void)
{
    LOGI("+%s\n", __func__);

    mutex_lock(&cam12mp_mtx);

    cam12mp_led_control(LED_OFF);
    cam12mp_sensor_poweroff();
    kfree(cam12mp_ctrl);

    mutex_unlock(&cam12mp_mtx);

    LOGI("-%s Done.\n", __func__);
    return 0;
}

/////////////////////////////////////
// Sensor Driver Setup (Kernel Init)
/////////////////////////////////////
static int __devinit cam12mp_spi_probe(struct spi_device *spi)
{
    int rc = 0;

    cam12mp_spi_dev = spi;
    spi->bits_per_word = 16;
    rc = spi_setup(spi);
    if (rc) LOGE(" - spi_setup Error !\n");
    return rc;
}

static int __devexit cam12mp_spi_remove(struct spi_device *spi)
{
    LOGI("+%s\n", __func__);
    return 0;
}

static struct spi_driver cam12mp_spi_driver = {
    .driver     = {
        .name   = "cam12mp",
        .owner  = THIS_MODULE,
    },
    .probe      = cam12mp_spi_probe,
    .remove     = __devexit_p(cam12mp_spi_remove),
};

static int cam12mp_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{
    s->s_init = cam12mp_sensor_init;
    s->s_release = cam12mp_sensor_release;
    s->s_config  = cam12mp_sensor_config;

    pDlMem = kzalloc((896*1024), GFP_KERNEL);
    LOGI("+%s Dlmem:0x%x\n", __func__,pDlMem);
    if(!pDlMem)
        LOGE(" -%s DlMem kzalloc() Failed!\n",__func__);

    return spi_register_driver(&cam12mp_spi_driver);
}

static int __cam12mp_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, cam12mp_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __cam12mp_probe,
    .driver = {
        .name = "msm_camera_cam12mp",
        .owner = THIS_MODULE,
    },
};

static int __init cam12mp_init(void)
{
    return platform_driver_register(&msm_camera_driver);
}

module_init(cam12mp_init);
