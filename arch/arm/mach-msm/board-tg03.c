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
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mfd/tps65023.h>
#include "max8649.h"
#include <linux/power_supply.h>
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/sirc.h>
#include <mach/dma.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_touchpad.h>
#include <mach/msm_i2ckbd.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_spi.h>
#include <mach/msm_tsif.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>

#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#include "proc_comm.h"
#include "smd_private.h"
#include <linux/msm_kgsl.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif
#include "tg03_captouch.h"

#include "dev_model.h"

#define TOUCHPAD_SUSPEND    34
#define TOUCHPAD_IRQ        38

#define MSM_PMEM_SF_SIZE    0x1700000

#define SMEM_SPINLOCK_I2C   "S:6"

#define MSM_PMEM_ADSP_SIZE  0x2B96000
#define MSM_FB_SIZE         0x400000
#define MSM_AUDIO_SIZE      0x80000
#define MSM_GPU_PHYS_SIZE   SZ_2M

#ifdef CONFIG_MSM_SOC_REV_A
#define MSM_SMI_BASE        0xE0000000
#else
#define MSM_SMI_BASE        0x00000000
#endif

#define MSM_SHARED_RAM_PHYS (MSM_SMI_BASE + 0x00100000)

#define MSM_PMEM_SMI_BASE   (MSM_SMI_BASE + 0x02B00000)
#define MSM_PMEM_SMI_SIZE   0x01500000

#define MSM_FB_BASE     MSM_PMEM_SMI_BASE
#define MSM_GPU_PHYS_BASE   (MSM_FB_BASE + MSM_FB_SIZE)
#define MSM_PMEM_SMIPOOL_BASE   (MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE   (MSM_PMEM_SMI_SIZE - MSM_FB_SIZE \
                    - MSM_GPU_PHYS_SIZE)

#define PMEM_KERNEL_EBI1_SIZE   0x28000

/* WLAN_0012_S */
#define PMIC_VREG_WLAN_LEVEL    2900
/* WLAN_0012_E */
#define PMIC_VREG_GP6_LEVEL 2900

#define FPGA_SDCC_STATUS    0x70000280

/* WLAN_0012_S */
static int vreg_wlan_refcount ;
int Wlan_vreg_enable(void);
int Wlan_vreg_disable(void);
/* WLAN_0012_E */

/* Vibrator Intialization */
void msm_init_pmic_vibrator(void);

int msm_prox_read_nvitem(unsigned int id);

static struct resource smc91x_resources[] = {
    [0] = {
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .flags  = IORESOURCE_IRQ,
    },
};


static struct platform_device smc91x_device = {
    .name           = "smc91x",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(smc91x_resources),
    .resource       = smc91x_resources,
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
    {"diag", 0},
    {"adb", 1},
    {"modem", 2},
    {"nmea", 3},
    {"mass_storage", 4},
    {"ethernet", 5},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
    {
        .product_id         = 0x9012,
        .functions      = 0x5, /* 0101 */
    },

    {
        .product_id         = 0x9013,
        .functions      = 0x15, /* 10101 */
    },

    {
        .product_id         = 0x9014,
        .functions      = 0x30, /* 110000 */
    },

    {
        .product_id         = 0x9015,
        .functions      = 0x12, /* 10010 */
    },

    {
        .product_id         = 0x9016,
        .functions      = 0xD, /* 01101 */
    },

    {
        .product_id         = 0x9017,
        .functions      = 0x1D, /* 11101 */
    },

    {
        .product_id         = 0xF000,
        .functions      = 0x10, /* 10000 */
    },

    {
        .product_id         = 0xF009,
        .functions      = 0x20, /* 100000 */
    },

    {
        .product_id         = 0x9018,
        .functions      = 0x1F, /* 011111 */
    },

    {
        .product_id         = 0x901A,
        .functions      = 0x0F, /* 01111 */
    },
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
    .hs_name = "8k_handset",
    .pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
    .name   = "msm-handset",
    .id     = -1,
    .dev    = {
        .platform_data = &hs_platform_data,
    },
};

#ifdef CONFIG_USB_FS_HOST
static struct msm_gpio fsusb_config[] = {
    { GPIO_CFG(139, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_dat" },
    { GPIO_CFG(140, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_se0" },
    { GPIO_CFG(141, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_oe_n" },
};

static int fsusb_gpio_init(void)
{
    return msm_gpios_request(fsusb_config, ARRAY_SIZE(fsusb_config));
}

static void msm_fsusb_setup_gpio(unsigned int enable)
{
    if (enable)
        msm_gpios_enable(fsusb_config, ARRAY_SIZE(fsusb_config));
    else
        msm_gpios_disable(fsusb_config, ARRAY_SIZE(fsusb_config));

}
#endif

#define MSM_USB_BASE              ((unsigned)addr)

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
    .version    = 0x0100,
    .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
    .vendor_id          = 0x5c6,
    .product_name       = "Qualcomm HSUSB Device",
    .serial_number      = "1234567890ABCDEF",
    .manufacturer_name  = "Qualcomm Incorporated",
    .compositions   = usb_func_composition,
    .num_compositions = ARRAY_SIZE(usb_func_composition),
    .function_map   = usb_functions_map,
    .num_functions  = ARRAY_SIZE(usb_functions_map),
    .config_gpio    = NULL,

#endif
};

static struct vreg *vreg_usb;
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{

    switch (PHY_TYPE(phy_info)) {
    case USB_PHY_INTEGRATED:
        if (on)
            msm_hsusb_vbus_powerup();
        else
            msm_hsusb_vbus_shutdown();
        break;
    case USB_PHY_SERIAL_PMIC:
        if (on)
            vreg_enable(vreg_usb);
        else
            vreg_disable(vreg_usb);
        break;
    default:
        pr_err("%s: undefined phy type ( %X ) \n", __func__,
                        phy_info);
    }

}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
    .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
    .vbus_power = msm_hsusb_vbus_power,
};

#ifdef CONFIG_USB_FS_HOST
static struct msm_usb_host_platform_data msm_usb_host2_pdata = {
    .phy_info   = USB_PHY_SERIAL_PMIC,
    .config_gpio = msm_fsusb_setup_gpio,
    .vbus_power = msm_hsusb_vbus_power,
};
#endif

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
    .name = PMEM_KERNEL_EBI1_DATA_NAME,
    /* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
     * the only valid choice at this time. The board structure is
     * set to all zeros by the C runtime initialization and that is now
     * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
     * include/linux/android_pmem.h.
     */
    .cached = 0,
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION

static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
    .name = PMEM_KERNEL_SMI_DATA_NAME,
    /* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
     * the only valid choice at this time. The board structure is
     * set to all zeros by the C runtime initialization and that is now
     * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
     * include/linux/android_pmem.h.
     */
    .cached = 0,
};

#endif

static struct android_pmem_platform_data android_pmem_pdata = {
    .name = "pmem",
    .allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
    .cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
    .name = "pmem_adsp",
    .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
    .cached = 0,
};

static struct android_pmem_platform_data android_pmem_smipool_pdata = {
    .name = "pmem_smipool",
    .start = MSM_PMEM_SMIPOOL_BASE,
    .size = MSM_PMEM_SMIPOOL_SIZE,
    .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
    .cached = 0,
};


static struct platform_device android_pmem_device = {
    .name = "android_pmem",
    .id = 0,
    .dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
    .name = "android_pmem",
    .id = 1,
    .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_smipool_device = {
    .name = "android_pmem",
    .id = 2,
    .dev = { .platform_data = &android_pmem_smipool_pdata },
};


static struct platform_device android_pmem_kernel_ebi1_device = {
    .name = "android_pmem",
    .id = 3,
    .dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct platform_device android_pmem_kernel_smi_device = {
    .name = "android_pmem",
    .id = 4,
    .dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

static struct resource msm_fb_resources[] = {
    {
        .flags  = IORESOURCE_DMA,
    }
};

static int msm_fb_detect_panel(const char *name)
{
    int ret = -EPERM;

    if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
        if (!strncmp(name, "mddi_toshiba_wvga_pt", 20))
            ret = 0;
        else
            ret = -ENODEV;
    } else if ((machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf())
            && !strcmp(name, "lcdc_external"))
        ret = 0;

    return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
    .detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
    .name   = "msm_fb",
    .id     = 0,
    .num_resources  = ARRAY_SIZE(msm_fb_resources),
    .resource       = msm_fb_resources,
    .dev    = {
        .platform_data = &msm_fb_pdata,
    }
};


static struct resource qsd_spi_resources[] = {
    {
        .name   = "spi_irq_in",
        .start  = INT_SPI_INPUT,
        .end    = INT_SPI_INPUT,
        .flags  = IORESOURCE_IRQ,
    },
    {
        .name   = "spi_irq_out",
        .start  = INT_SPI_OUTPUT,
        .end    = INT_SPI_OUTPUT,
        .flags  = IORESOURCE_IRQ,
    },
    {
        .name   = "spi_irq_err",
        .start  = INT_SPI_ERROR,
        .end    = INT_SPI_ERROR,
        .flags  = IORESOURCE_IRQ,
    },
    {
        .name   = "spi_base",
        .start  = 0xA1200000,
        .end    = 0xA1200000 + SZ_4K - 1,
        .flags  = IORESOURCE_MEM,
    },
    {
        .name   = "spidm_channels",
        .flags  = IORESOURCE_DMA,
    },
    {
        .name   = "spidm_crci",
        .flags  = IORESOURCE_DMA,
    },
};

static struct platform_device qsd_device_spi = {
    .name           = "spi_qsd",
    .id         = 0,
    .num_resources  = ARRAY_SIZE(qsd_spi_resources),
    .resource   = qsd_spi_resources,
};

static struct spi_board_info msm_spi_board_info[] __initdata = {
    {
        .modalias      = "cam12mp",
        .mode          = SPI_MODE_3,
        .bus_num       = 0,
        .chip_select   = 3,
        .max_speed_hz  = 26330000,
    },
#if 1
    {
        .modalias   = "adxl345",
        .mode       = SPI_MODE_3,
        .bus_num    = 0,
        .chip_select    = 0,
        .max_speed_hz   = 4800000,
    },
#endif

};

#define CT_CSR_PHYS     0xA8700000
#define TCSR_SPI_MUX        (ct_csr_base + 0x54)
static int msm_qsd_spi_dma_config(void)
{
    void __iomem *ct_csr_base = 0;
    u32 spi_mux;
    int ret = 0;

    ct_csr_base = ioremap(CT_CSR_PHYS, PAGE_SIZE);
    if (!ct_csr_base) {
        pr_err("%s: Could not remap %x\n", __func__, CT_CSR_PHYS);
        return -1;
    }

    spi_mux = readl(TCSR_SPI_MUX);
    switch (spi_mux) {
    case (1):
        qsd_spi_resources[4].start  = DMOV_HSUART1_RX_CHAN;
        qsd_spi_resources[4].end    = DMOV_HSUART1_TX_CHAN;
        qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
        qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
        break;
    case (2):
        qsd_spi_resources[4].start  = DMOV_HSUART2_RX_CHAN;
        qsd_spi_resources[4].end    = DMOV_HSUART2_TX_CHAN;
        qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
        qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
        break;
    case (3):
        qsd_spi_resources[4].start  = DMOV_CE_OUT_CHAN;
        qsd_spi_resources[4].end    = DMOV_CE_IN_CHAN;
        qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
        qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
        break;
    default:
        ret = -1;
    }

    iounmap(ct_csr_base);
    return ret;
}

static struct msm_gpio qsd_spi_gpio_config_data[] = {
    { GPIO_CFG(17, 1, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), "spi_clk" },
    { GPIO_CFG(18, 1, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), "spi_mosi" },
    { GPIO_CFG(19, 1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), "spi_miso" },
    { GPIO_CFG(20, 1, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), "spi_cs0" },
#ifdef CONFIG_TG03_CAMERA
 #ifdef CONFIG_CAM12MP
    { GPIO_CFG(145, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "spi_cs3" },
 #endif
#endif
};

static int msm_qsd_spi_gpio_config(void)
{
    int rc;
    int model = dev_model_get_model_no();
    if(model == DEV_MODEL_NO_0 || model == DEV_MODEL_NO_1) {
        qsd_spi_gpio_config_data[1].gpio_cfg
            = GPIO_CFG(18, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA);
    }

    rc = msm_gpios_request_enable(qsd_spi_gpio_config_data,
        ARRAY_SIZE(qsd_spi_gpio_config_data));
    if (rc)
        return rc;

    return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
    int model = dev_model_get_model_no();
    if(model == DEV_MODEL_NO_0 || model == DEV_MODEL_NO_1) {
        qsd_spi_gpio_config_data[1].gpio_cfg
            = GPIO_CFG(18, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA);
    }

    msm_gpios_disable_free(qsd_spi_gpio_config_data,
        ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
    .max_clock_speed = 26330000,
    .clk_name = "spi_clk",
    .gpio_config  = msm_qsd_spi_gpio_config,
    .gpio_release = msm_qsd_spi_gpio_release,
    .dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
    qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

static int mddi_toshiba_pmic_bl(int level)
{
    int ret = -EPERM;

    if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
        ret = pmic_set_led_intensity(LED_LCD, level);

        if (ret)
            printk(KERN_WARNING "%s: can't set lcd backlight!\n",
                        __func__);
    }

    return ret;
}

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
    .pmic_backlight = mddi_toshiba_pmic_bl,
};

static struct platform_device mddi_toshiba_device = {
    .name   = "mddi_toshiba",
    .id     = 0,
    .dev    = {
        .platform_data = &mddi_toshiba_pdata,
    }
};

static void msm_fb_vreg_config(const char *name, int on)
{
    struct vreg *vreg;
    int ret = 0;

    vreg = vreg_get(NULL, name);
    if (IS_ERR(vreg)) {
        printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
        __func__, name, PTR_ERR(vreg));
        return;
    }

    ret = (on) ? vreg_enable(vreg) : vreg_disable(vreg);
    if (ret)
        printk(KERN_ERR "%s: %s(%s) failed!\n",
            __func__, (on) ? "vreg_enable" : "vreg_disable", name);
}

#define MDDI_RST_OUT_GPIO 100

static int mddi_power_save_on;
static int msm_fb_mddi_power_save(int on)
{
    int flag_on = !!on;
    int ret = 0;


    if (mddi_power_save_on == flag_on)
        return ret;

    mddi_power_save_on = flag_on;

    if (!flag_on && (machine_is_qsd8x50_ffa()
                || machine_is_qsd8x50a_ffa())) {
        gpio_set_value(MDDI_RST_OUT_GPIO, 0);
        mdelay(1);
    }

    ret = pmic_lp_mode_control(flag_on ? OFF_CMD : ON_CMD,
        PM_VREG_LP_MSME2_ID);
    if (ret)
        printk(KERN_ERR "%s: pmic_lp_mode_control failed!\n", __func__);

    msm_fb_vreg_config("gp5", flag_on);
    msm_fb_vreg_config("boost", flag_on);

    if (flag_on && (machine_is_qsd8x50_ffa()
            || machine_is_qsd8x50a_ffa())) {
        gpio_set_value(MDDI_RST_OUT_GPIO, 0);
        mdelay(1);
        gpio_set_value(MDDI_RST_OUT_GPIO, 1);
        gpio_set_value(MDDI_RST_OUT_GPIO, 1);
        mdelay(1);
    }

    return ret;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
    *clk_rate *= 2;
    return 0;
}

static struct mddi_platform_data mddi_pdata = {
    .mddi_power_save = NULL,
    .mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
    .gpio = 98,
};

static void __init msm_fb_add_devices(void)
{
    msm_fb_register_device("mdp", &mdp_pdata);
    msm_fb_register_device("pmdh", &mddi_pdata);
    msm_fb_register_device("emdh", &mddi_pdata);
    msm_fb_register_device("lcdc", 0);
}

static struct resource msm_audio_resources[] = {
    {
        .flags  = IORESOURCE_DMA,
    },
    {
        .name   = "aux_pcm_dout",
        .start  = 68,
        .end    = 68,
        .flags  = IORESOURCE_IO,
    },
    {
        .name   = "aux_pcm_din",
        .start  = 69,
        .end    = 69,
        .flags  = IORESOURCE_IO,
    },
    {
        .name   = "aux_pcm_syncout",
        .start  = 70,
        .end    = 70,
        .flags  = IORESOURCE_IO,
    },
    {
        .name   = "aux_pcm_clkin_a",
        .start  = 71,
        .end    = 71,
        .flags  = IORESOURCE_IO,
    },
    {
        .name   = "audio_base_addr",
        .start  = 0xa0700000,
        .end    = 0xa0700000 + 4,
        .flags  = IORESOURCE_MEM,
    },

};

static unsigned audio_gpio_on[] = {
    GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_DOUT */
    GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_SYNC */
    GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_CLK */
};

static void __init audio_gpio_init(void)
{
    int pin, rc;

    for (pin = 0; pin < ARRAY_SIZE(audio_gpio_on); pin++) {
        rc = gpio_tlmm_config(audio_gpio_on[pin],
            GPIO_ENABLE);
        if (rc) {
            printk(KERN_ERR
                "%s: gpio_tlmm_config(%#x)=%d\n",
                __func__, audio_gpio_on[pin], rc);
            return;
        }
    }
}

static struct platform_device msm_audio_device = {
    .name   = "msm_audio",
    .id     = 0,
    .num_resources  = ARRAY_SIZE(msm_audio_resources),
    .resource       = msm_audio_resources,
};

static struct resource bluesleep_resources[] = {
    {
        .name   = "gpio_host_wake",
        .start  = 36,
        .end    = 36,
        .flags  = IORESOURCE_IO,
    },
    {
        .name   = "gpio_ext_wake",
        .start  = 42,
        .end    = 42,
        .flags  = IORESOURCE_IO,
    },
/* New GPIO resources for Bluesleep: Added Start */
    {
            .name   = "gpio_uart_rfr",
            .start  = 43,
            .end    = 43,
            .flags  = IORESOURCE_IO,
        },
        {
            .name   = "gpio_uart_cts",
            .start  = 44,
            .end    = 44,
            .flags  = IORESOURCE_IO,
        },
        {
            .name   = "gpio_uart_rx",
            .start  = 45,
            .end    = 45,
        .flags  = IORESOURCE_IO,
        },
        {
        .name   = "gpio_uart_tx",
            .start  = 46,
            .end    = 46,
            .flags  = IORESOURCE_IO,
        },
        {
            .name   = "gpio_reset_n",
            .start  = 81,
            .end    = 81,
            .flags  = IORESOURCE_IO,
        },
/* New GPIO resources for Bluesleep: Added End */
    {
        .name   = "host_wake",
        .start  = MSM_GPIO_TO_INT(36),
        .end    = MSM_GPIO_TO_INT(36),
        .flags  = IORESOURCE_IRQ,
    },
};

static struct platform_device msm_bluesleep_device = {
    .name = "bluesleep",
    .id     = -1,
    .num_resources  = ARRAY_SIZE(bluesleep_resources),
    .resource   = bluesleep_resources,
};

#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
    .name = "bt_power",
};

enum {
    BT_SYSRST,
    BT_WAKE,
    BT_HOST_WAKE,
    BT_VDD_IO,
    BT_RFR,
    BT_CTS,
    BT_RX,
    BT_TX,
    BT_VDD_FREG
};
/* Modified BT_OFF settings to that of Sleep Mode : Start */

static unsigned bt_config_power_off[] = {
        GPIO_CFG(81, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   //BT_RESET_N
        GPIO_CFG(42, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   //BT_WAKEUP
        GPIO_CFG(36, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),   //BT_HOST_WAKEUP
        GPIO_CFG(46, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   //UART1_TX
        GPIO_CFG(45, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),    //UART1_RX
        GPIO_CFG(44, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),    //UART1_CTS
        GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   //UART1_RFR
};

static struct msm_gpio wlan_config_power_off[] = {
    { GPIO_CFG(62, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "SDC2_CLK" },
    { GPIO_CFG(63, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "SDC2_CMD" },
    { GPIO_CFG(64, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "SDC2_D3" },
    { GPIO_CFG(65, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "SDC2_D2" },
    { GPIO_CFG(66, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "SDC2_D1" },
    { GPIO_CFG(67, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "SDC2_D0" },
    { GPIO_CFG(113, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "VDD_WLAN" },
    { GPIO_CFG(138, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
        "WLAN_PWD" }
};

static struct msm_gpio wlan_config_power_on[] = {
    { GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        "SDC2_CLK" },
    { GPIO_CFG(63, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
        "SDC2_CMD" },
    { GPIO_CFG(64, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
        "SDC2_D3" },
    { GPIO_CFG(65, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
        "SDC2_D2" },
    { GPIO_CFG(66, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
        "SDC2_D1" },
    { GPIO_CFG(67, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
        "SDC2_D0" },
    { GPIO_CFG(113, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        "VDD_WLAN" },
    { GPIO_CFG(138, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        "WLAN_PWD" }
};

static int bluetooth_status = 0;
static int is_reboot = 1;

static int bluetooth_power(int on)
{
    struct vreg *vreg_bt;
    int pin, rc;
    int Wlan_vreg_Status = 0;

    printk(KERN_ERR "%s: on = %d \n", __func__, on);
    /* do not have vreg bt defined, gp6 is the same */
    /* vreg_get parameter 1 (struct device *) is ignored */
    vreg_bt = vreg_get(NULL, "gp6");

    if (IS_ERR(vreg_bt)) {
        printk(KERN_ERR "%s: vreg get failed (%ld)\n",
               __func__, PTR_ERR(vreg_bt));
        return PTR_ERR(vreg_bt);
    }
    /* If BT is already ON, we will not switch on again */
    if (bluetooth_status == on)
    {
        if (is_reboot != 1)
        {
            return 0;
        }
    }
    is_reboot = 0;
    bluetooth_status = on;

    if (on) {
        /* Apply NO_PULL on Reset Pin: Start */
        gpio_tlmm_config(GPIO_CFG(81, 0, GPIO_OUTPUT, GPIO_NO_PULL,GPIO_2MA),GPIO_ENABLE);
        /* Apply NO_PULL on Reset Pin: End */
        gpio_tlmm_config(GPIO_CFG(42, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);

        gpio_direction_output(42, 1);
        /* units of mV, steps of 50 mV */

        Wlan_vreg_Status = Wlan_vreg_enable();
        if(Wlan_vreg_Status < 0)
        {
            printk(KERN_ERR "%s: Wlan_vreg_enable Wlan_vreg_Status (%d)\n", __func__, Wlan_vreg_Status);
            return -EIO;
        }

        gpio_tlmm_config(GPIO_CFG(36, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);
        msleep(10);
        gpio_set_value(81, 1);      /* Normal Operation */
        msleep(10);

        gpio_tlmm_config(GPIO_CFG(36, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);


        gpio_tlmm_config(GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
        gpio_tlmm_config(GPIO_CFG(44, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
        gpio_tlmm_config(GPIO_CFG(45, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
        gpio_tlmm_config(GPIO_CFG(46, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
        gpio_set_value(42, 0); /* BT Weakup  */

        }

    else {
        gpio_set_value(42, 1); //BT_WAKEUP
        gpio_set_value(36, 0);//HOST_WAKE
        gpio_set_value(81, 0); //BT_RESET_N
        msleep(10); //50

        Wlan_vreg_Status = Wlan_vreg_disable();
        if(Wlan_vreg_Status < 0)
        {
            printk(KERN_ERR "%s: Wlan_vreg_disable Wlan_vreg_Status (%d)\n", __func__, Wlan_vreg_Status);
            return -EIO;
        }

        for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
            rc = gpio_tlmm_config(bt_config_power_off[pin],
                          GPIO_ENABLE);
            if (rc) {
                printk(KERN_ERR
                       "%s: gpio_tlmm_config(%#x)=%d\n",
                       __func__, bt_config_power_off[pin], rc);
                return -EIO;
            }
        }


    }

    printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

    return 0;
}

static void __init bt_power_init(void)
{
    msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif

static struct resource kgsl_resources[] = {
       {
        .name  = "kgsl_reg_memory",
        .start = 0xA0000000,
        .end = 0xA001ffff,
        .flags = IORESOURCE_MEM,
       },
       {
        .name   = "kgsl_phys_memory",
        .start = MSM_GPU_PHYS_BASE,
        .end = MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE - 1,
        .flags = IORESOURCE_MEM,
       },
       {
        .name = "kgsl_yamato_irq",
        .start = INT_GRAPHICS,
        .end = INT_GRAPHICS,
        .flags = IORESOURCE_IRQ,
       },
};
static struct kgsl_platform_data kgsl_pdata = {
    .high_axi_3d = 128000, /* Max for 8K */
    .max_grp2d_freq = 0,
    .min_grp2d_freq = 0,
    .set_grp2d_async = NULL,
    .max_grp3d_freq = 0,
    .min_grp3d_freq = 0,
    .set_grp3d_async = NULL,
    .imem_clk_name = "imem_clk",
    .grp3d_clk_name = "grp_clk",
    .grp2d_clk_name = NULL,
};

static struct platform_device msm_device_kgsl = {
       .name = "kgsl",
       .id = -1,
       .num_resources = ARRAY_SIZE(kgsl_resources),
       .resource = kgsl_resources,
    .dev = {
        .platform_data = &kgsl_pdata,
    },
};

#ifdef CONFIG_LEDS_MSM_PMIC
static struct platform_device msm_device_pmic_leds = {
    .name   = "pmic-leds",
    .id = -1,
};
#endif

static struct platform_device tg03_i2c_led = {
        .name   = "i2c-leds",
        .id     = -1,
};

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_A_SYNC      GPIO_CFG(106, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
#define TSIF_A_DATA      GPIO_CFG(107, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
#define TSIF_A_EN        GPIO_CFG(108, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
#define TSIF_A_CLK       GPIO_CFG(109, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)

static const struct msm_gpio tsif_gpios[] = {
    { .gpio_cfg = TSIF_A_CLK,  .label =  "tsif_clk", },
    { .gpio_cfg = TSIF_A_EN,   .label =  "tsif_en", },
    { .gpio_cfg = TSIF_A_DATA, .label =  "tsif_data", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
    .num_gpios = ARRAY_SIZE(tsif_gpios),
    .gpios = tsif_gpios,
    .tsif_clk = "tsif_clk",
    .tsif_ref_clk = "tsif_ref_clk",
};

#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

#ifdef CONFIG_QSD_SVS
#define TPS65023_MAX_DCDC1  1600
#else
#define TPS65023_MAX_DCDC1  CONFIG_QSD_PMIC_DEFAULT_DCDC1
#endif

static int qsd8x50_tps65023_set_dcdc1(int mVolts)
{
    int rc = 0;
#ifdef CONFIG_QSD_SVS
    rc = tps65023_set_dcdc1_level(mVolts);
    /* By default the TPS65023 will be initialized to 1.225V.
     * So we can safely switch to any frequency within this
     * voltage even if the device is not probed/ready.
     */
    if (rc == -ENODEV && mVolts <= CONFIG_QSD_PMIC_DEFAULT_DCDC1)
        rc = 0;
#else
    /* Disallow frequencies not supported in the default PMIC
     * output voltage.
     */
    if (mVolts > CONFIG_QSD_PMIC_DEFAULT_DCDC1)
        rc = -EFAULT;
#endif
    return rc;
}

#define MAX8649_MAX_DCDC2   1380

static int qsd8x50_max8649_set_dcdc2(int mVolts)
{
    int rc = 0;
    rc = max8649_set_dcdc2_level(mVolts);
    if (rc == -ENODEV && mVolts <= MAX8649_MAX_DCDC2)
        rc = 0;
    return rc;
}

static struct msm_acpu_clock_platform_data qsd8x50_clock_data = {
    .acpu_switch_time_us = 20,
    .max_speed_delta_khz = 256000,
    .vdd_switch_time_us = 62,
    .max_vdd = MAX8649_MAX_DCDC2,
    .acpu_set_vdd = qsd8x50_max8649_set_dcdc2,
};

#define GPIO_TCH_TSPOW_ON   104
#define GPIO_TCH_SCL        146
#define GPIO_TCH_SDA        147
#define GPIO_TCH_RST        48
#define GPIO_TCH_INT        157

static void tg03_cap_vreg_config_m0( int on )
{
    if ( on ){
        gpio_set_value( GPIO_TCH_TSPOW_ON, 0 );   /* TSPOW_ON = PowerOFF */
        gpio_set_value( GPIO_TCH_RST, 0 );   /* Reset */
        msleep( 100 );
        gpio_set_value( GPIO_TCH_TSPOW_ON, 1 );   /* TSPOW_ON = PowerON */
        msleep( 100 );
        gpio_set_value( GPIO_TCH_RST, 1 );   /* Reset Off  */
    }else{
        gpio_set_value( GPIO_TCH_TSPOW_ON, 0 );   /* TSPOW_ON = PowerOFF */
        gpio_set_value( GPIO_TCH_RST, 0 );   /* Reset    */
    }
}

static void tg03_cap_vreg_config( int on )
{
    struct vreg *vreg_ts;
    int rc;

    if( dev_model_get_model_no() == DEV_MODEL_NO_0 ){
        tg03_cap_vreg_config_m0( on );
        return ;
    }

    vreg_ts = vreg_get( NULL, "gp3" );
    if ( on ){
        rc = vreg_set_level( vreg_ts, 2700 );
        if (rc) {
            pr_err( "%s: vreg gp3 set level failed (%d)\n", __func__, rc );
        }

        gpio_set_value( GPIO_TCH_RST, 0 );   /* Reset */

        rc = vreg_enable( vreg_ts );
        if (rc) {
            pr_err( "%s: vreg gp3 enable failed (%d)\n", __func__, rc );
        }
        msleep( 40 );
        gpio_set_value( GPIO_TCH_RST, 1 );   /* Reset Off */
    }else{
        gpio_set_value( GPIO_TCH_RST, 0 );   /* Reset */
        rc = vreg_disable( vreg_ts );
        if (rc) {
            pr_err( "%s: vreg gp3 disable failed (%d)\n", __func__, rc );
        }
    }
}

static uint32_t msm_ts_gpio_table[] = {
    GPIO_CFG( GPIO_TCH_SCL, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* TS I2C SERIAL CLOCK */
    GPIO_CFG( GPIO_TCH_SDA, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* TS I2C SERIAL DATA */
    GPIO_CFG( GPIO_TCH_RST, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* RESET */
    GPIO_CFG( GPIO_TCH_INT, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA)  /* INTERRUPT */
};

static int tg03_captouch_init(void)
{
    int n, rc;

    /* request */
    rc = gpio_request( GPIO_TCH_SCL, "gpio_ts_scl" );
    if (rc) {
        pr_err("gpio_request failed on SCL(146) (rc=%d)\n", rc);
    }
    rc = gpio_request( GPIO_TCH_SDA, "gpio_ts_sda" );
    if (rc) {
        pr_err("gpio_request failed on SDA(147) (rc=%d)\n", rc);
    }
    rc = gpio_request( GPIO_TCH_RST, "gpio_ts_reset" );
    if (rc) {
        pr_err("gpio_request failed on INT(48) (rc=%d)\n", rc);
    }
    rc = gpio_request( GPIO_TCH_INT, "gpio_ts_int" );
    if (rc) {
        pr_err("gpio_request failed on RESET(157) (rc=%d)\n", rc);
    }

    /* config */
    for (n = 0; n < ARRAY_SIZE(msm_ts_gpio_table); n++) {
        rc = gpio_tlmm_config(msm_ts_gpio_table[n], GPIO_ENABLE);
        if (rc) {
            printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, msm_ts_gpio_table[n], rc );
        }
    }


    return rc;
}

tg03_captouch_platfrom_data tg03_cap_pdata[] =
{
    {
        /* MODEL_NO_1_SWC */
        .x_max        = 1023,
        .y_max        = 1023,
        .pressure_max = 255,
        .max_point    = 3,
        .gpio_int     = 157,
        .gpio_reset   = 48,
        .touch_addr   = 0x4A,
        .hw_config    = {
            .config_crc   = 0x00de0272,
            .config_t38   = {   0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t7    = { 11, 11, 50 },
            .config_t8    = {  10,  5, 20, 20,  0,  0, 10, 15 },
            .config_t9    = { 131,  0,  0, 19, 11,  0, 17, 50,  4,  4,  0,  1,  1,  0,  3, 10, 25, 10,  0,  0,  0,  0,  1,  1, 13, 13,153, 45,153, 70, 50 },
            .config_t15   = {   0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0 },
            .config_t18   = {   0,  0 },
            .config_t19   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t20   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t22   = {  31,  0,  0, 30,  0,226,255,  0, 20,  0,  0,  6, 11, 15, 19, 21,  0 },
            .config_t23   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t24   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t25   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t27   = {   0,  0,  0,  0,  0,  0,  0 },
            .config_t28   = {   0,  0,  3,  4,  8, 15 },
            .resume_atchcalsthr = 15,
        },
        .vreg_config  = &tg03_cap_vreg_config,
    },
    {
        /* MODEL_NO_PR_WTK */
        .x_max        = 1023,
        .y_max        = 1023,
        .pressure_max = 255,
        .max_point    = 3,
        .gpio_int     = 157,
        .gpio_reset   = 48,
        .touch_addr   = 0x4A,
        .hw_config    = {
            .config_crc   = 0x0016701d,
            .config_t38   = {   1,  0, 89, 71,160,  0,  0,  0 },
            .config_t7    = {   8,255, 50 },
            .config_t8    = {   8,  5,  5,  5,  0,  0,  0,  1 },
            .config_t9    = { 131,  0,  0, 19, 11,  0, 17, 40,  2,  4,  0,  3,  1, 48,  3, 10, 10, 10,  0,  0,  0,  0,  2,  2, 13, 13,152, 50,162, 70, 20 },
            .config_t15   = {   0,  0,  0,  1,  1,  0, 65, 30,  2,  0,  0 },
            .config_t18   = {   0,  0 },
            .config_t19   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t20   = {   7,  1,  1,  1,  1,  3,  0, 15,  0,  0,  0,  0 },
            .config_t22   = {   7,  0,  0,  0,  0,  0,  0,  0, 20,  0,  0,  6, 11, 15, 19, 21,  0 },
            .config_t23   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t24   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t25   = {   3,  0,248, 42,124, 21,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t27   = {   0,  0,  0,  0,  0,  0,  0 },
            .config_t28   = {   0,  0,  3,  0,  0,  0 },
            .resume_atchcalsthr = 40,
        },
        .vreg_config  = &tg03_cap_vreg_config,
    },
    {
        /* MODEL_NO_PR_SWC */
        .x_max        = 1023,
        .y_max        = 1023,
        .pressure_max = 255,
        .max_point    = 3,
        .gpio_int     = 157,
        .gpio_reset   = 48,
        .touch_addr   = 0x4A,
        .hw_config    = {
            .config_crc   = 0x008ab4b8,
            .config_t38   = {   0,  0, 65, 73,138,  0,  0,  0 },
            .config_t7    = {   8,255, 50 },
            .config_t8    = {   8,  5,  5,  5,  0,  0,  0,  1 },
            .config_t9    = { 131,  0,  0, 19, 11,  0, 49, 60,  2,  4,  0,  3,  1, 48,  3, 10, 10, 10,  0,  0,  0,  0,  0,  0, 15, 15,152, 50,160, 75, 20 },
            .config_t15   = {   0,  0,  0,  1,  1,  0, 65, 30,  2,  0,  0 },
            .config_t18   = {   0,  0 },
            .config_t19   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t20   = {   7,  1,  1,  1,  1,  3,  0, 15,  0,  0,  0,  0 },
            .config_t22   = {   7,  0,  0,  0,  0,  0,  0,  0, 20,  0,  0,  6, 11, 15, 19, 21,  0 },
            .config_t23   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t24   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t25   = {   3,  0,248, 42,124, 21,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t27   = {   0,  0,  0,  0,  0,  0,  0 },
            .config_t28   = {   0,  0,  3,  0,  0,  0 },
            .resume_atchcalsthr = 60,
        },
        .vreg_config  = &tg03_cap_vreg_config,
    },
    {
        /* MODEL_NO_PR_NONE */
        .x_max        = 1023,
        .y_max        = 1023,
        .pressure_max = 255,
        .max_point    = 3,
        .gpio_int     = 157,
        .gpio_reset   = 48,
        .touch_addr   = 0x4A,
        .hw_config    = {
            .config_crc   = 0x0016701d,
            .config_t38   = {   1,  0, 89, 71,160,  0,  0,  0 },
            .config_t7    = {   8,255, 50 },
            .config_t8    = {   8,  5,  5,  5,  0,  0,  0,  1 },
            .config_t9    = { 131,  0,  0, 19, 11,  0, 17, 40,  2,  4,  0,  3,  1, 48,  3, 10, 10, 10,  0,  0,  0,  0,  2,  2, 13, 13,152, 50,162, 70, 20 },
            .config_t15   = {   0,  0,  0,  1,  1,  0, 65, 30,  2,  0,  0 },
            .config_t18   = {   0,  0 },
            .config_t19   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t20   = {   7,  1,  1,  1,  1,  3,  0, 15,  0,  0,  0,  0 },
            .config_t22   = {   7,  0,  0,  0,  0,  0,  0,  0, 20,  0,  0,  6, 11, 15, 19, 21,  0 },
            .config_t23   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t24   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t25   = {   3,  0,248, 42,124, 21,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t27   = {   0,  0,  0,  0,  0,  0,  0 },
            .config_t28   = {   0,  0,  3,  0,  0,  0 },
            .resume_atchcalsthr = 40,
        },
        .vreg_config  = &tg03_cap_vreg_config,
    }
};

struct platform_device tg03_cap_device = {
    .name   = "tg03_captouch",
    .id     = 0,
    .dev = {
        .platform_data = tg03_cap_pdata,
    },
};

static void touchpad_gpio_release(void)
{
    gpio_free(TOUCHPAD_IRQ);
    gpio_free(TOUCHPAD_SUSPEND);
}

static int touchpad_gpio_setup(void)
{
    int rc;
    int suspend_pin = TOUCHPAD_SUSPEND;
    int irq_pin = TOUCHPAD_IRQ;
    unsigned suspend_cfg =
        GPIO_CFG(suspend_pin, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
    unsigned irq_cfg =
        GPIO_CFG(irq_pin, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);

    rc = gpio_request(irq_pin, "msm_touchpad_irq");
    if (rc) {
        pr_err("gpio_request failed on pin %d (rc=%d)\n",
               irq_pin, rc);
        goto err_gpioconfig;
    }
    rc = gpio_request(suspend_pin, "msm_touchpad_suspend");
    if (rc) {
        pr_err("gpio_request failed on pin %d (rc=%d)\n",
               suspend_pin, rc);
        goto err_gpioconfig;
    }
    rc = gpio_tlmm_config(suspend_cfg, GPIO_ENABLE);
    if (rc) {
        pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
               suspend_pin, rc);
        goto err_gpioconfig;
    }
    rc = gpio_tlmm_config(irq_cfg, GPIO_ENABLE);
    if (rc) {
        pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
               irq_pin, rc);
        goto err_gpioconfig;
    }
    return rc;

err_gpioconfig:
    touchpad_gpio_release();
    return rc;
}

static struct msm_touchpad_platform_data msm_touchpad_data = {
    .gpioirq     = TOUCHPAD_IRQ,
    .gpiosuspend = TOUCHPAD_SUSPEND,
    .gpio_setup  = touchpad_gpio_setup,
    .gpio_shutdown = touchpad_gpio_release
};

#define KBD_RST 35
#define KBD_IRQ 36

static void kbd_gpio_release(void)
{
    gpio_free(KBD_IRQ);
    gpio_free(KBD_RST);
}

static int kbd_gpio_setup(void)
{
    int rc;
    int respin = KBD_RST;
    int irqpin = KBD_IRQ;
    unsigned rescfg =
        GPIO_CFG(respin, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA);
    unsigned irqcfg =
        GPIO_CFG(irqpin, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);

    rc = gpio_request(irqpin, "gpio_keybd_irq");
    if (rc) {
        pr_err("gpio_request failed on pin %d (rc=%d)\n",
            irqpin, rc);
        goto err_gpioconfig;
    }
    rc = gpio_request(respin, "gpio_keybd_reset");
    if (rc) {
        pr_err("gpio_request failed on pin %d (rc=%d)\n",
            respin, rc);
        goto err_gpioconfig;
    }
    rc = gpio_tlmm_config(rescfg, GPIO_ENABLE);
    if (rc) {
        pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
            respin, rc);
        goto err_gpioconfig;
    }
    rc = gpio_tlmm_config(irqcfg, GPIO_ENABLE);
    if (rc) {
        pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
            irqpin, rc);
        goto err_gpioconfig;
    }
    return rc;

err_gpioconfig:
    kbd_gpio_release();
    return rc;
}

/* use gpio output pin to toggle keyboard external reset pin */
static void kbd_hwreset(int kbd_mclrpin)
{
    gpio_direction_output(kbd_mclrpin, 0);
    gpio_direction_output(kbd_mclrpin, 1);
}

static struct msm_i2ckbd_platform_data msm_kybd_data = {
    .hwrepeat = 0,
    .scanset1 = 1,
    .gpioreset = KBD_RST,
    .gpioirq = KBD_IRQ,
    .gpio_setup = kbd_gpio_setup,
    .gpio_shutdown = kbd_gpio_release,
    .hw_reset = kbd_hwreset,
};

static struct i2c_board_info msm_i2c_board_info[] __initdata = {
    {
        I2C_BOARD_INFO("max8649", 0x60),
    },
    {
        I2C_BOARD_INFO("yas529", 0x2e),
    },
};

#ifdef CONFIG_TG03_CAMERA
static uint32_t camera_off_gpio_table[] = {
    /* parallel CAMERA interfaces */
 #ifdef CONFIG_CAM12MP
    GPIO_CFG(0,  0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* RESET */
    GPIO_CFG(1,  0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* STANDBY */
    GPIO_CFG(2,  0, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA), /* M_CAM_SCL */
    GPIO_CFG(3,  0, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA), /* M_CAM_SDA */
    GPIO_CFG(4,  0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
    GPIO_CFG(5,  0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
    GPIO_CFG(6,  0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
    GPIO_CFG(7,  0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
    GPIO_CFG(8,  0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
    GPIO_CFG(9,  0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
    GPIO_CFG(10, 0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
    GPIO_CFG(11, 0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
    GPIO_CFG(12, 0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
    GPIO_CFG(13, 0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
    GPIO_CFG(14, 0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
    GPIO_CFG(29, 0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* ISP CAM 18V */
    GPIO_CFG(151,0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* RAM Standby */
 #endif
};

static uint32_t camera_on_gpio_table[] = {
    /* parallel CAMERA interfaces */
 #ifdef CONFIG_CAM12MP
    GPIO_CFG(0,  0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* RESET */
    GPIO_CFG(1,  0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* STANDBY */
    GPIO_CFG(2,  0, GPIO_INPUT,  GPIO_PULL_UP,   GPIO_2MA), /* M_CAM_SCL */
    GPIO_CFG(3,  0, GPIO_INPUT,  GPIO_PULL_UP,   GPIO_2MA), /* M_CAM_SDA */
    GPIO_CFG(4,  1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
    GPIO_CFG(5,  1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
    GPIO_CFG(6,  1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
    GPIO_CFG(7,  1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
    GPIO_CFG(8,  1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
    GPIO_CFG(9,  1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
    GPIO_CFG(10, 1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
    GPIO_CFG(11, 1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
    GPIO_CFG(12, 0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
    GPIO_CFG(13, 1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
    GPIO_CFG(14, 1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
    GPIO_CFG(29, 0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* ISP CAM 18V */
    GPIO_CFG(151,0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* RAM Standby */
 #endif
};

static void config_gpio_table(uint32_t *table, int len)
{
    int n, rc;
    for (n = 0; n < len; n++) {
        rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
        if (rc) {
            printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
                __func__, table[n], rc);
            break;
        }
    }
}

static void config_camera_on_gpios(void)
{
    if (gpio_request( 2, "m_cam_scl" ))
        pr_err("gpio_request failed on SCL(2)\n");
    if (gpio_request( 3, "m_cam_sda" ))
        pr_err("gpio_request failed on SDA(3)\n");

    config_gpio_table(camera_on_gpio_table,
        ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
    gpio_export( 2, false );
    gpio_free( 2 );
    gpio_export( 3, false );
    gpio_free( 3 );

    config_gpio_table(camera_off_gpio_table,
        ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
    {
        .start  = 0xA0F00000,
        .end    = 0xA0F00000 + SZ_1M - 1,
        .flags  = IORESOURCE_MEM,
    },
    {
        .start  = INT_VFE,
        .end    = INT_VFE,
        .flags  = IORESOURCE_IRQ,
    },
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
    .camera_gpio_on  = config_camera_on_gpios,
    .camera_gpio_off = config_camera_off_gpios,
    .ioext.mdcphy = MSM_MDC_PHYS,
    .ioext.mdcsz  = MSM_MDC_SIZE,
    .ioext.appphy = MSM_CLK_CTL_PHYS,
    .ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_flash_src msm_flash_src = {
    .flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
    ._fsrc.pmic_src.low_current  = 30,
    ._fsrc.pmic_src.high_current = 100,
};

#ifdef CONFIG_CAM12MP
static struct msm_camera_sensor_flash_data flash_cam12mp = {
    .flash_type = MSM_CAMERA_FLASH_LED,
    .flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_cam12mp_data = {
    .sensor_name    = "cam12mp",
    .sensor_reset   = 0,
    .sensor_pwd     = 1,
    .vcm_pwd        = 0,
    .vcm_enable     = 0,
    .pdata          = &msm_camera_device_data,
    .resource       = msm_camera_resources,
    .num_resources  = ARRAY_SIZE(msm_camera_resources),
    .flash_data     = &flash_cam12mp
};

static struct platform_device msm_camera_sensor_cam12mp = {
    .name      = "msm_camera_cam12mp",
    .dev       = {
        .platform_data = &msm_camera_sensor_cam12mp_data,
    },
};
#endif
#endif

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
    .voltage_min_design     = 3200,
    .voltage_max_design = 4200,
    .avail_chg_sources      = AC_CHG | USB_CHG ,
    .batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
    .calculate_capacity = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
    u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
    u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

    return (current_voltage - low_voltage) * 100
        / (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
    .name           = "msm-battery",
    .id         = -1,
    .dev.platform_data  = &msm_psy_batt_data,
};

static int hsusb_rpc_connect(int connect)
{
    if (connect)
        return msm_hsusb_rpc_connect();
    else
        return msm_hsusb_rpc_close();
}

static struct msm_otg_platform_data msm_otg_pdata = {
    .rpc_connect    = hsusb_rpc_connect,
    .pmic_notif_init         = msm_pm_app_rpc_init,
    .pmic_notif_deinit       = msm_pm_app_rpc_deinit,
    .pmic_register_vbus_sn   = msm_pm_app_register_vbus_sn,
    .pmic_unregister_vbus_sn = msm_pm_app_unregister_vbus_sn,
    .pmic_enable_ldo         = msm_pm_app_enable_usb_ldo,
    .pemp_level              = PRE_EMPHASIS_WITH_10_PERCENT,
    .cdr_autoreset           = CDR_AUTO_RESET_DEFAULT,
    .drv_ampl                = HS_DRV_AMPLITUDE_5_PERCENT,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

static struct platform_device sub_pmic_dev = {
    .name = "sub_pmic",
};

static struct platform_device memread_dev = {
    .name = "memread_driver",
};

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_GPIOLIB
    &msm_gpio_devices[0],
    &msm_gpio_devices[1],
    &msm_gpio_devices[2],
    &msm_gpio_devices[3],
    &msm_gpio_devices[4],
    &msm_gpio_devices[5],
    &msm_gpio_devices[6],
    &msm_gpio_devices[7],
#endif
};

static struct platform_device msm_wlan_ar6000_pm_device = {
    .name       = "wlan_ar6000_pm_dev",
    .id     = 1,
    .num_resources  = 0,
    .resource   = NULL,
};

static struct platform_device *devices[] __initdata = {
    &msm_wlan_ar6000_pm_device,
    &msm_fb_device,
    &mddi_toshiba_device,
    &msm_device_smd,
    &msm_device_dmov,
    &android_pmem_kernel_ebi1_device,
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
    &android_pmem_kernel_smi_device,
#endif
    &android_pmem_device,
    &android_pmem_adsp_device,
    &android_pmem_smipool_device,
    &msm_device_nand,
    &msm_device_i2c,
    &qsd_device_spi,
#ifdef CONFIG_USB_FUNCTION
    &mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
    &mass_storage_device,
    &android_usb_device,
#endif
    &msm_device_tssc,
    &msm_audio_device,
    &msm_device_uart_dm1,
    &msm_device_uart_dm2,
    &msm_bluesleep_device,
#ifdef CONFIG_BT
    &msm_bt_power_device,
#endif
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
    &msm_device_uart3,
#endif
#ifdef CONFIG_LEDS_MSM_PMIC
    &msm_device_pmic_leds,
#endif
    &tg03_i2c_led,
    &msm_device_kgsl,
    &hs_device,
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
    &msm_device_tsif,
#endif
#ifdef CONFIG_TG03_CAMERA
 #ifdef CONFIG_CAM12MP
    &msm_camera_sensor_cam12mp,
 #endif
#endif
    &msm_batt_device,
    &tg03_cap_device,
    &sub_pmic_dev,
    &memread_dev,
};

static void __init qsd8x50_init_irq(void)
{
    msm_init_irq();
    msm_init_sirc();
}

static void kgsl_phys_memory_init(void)
{
    request_mem_region(kgsl_resources[1].start,
        resource_size(&kgsl_resources[1]), "kgsl");
}

static void usb_mpp_init(void)
{
    unsigned rc;
    unsigned mpp_usb = 20;

    if (machine_is_qsd8x50_ffa()) {
        rc = mpp_config_digital_out(mpp_usb,
            MPP_CFG(MPP_DLOGIC_LVL_VDD,
                MPP_DLOGIC_OUT_CTRL_HIGH));
        if (rc)
            pr_err("%s: configuring mpp pin"
                "to enable 3.3V LDO failed\n", __func__);
    }
}

extern int msm_usb_get_serial_number(void); /* USB_FROYO+ */
static void __init qsd8x50_init_usb(void)
{
#ifdef CONFIG_USB_MSM_OTG_72K
    platform_device_register(&msm_device_otg);
#endif

#ifdef CONFIG_USB_FUNCTION_MSM_HSUSB
    platform_device_register(&msm_device_hsusb_peripheral);
#endif

#ifdef CONFIG_USB_MSM_72K
    platform_device_register(&msm_device_gadget_peripheral);
#endif

    if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa())
        return;

    vreg_usb = vreg_get(NULL, "boost");

    if (IS_ERR(vreg_usb)) {
        printk(KERN_ERR "%s: vreg get failed (%ld)\n",
               __func__, PTR_ERR(vreg_usb));
        return;
    }

    platform_device_register(&msm_device_hsusb_otg);
    msm_add_host(0, &msm_usb_host_pdata);
#ifdef CONFIG_USB_FS_HOST
    if (fsusb_gpio_init())
        return;
    msm_add_host(1, &msm_usb_host2_pdata);
#endif
/* USB_FROYO+ s */
#ifdef CONFIG_USB_ANDROID
    if (!msm_usb_get_serial_number())
        printk(KERN_ERR "%s: serial number get failed\n", __func__);
#endif
/* USB_FROYO+ e */
}

static struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
    || defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
    || defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
    || defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
    struct msm_gpio *cfg_data;
    uint32_t size;
};

static struct msm_gpio sdc1_cfg_data[] = {
    {GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_3"},
    {GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_2"},
    {GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_1"},
    {GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_0"},
    {GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_cmd"},
    {GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
    {GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc2_clk"},
    {GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_cmd"},
    {GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_3"},
    {GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_2"},
    {GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_1"},
    {GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_0"},
};

static struct msm_gpio sdc3_cfg_data[] = {
    {GPIO_CFG(88, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc3_clk"},
    {GPIO_CFG(89, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_cmd"},
    {GPIO_CFG(90, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_3"},
    {GPIO_CFG(91, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_2"},
    {GPIO_CFG(92, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_1"},
    {GPIO_CFG(93, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
    {GPIO_CFG(158, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_4"},
    {GPIO_CFG(159, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_5"},
    {GPIO_CFG(160, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_6"},
    {GPIO_CFG(161, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_7"},
#endif
};

static struct msm_gpio sdc4_cfg_data[] = {
    {GPIO_CFG(142, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc4_clk"},
    {GPIO_CFG(143, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_cmd"},
    {GPIO_CFG(144, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_0"},
    {GPIO_CFG(145, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_1"},
    {GPIO_CFG(146, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_2"},
    {GPIO_CFG(147, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_3"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
    {
        .cfg_data = sdc1_cfg_data,
        .size = ARRAY_SIZE(sdc1_cfg_data),
    },
    {
        .cfg_data = sdc2_cfg_data,
        .size = ARRAY_SIZE(sdc2_cfg_data),
    },
    {
        .cfg_data = sdc3_cfg_data,
        .size = ARRAY_SIZE(sdc3_cfg_data),
    },
    {
        .cfg_data = sdc4_cfg_data,
        .size = ARRAY_SIZE(sdc4_cfg_data),
    },
};

static unsigned long vreg_sts, gpio_sts;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
    int rc = 0;
    struct sdcc_gpio *curr;

    curr = &sdcc_cfg_data[dev_id - 1];
    if (!(test_bit(dev_id, &gpio_sts)^enable))
        return;

    if (enable) {
        set_bit(dev_id, &gpio_sts);
        rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
        if (rc)
            printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
                __func__,  dev_id);
    } else {
        clear_bit(dev_id, &gpio_sts);
        msm_gpios_disable_free(curr->cfg_data, curr->size);
    }
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
    int rc = 0;
    struct platform_device *pdev;

    pdev = container_of(dv, struct platform_device, dev);
    msm_sdcc_setup_gpio(pdev->id, !!vdd);

    if (vdd == 0) {
        if (!vreg_sts)
            return 0;

        clear_bit(pdev->id, &vreg_sts);

        if (!vreg_sts) {
            rc = vreg_disable(vreg_mmc);
            if (rc)
                printk(KERN_ERR "%s: return val: %d \n",
                    __func__, rc);
        }
        return 0;
    }

    if (!vreg_sts) {
        rc = vreg_set_level(vreg_mmc, PMIC_VREG_GP6_LEVEL);
        if (!rc)
            rc = vreg_enable(vreg_mmc);
        if (rc)
            printk(KERN_ERR "%s: return val: %d \n",
                    __func__, rc);
    }
    set_bit(pdev->id, &vreg_sts);
    return 0;
}

#endif
#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
    || defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
    || defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static int msm_sdcc_get_wpswitch(struct device *dv)
{
    void __iomem *wp_addr = 0;
    uint32_t ret = 0;
    struct platform_device *pdev;

    if (!(machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf()))
        return -1;

    pdev = container_of(dv, struct platform_device, dev);

    wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
    if (!wp_addr) {
        pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
        return -ENOMEM;
    }

    ret = (readl(wp_addr) >> ((pdev->id - 1) << 1)) & (0x03);
    pr_info("%s: WP/CD Status for Slot %d = 0x%x \n", __func__,
                            pdev->id, ret);
    iounmap(wp_addr);
    return ((ret == 0x02) ? 1 : 0);

}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data qsd8x50_sdc1_data = {
    .ocr_mask   = MMC_VDD_27_28 | MMC_VDD_28_29,
    .translate_vdd  = msm_sdcc_setup_power,
    .mmc_bus_width  = MMC_CAP_4_BIT_DATA,
    .wpswitch   = msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
    .dummy52_required = 1,
#endif
    .msmsdcc_fmin   = 144000,
    .msmsdcc_fmid   = 25000000,
    .msmsdcc_fmax   = 49152000,
    .nonremovable   = 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data qsd8x50_sdc2_data = {
    .ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
    .translate_vdd  = NULL,
    .mmc_bus_width  = MMC_CAP_4_BIT_DATA,
    .wpswitch   = msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
    .dummy52_required = 1,
#endif
    .msmsdcc_fmin   = 144000,
    .msmsdcc_fmid   = 25000000,
    .msmsdcc_fmax   = 49152000,
    .nonremovable   = 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data qsd8x50_sdc3_data = {
    .ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
    .translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
    .mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
    .mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
    .dummy52_required = 1,
#endif
    .msmsdcc_fmin   = 144000,
    .msmsdcc_fmid   = 25000000,
    .msmsdcc_fmax   = 49152000,
    .nonremovable   = 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data qsd8x50_sdc4_data = {
    .ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
    .translate_vdd  = msm_sdcc_setup_power,
    .mmc_bus_width  = MMC_CAP_4_BIT_DATA,
    .wpswitch   = msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
    .dummy52_required = 1,
#endif
    .msmsdcc_fmin   = 144000,
    .msmsdcc_fmid   = 25000000,
    .msmsdcc_fmax   = 49152000,
    .nonremovable   = 0,
};
#endif

static void __init qsd8x50_init_mmc(void)
{
    if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa())
        vreg_mmc = vreg_get(NULL, "gp6");
    else
        vreg_mmc = vreg_get(NULL, "gp5");

    if (IS_ERR(vreg_mmc)) {
        printk(KERN_ERR "%s: vreg get failed (%ld)\n",
               __func__, PTR_ERR(vreg_mmc));
        return;
    }

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
    msm_add_sdcc(1, &qsd8x50_sdc1_data);
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
        msm_add_sdcc(2, &qsd8x50_sdc2_data);
#endif
/* WLAN_0012_S */
    if (machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf()) {
/* WLAN_0012_E */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
        msm_add_sdcc(3, &qsd8x50_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
        msm_add_sdcc(4, &qsd8x50_sdc4_data);
#endif
    }

}

static void __init qsd8x50_cfg_smc91x(void)
{
    int rc = 0;

    if (machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf()) {
        smc91x_resources[0].start = 0x70000300;
        smc91x_resources[0].end = 0x700003ff;
        smc91x_resources[1].start = MSM_GPIO_TO_INT(156);
        smc91x_resources[1].end = MSM_GPIO_TO_INT(156);
    } else if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
        smc91x_resources[0].start = 0x84000300;
        smc91x_resources[0].end = 0x840003ff;
        smc91x_resources[1].start = MSM_GPIO_TO_INT(87);
        smc91x_resources[1].end = MSM_GPIO_TO_INT(87);

        rc = gpio_tlmm_config(GPIO_CFG(87, 0, GPIO_INPUT,
                           GPIO_PULL_DOWN, GPIO_2MA),
                           GPIO_ENABLE);
        if (rc) {
            printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",
                    __func__, rc);
        }
    } else
        printk(KERN_ERR "%s: invalid machine type\n", __func__);
}

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
    [MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

    [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
    [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
        = 1,
    [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
    [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
    [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

    [MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
    [MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
    [MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
    [MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
    [MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
    int gpio_scl;
    int gpio_sda;
    if (iface) {
    } else {
        gpio_scl = 95;
        gpio_sda = 96;
    }
    if (config_type) {
        gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
                    GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
        gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
                    GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
    } else {
        gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
                    GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
        gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
                    GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
    }
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
    .clk_freq = 384000,
    .rsl_id = SMEM_SPINLOCK_I2C,
    .pri_clk = 95,
    .pri_dat = 96,
    .msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
    if (gpio_request(95, "i2c_pri_clk"))
        pr_err("failed to request gpio i2c_pri_clk\n");
    if (gpio_request(96, "i2c_pri_dat"))
        pr_err("failed to request gpio i2c_pri_dat\n");

    msm_i2c_pdata.rmutex = (uint32_t)smem_alloc(SMEM_I2C_MUTEX, 8);
    msm_i2c_pdata.pm_lat =
        msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
        .latency;
    msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
    pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static unsigned pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
static void __init pmem_kernel_smi_size_setup(char **p)
{
    pmem_kernel_smi_size = memparse(*p, p);

    /* Make sure that we don't allow more SMI memory then is
       available - the kernel mapping code has no way of knowing
       if it has gone over the edge */

    if (pmem_kernel_smi_size > MSM_PMEM_SMIPOOL_SIZE)
        pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
}
__early_param("pmem_kernel_smi_size=", pmem_kernel_smi_size_setup);
#endif

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
    pmem_sf_size = memparse(*p, p);
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
    pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);


static unsigned audio_size = MSM_AUDIO_SIZE;
static void __init audio_size_setup(char **p)
{
    audio_size = memparse(*p, p);
}
__early_param("audio_size=", audio_size_setup);

static void __init qsd8x50_init(void)
{
    if (socinfo_init() < 0)
        printk(KERN_ERR "%s: socinfo_init() failed!\n",
               __func__);
    msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
    platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
    msm_acpu_clock_init(&qsd8x50_clock_data);

    msm_hsusb_pdata.swfi_latency =
        msm_pm_data
        [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
    msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;

    msm_gadget_pdata.swfi_latency =
        msm_pm_data
        [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
    msm_device_otg.dev.platform_data = &msm_otg_pdata;
    msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
    msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif

    dev_model_init_model_table();

    platform_add_devices(devices, ARRAY_SIZE(devices));
    msm_fb_add_devices();
#ifdef CONFIG_MSM_CAMERA
    config_camera_off_gpios(); /* might not be necessary */
#endif
    qsd8x50_init_usb();
    qsd8x50_init_mmc();
    bt_power_init();
    audio_gpio_init();
    msm_device_i2c_init();
    msm_qsd_spi_init();
    i2c_register_board_info(0, msm_i2c_board_info,
                ARRAY_SIZE(msm_i2c_board_info));
    spi_register_board_info(msm_spi_board_info,
                ARRAY_SIZE(msm_spi_board_info));
    msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
    kgsl_phys_memory_init();
    tg03_captouch_init();
#ifdef CONFIG_SURF_FFA_GPIO_KEYPAD
    if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa())
        platform_device_register(&keypad_device_8k_ffa);
    else
        platform_device_register(&keypad_device_surf);
#endif

#ifdef CONFIG_GPIO_KEYPAD
    platform_device_register(&keypad_device);
#endif
    /* Vibrator Intialization */
    msm_init_pmic_vibrator();
}

static void __init qsd8x50_allocate_memory_regions(void)
{
    void *addr;
    unsigned long size;

    size = pmem_kernel_ebi1_size;
    if (size) {
        addr = alloc_bootmem_aligned(size, 0x100000);
        android_pmem_kernel_ebi1_pdata.start = __pa(addr);
        android_pmem_kernel_ebi1_pdata.size = size;
        pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
            " ebi1 pmem arena\n", size, addr, __pa(addr));
    }

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
    size = pmem_kernel_smi_size;
    if (size > MSM_PMEM_SMIPOOL_SIZE) {
        printk(KERN_ERR "pmem kernel smi arena size %lu is too big\n",
            size);

        size = MSM_PMEM_SMIPOOL_SIZE;
    }

    android_pmem_kernel_smi_pdata.start = MSM_PMEM_SMIPOOL_BASE;
    android_pmem_kernel_smi_pdata.size = size;

    pr_info("allocating %lu bytes at %lx (%lx physical)"
        "for pmem kernel smi arena\n", size,
        (long unsigned int) MSM_PMEM_SMIPOOL_BASE,
        __pa(MSM_PMEM_SMIPOOL_BASE));
#endif

    size = pmem_sf_size;
    if (size) {
        addr = alloc_bootmem(size);
        android_pmem_pdata.start = __pa(addr);
        android_pmem_pdata.size = size;
        pr_info("allocating %lu bytes at %p (%lx physical) for sf "
            "pmem arena\n", size, addr, __pa(addr));
    }

    size = pmem_adsp_size;
    if (size) {
        addr = alloc_bootmem(size);
        android_pmem_adsp_pdata.start = __pa(addr);
        android_pmem_adsp_pdata.size = size;
        pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
            "pmem arena\n", size, addr, __pa(addr));
    }


    size = MSM_FB_SIZE;
    addr = (void *)MSM_FB_BASE;
    msm_fb_resources[0].start = (unsigned long)addr;
    msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
    pr_info("using %lu bytes of SMI at %lx physical for fb\n",
           size, (unsigned long)addr);

    size = audio_size ? : MSM_AUDIO_SIZE;
    addr = alloc_bootmem(size);
    msm_audio_resources[0].start = __pa(addr);
    msm_audio_resources[0].end = msm_audio_resources[0].start + size - 1;
    pr_info("allocating %lu bytes at %p (%lx physical) for audio\n",
        size, addr, __pa(addr));
}

int msm_prox_read_nvitem(unsigned int id)
{
    int rc;
    unsigned int prox_read_data;

    /* read NV*/
    rc = msm_proc_comm(PCOM_NV_READ, &id, &prox_read_data);
    if(rc)
    {
        printk("PROX NV read error %d\n",rc);
        return -1;
    }
    else
    {
        return prox_read_data;
    }
}

static void __init qsd8x50_map_io(void)
{
    msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
    msm_map_qsd8x50_io();
    qsd8x50_allocate_memory_regions();
}

MACHINE_START(QSD8X50_TG03, "QSD8X50 TG03")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .map_io = qsd8x50_map_io,
    .init_irq = qsd8x50_init_irq,
    .init_machine = qsd8x50_init,
    .timer = &msm_timer,
MACHINE_END

/* WLAN_0012_S */
int Wlan_vreg_enable(void)
{
  struct vreg *vreg_wlan;
  int rc;
  printk(KERN_ERR "%s: Entering vreg_wlan_refcount (%d)\n", __func__, vreg_wlan_refcount);

  if(vreg_wlan_refcount==0)
  {
    vreg_wlan = vreg_get(NULL, "wlan");

    if (IS_ERR(vreg_wlan))
    {
      printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__, PTR_ERR(vreg_wlan));
      return PTR_ERR(vreg_wlan);
    }

    rc = vreg_set_level(vreg_wlan, PMIC_VREG_WLAN_LEVEL);
    if (rc)
    {
      printk(KERN_ERR "%s: vreg wlan set level failed (%d)\n", __func__, rc);
      return -EIO;
    }
    rc = vreg_enable(vreg_wlan);

    if (rc)
    {
      printk(KERN_ERR "%s: vreg wlan enable failed (%d)\n", __func__, rc);
      return -EIO;
    }
  }
  else
  {
    printk(KERN_ERR "%s: vreg_wlan_refcount not Enabling (%d)\n", __func__, vreg_wlan_refcount);
  }
  printk(KERN_ERR "%s: vreg_wlan_refcount (%d)\n", __func__, vreg_wlan_refcount);

  if(vreg_wlan_refcount<2)
  {
    printk(KERN_ERR "%s: Incrementing vreg_wlan_refcount < 2 (%d)\n", __func__, vreg_wlan_refcount);
    vreg_wlan_refcount++;
    return 1;
  }

  return 1;
}
EXPORT_SYMBOL(Wlan_vreg_enable);

int Wlan_vreg_disable(void)
{

  struct vreg *vreg_wlan;
  int rc;
  printk(KERN_ERR "%s: Entering vreg_wlan_refcount (%d)\n", __func__, vreg_wlan_refcount);

  if(vreg_wlan_refcount==1)
  {
    vreg_wlan = vreg_get(NULL, "wlan");
    if (IS_ERR(vreg_wlan))
    {
      printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__, PTR_ERR(vreg_wlan));
      return PTR_ERR(vreg_wlan);
    }

    rc = vreg_disable(vreg_wlan);
    if (rc) {
      printk(KERN_ERR "%s: vreg wlan disable failed (%d)\n",
          __func__, rc);
      return -EIO;
    }
  }
  else
  {
    printk(KERN_ERR "%s: vreg_wlan_refcount not Disabling (%d)\n", __func__, vreg_wlan_refcount);
  }

  printk(KERN_ERR "%s: vreg_wlan_refcount (%d)\n", __func__, vreg_wlan_refcount);

  if(vreg_wlan_refcount >=1 )
  {
    printk(KERN_ERR "%s: Decrementing vreg_wlan_refcount > = (%d)\n", __func__, vreg_wlan_refcount);
    vreg_wlan_refcount--;
    return 1;
  }

  return 1;
}
EXPORT_SYMBOL(Wlan_vreg_disable);
/* WLAN_0012_E */
