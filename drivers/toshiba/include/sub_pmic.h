
/*
  sub_pmic.h

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

#ifndef __ARCH_ARM_MACH_SUB_PMIC_H
#define __ARCH_ARM_MACH_SUB_PMIC_H


/*===========================================================================
    DEFINE
============================================================================*/
/* ERROR CODE */
#define SUB_PMIC_SUCCESS (0)
#define SUB_PMIC_ERROR   (-1)

#define FALSE false
#define TRUE true

/* LDO */
enum subpm_ldo_status{
    SUB_PMIC_LDO_OFF = 0,             /* LDO* OFF */
    SUB_PMIC_LDO_ON = 1,             /* LDO* ON  */
};

enum subpm_ldo_no {
    SUB_PMIC_LDO1 = 0x01,
    SUB_PMIC_LDO2 = 0x02,
    SUB_PMIC_LDO3 = 0x03,
    SUB_PMIC_LDO4 = 0x04,
    SUB_PMIC_LDO5 = 0x05,
    SUB_PMIC_LDO6 = 0x06,
    SUB_PMIC_LDO7 = 0x07,
    SUB_PMIC_LDO8 = 0x08,
    SUB_PMIC_MAX_LDO   = SUB_PMIC_LDO8,    /* Number of LDO selectable */
    SUB_PMIC_FELICA29V = SUB_PMIC_LDO1,    /* FELICA29V */
    SUB_PMIC_CAM28V    = SUB_PMIC_LDO2,    /* CAM28V */
    SUB_PMIC_CAM12V    = SUB_PMIC_LDO3,    /* CAM12V */
    SUB_PMIC_AP26V     = SUB_PMIC_LDO4,    /* AP26V */
    SUB_PMIC_DTV28V    = SUB_PMIC_LDO5,    /* DTV28V */
    SUB_PMIC_LCD29V    = SUB_PMIC_LDO6,    /* LCD29V */
    SUB_PMIC_DTV12V    = SUB_PMIC_LDO7,    /* DTV12V */
    SUB_PMIC_ISP26V    = SUB_PMIC_LDO8,    /* ISP26V */
};


/* REG */
enum subpm_reg_status{
    SUB_PMIC_REG_OFF = 0,             /* REG* OFF */
    SUB_PMIC_REG_ON = 1,             /* REG* ON  */
};

enum subpm_reg_no{
    SUB_PMIC_REG1 = 0x01,
    SUB_PMIC_REG2 = 0x02,
    SUB_PMIC_MAX_REG = SUB_PMIC_REG2,   /* Number of REG selectable */
    SUB_PMIC_ISP12V  = SUB_PMIC_REG2,   /* ISP12V */
};


/* UART */
enum subpm_uart_kind{
    SUB_PMIC_LOG = 0,
    SUB_PMIC_IRDA = 1,
    SUB_PMIC_MAX_UART = 2,  /* Number of UART-SWITCH selectable */
};


enum subpm_uart_status{
    SUB_PMIC_UART_DISABLE = 0x00,      /* UART DISABLE */
    SUB_PMIC_UART_ENABLE = 0x01,       /* UART ENABLE */
};


/* IrDA */
enum subpm_irda_pwrctrl{
    SUB_PMIC_IRDA_OFF = 0,    /* IrDA OFF */
    SUB_PMIC_IRDA_ON = 1,     /* IrDA ON */
};


/* DTV */
enum subpm_dtv_reset{
    SUB_PMIC_DTV_RESET = 0,    /* DTV RESET */
    SUB_PMIC_DTV_OPEN = 1,     /* DTV OPEN */
};


/* GPIO(OUTPUT) */
enum subpm_gpio_no{
    SUB_PMIC_GPKI00 = 1,
    SUB_PMIC_GPKI01 = 2,
    SUB_PMIC_GPKI02 = 3,
    SUB_PMIC_GPKI03 = 4,
    SUB_PMIC_GPKI04 = 5,
    SUB_PMIC_GPKI05 = 6,
    SUB_PMIC_GPKI06 = 7,
    SUB_PMIC_GPKI07 = 8,
    SUB_PMIC_GPKO00 = 9,
    SUB_PMIC_GPKO01 = 10,
    SUB_PMIC_GPKO02 = 11,
    SUB_PMIC_GPKO03 = 12,
    SUB_PMIC_GPKO04 = 13,
    SUB_PMIC_GPKO05 = 14,
    SUB_PMIC_GPKO06 = 15,
    SUB_PMIC_GPKO07 = 16,
    SUB_PMIC_GPKO08 = 17,
    SUB_PMIC_GPKO09 = 18,
    SUB_PMIC_GPKO10 = 19,
    SUB_PMIC_GPKO11 = 20,
    SUB_PMIC_MAX_GPIO = 20,    /* Number of GPKO selectable */
};

enum subpm_gpio_status{
    SUB_PMIC_GPIO_LO = 0,    /* Low */
    SUB_PMIC_GPIO_HI = 1,    /* High */
};

/*===========================================================================
    EXTERN FUNCTION
============================================================================*/
extern int sub_pmic_get_sd_detect(bool *sd_status);
extern int sub_pmic_ldo_ctrl(enum subpm_ldo_no ldo_no, enum subpm_ldo_status state);
extern int sub_pmic_reg_ctrl(enum subpm_reg_no reg_no, enum subpm_reg_status state);
extern int sub_pmic_uart_select(enum subpm_uart_kind kind, enum subpm_uart_status in, enum subpm_uart_status out);
extern int sub_pmic_irda_pwrctrl(enum subpm_irda_pwrctrl state);
extern int sub_pmic_dtv_reset(enum subpm_dtv_reset state);

extern int sub_pmic_get_ldo_status(enum subpm_ldo_no ldo_no, enum subpm_ldo_status *status);
extern int sub_pmic_get_reg_status(enum subpm_reg_no reg_no, enum subpm_reg_status *status);
extern int sub_pmic_get_uart_status(enum subpm_uart_kind kind, enum subpm_uart_status *in, enum subpm_uart_status *out);
extern int sub_pmic_get_gpio_status(enum subpm_gpio_no gpio_no, enum subpm_gpio_status *status);


#endif
