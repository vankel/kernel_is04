/*
  subpmic Driver

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

#ifndef SUBPM_TG03_EXT_H
#define SUBPM_TG03_EXT_H

/***************************
  Error define
****************************/
#define SUBPM_SUCCESS        0
#define SUBPM_ERROR         -1
#define SUBPM_I2C_FAIL      -2
#define SUBPM_INVALID       -3


/***************************
  LDO control
****************************/
/* LDO type */
typedef enum {
    SUBPM_LDO_NONE=0,
    SUBPM_LDO_1,        /* Felica29V */
    SUBPM_LDO_2,        /* CAM28V */
    SUBPM_LDO_3,        /* CAM12V */
    SUBPM_LDO_4,        /* AP26V */
    SUBPM_LDO_5,        /* DTV28V */
    SUBPM_LDO_6,        /* LCD29V */
    SUBPM_LDO_7,        /* DTV12V */
    SUBPM_LDO_8,        /* ISP26V */
    SUBPM_LDO_MAX
}subpm_ldo_type;

#define SUBPM_LDO_FELICA29V     SUBPM_LDO_1
#define SUBPM_LDO_CAM28V        SUBPM_LDO_2
#define SUBPM_LDO_CAM12V        SUBPM_LDO_3
#define SUBPM_LDO_AP26V         SUBPM_LDO_4
#define SUBPM_LDO_DTV28V        SUBPM_LDO_5
#define SUBPM_LDO_LCD29V        SUBPM_LDO_6
#define SUBPM_LDO_DTV12V        SUBPM_LDO_7
#define SUBPM_LDO_ISP26V        SUBPM_LDO_8

/***************************
  REG control
****************************/
/* REG type */
typedef enum {
    SUBPM_REG_NONE=0,
    SUBPM_REG_1,
    SUBPM_REG_2,        /* ISP12V */
    SUBPM_REG_MAX
}subpm_reg_type;

#define SUBPM_REG_ISP12V        SUBPM_REG_2

/***************************
  UART bus control
****************************/
/* UART type */
typedef enum {
    SUBPM_UART_SW_NONE=0,
    SUBPM_UART_SW_1,        /* Felica */
    SUBPM_UART_SW_2,
    SUBPM_UART_SW_3,
    SUBPM_UART_SW_4,        /* Irda */
    SUBPM_UART_SW_5,        /* LOG */
    SUBPM_UART_SW_MAX
}subpm_uart_type;


/* UART IO CONTROL */
typedef struct
{
    unsigned char output; /* OUTPUT ENABLE/DISABLE */
    unsigned char input;  /* INPUT ENABLE/DISABLE  */
    unsigned char reserved[2];
}subpm_uart_ctrl_s;

#define SUBPM_UART_SW_IRDA          SUBPM_UART_SW_4
#define SUBPM_UART_SW_LOG           SUBPM_UART_SW_5

/***************************
  GPIO control
****************************/
/* GPIO type */
typedef enum {
    SUBPM_GPIO_NONE=0,
    SUBPM_GPIO_GPKI0,    /* =SUBPM_GPIO_SENDKEY */
    SUBPM_GPIO_GPKI1,    /* =SUBPM_GPIO_BACKKEY */
    SUBPM_GPIO_GPKI2,    /* =SUBPM_GPIO_HOMEKEY */
    SUBPM_GPIO_GPKI3,    /* =SUBPM_GPIO_MENUKEY */
    SUBPM_GPIO_GPKI4,    /* =SUBPM_GPIO_ENDKEY */
    SUBPM_GPIO_GPKI5,
    SUBPM_GPIO_GPKI6,    /* =SUBPM_GPIO_SDDET */
    SUBPM_GPIO_GPKI7,
    SUBPM_GPIO_GPKO0,    /* =SUBPM_GPIO_USB_ID */
    SUBPM_GPIO_GPKO1,    /* =SUBPM_GPIO_REMOCON */
    SUBPM_GPIO_GPKO2,
    SUBPM_GPIO_GPKO3,
    SUBPM_GPIO_GPKO4,
    SUBPM_GPIO_GPKO5,
    SUBPM_GPIO_GPKO6,
    SUBPM_GPIO_GPKO7,
    SUBPM_GPIO_GPKO8,
    SUBPM_GPIO_GPKO9,
    SUBPM_GPIO_GPKO10,
    SUBPM_GPIO_GPKO11,
    SUBPM_GPIO_MAX
}subpm_gpio_type;

#define SUBPM_GPIO_SENDKEY      SUBPM_GPIO_GPKI0
#define SUBPM_GPIO_BACKKEY      SUBPM_GPIO_GPKI1
#define SUBPM_GPIO_HOMEKEY      SUBPM_GPIO_GPKI2
#define SUBPM_GPIO_MENUKEY      SUBPM_GPIO_GPKI3
#define SUBPM_GPIO_ENDKEY       SUBPM_GPIO_GPKI4
#define SUBPM_GPIO_IRDA         SUBPM_GPIO_GPKI5
#define SUBPM_GPIO_SDDET        SUBPM_GPIO_GPKI6
#define SUBPM_GPIO_DTV_RST      SUBPM_GPIO_GPKO6

#define SUBPM_GPIO_USB_ID       SUBPM_GPIO_GPKO0
#define SUBPM_GPIO_REMOCON      SUBPM_GPIO_GPKO1

/***************************
  Control flags
****************************/
#define SUBPM_CTRL_ON   1
#define SUBPM_CTRL_OFF  0

#define SUBPM_GPIO_HI   1
#define SUBPM_GPIO_LO   0


#endif  /* SUBPM_TG03_EXT_H */
