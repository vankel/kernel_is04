/*
 * DVB USB Linux driver for Afatech AF9015 DVB-T USB2.0 receiver
 *
 * Copyright (C) 2007 Antti Palosaari <crope@iki.fi>
 *
 * Thanks to Afatech who kindly provided information.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _DVB_USB_AF9015_H_
#define _DVB_USB_AF9015_H_

#define DVB_USB_LOG_PREFIX "af9015"
#include "dvb-usb.h"

#define deb_info(args...) dprintk(dvb_usb_af9015_debug, 0x01, args)
#define deb_rc(args...)   dprintk(dvb_usb_af9015_debug, 0x02, args)
#define deb_xfer(args...) dprintk(dvb_usb_af9015_debug, 0x04, args)
#define deb_reg(args...)  dprintk(dvb_usb_af9015_debug, 0x08, args)
#define deb_i2c(args...)  dprintk(dvb_usb_af9015_debug, 0x10, args)
#define deb_fw(args...)   dprintk(dvb_usb_af9015_debug, 0x20, args)

#define AF9015_I2C_EEPROM  0xa0
#define AF9015_I2C_DEMOD   0x38
#define AF9015_USB_TIMEOUT 2000

/* EEPROM locations */
#define AF9015_EEPROM_IR_MODE        0x18
#define AF9015_EEPROM_IR_REMOTE_TYPE 0x34
#define AF9015_EEPROM_TS_MODE        0x31
#define AF9015_EEPROM_DEMOD2_I2C     0x32

#define AF9015_EEPROM_SAW_BW1        0x35
#define AF9015_EEPROM_XTAL_TYPE1     0x36
#define AF9015_EEPROM_SPEC_INV1      0x37
#define AF9015_EEPROM_IF1L           0x38
#define AF9015_EEPROM_IF1H           0x39
#define AF9015_EEPROM_MT2060_IF1L    0x3a
#define AF9015_EEPROM_MT2060_IF1H    0x3b
#define AF9015_EEPROM_TUNER_ID1      0x3c

#define AF9015_EEPROM_SAW_BW2        0x45
#define AF9015_EEPROM_XTAL_TYPE2     0x46
#define AF9015_EEPROM_SPEC_INV2      0x47
#define AF9015_EEPROM_IF2L           0x48
#define AF9015_EEPROM_IF2H           0x49
#define AF9015_EEPROM_MT2060_IF2L    0x4a
#define AF9015_EEPROM_MT2060_IF2H    0x4b
#define AF9015_EEPROM_TUNER_ID2      0x4c

#define AF9015_EEPROM_OFFSET (AF9015_EEPROM_SAW_BW2 - AF9015_EEPROM_SAW_BW1)

#define AF9015_GPIO_ON (1 << 0)
#define AF9015_GPIO_EN (1 << 1)
#define AF9015_GPIO_O  (1 << 2)
#define AF9015_GPIO_I  (1 << 3)

#define AF9015_GPIO_TUNER_ON  (AF9015_GPIO_ON|AF9015_GPIO_EN)
#define AF9015_GPIO_TUNER_OFF (AF9015_GPIO_ON|AF9015_GPIO_EN|AF9015_GPIO_O)

struct req_t {
	u8  cmd;       /* [0] */
	/*  seq */     /* [1] */
	u8  i2c_addr;  /* [2] */
	u16 addr;      /* [3|4] */
	u8  mbox;      /* [5] */
	u8  addr_len;  /* [6] */
	u8  data_len;  /* [7] */
	u8  *data;
};

enum af9015_cmd {
	GET_CONFIG           = 0x10,
	DOWNLOAD_FIRMWARE    = 0x11,
	BOOT                 = 0x13,
	READ_MEMORY          = 0x20,
	WRITE_MEMORY         = 0x21,
	READ_WRITE_I2C       = 0x22,
	COPY_FIRMWARE        = 0x23,
	RECONNECT_USB        = 0x5a,
	WRITE_VIRTUAL_MEMORY = 0x26,
	GET_IR_CODE          = 0x27,
	READ_I2C,
	WRITE_I2C,
};

enum af9015_ir_mode {
	AF9015_IR_MODE_DISABLED = 0,
	AF9015_IR_MODE_HID,
	AF9015_IR_MODE_RLC,
	AF9015_IR_MODE_RC6,
};

struct af9015_state {
	struct i2c_adapter i2c_adap; /* I2C adapter for 2nd FE */
};

struct af9015_config {
	u8  dual_mode:1;
	u16 mt2060_if1[2];
	u16 firmware_size;
	u16 firmware_checksum;
	u8  *ir_table;
	u16 ir_table_size;
};

enum af9015_remote {
	AF9015_REMOTE_NONE                    = 0,
	AF9015_REMOTE_A_LINK_DTU_M,
	AF9015_REMOTE_MSI_DIGIVOX_MINI_II_V3,
	AF9015_REMOTE_MYGICTV_U718,
	AF9015_REMOTE_DIGITTRADE_DVB_T,
};

/* Leadtek WinFast DTV Dongle Gold */
static struct dvb_usb_rc_key af9015_rc_keys_leadtek[] = {
	{ 0x00, 0x1e, KEY_1 },
	{ 0x00, 0x1f, KEY_2 },
	{ 0x00, 0x20, KEY_3 },
	{ 0x00, 0x21, KEY_4 },
	{ 0x00, 0x22, KEY_5 },
	{ 0x00, 0x23, KEY_6 },
	{ 0x00, 0x24, KEY_7 },
	{ 0x00, 0x25, KEY_8 },
	{ 0x00, 0x26, KEY_9 },
	{ 0x00, 0x27, KEY_0 },
	{ 0x00, 0x28, KEY_ENTER },
	{ 0x00, 0x4f, KEY_VOLUMEUP },
	{ 0x00, 0x50, KEY_VOLUMEDOWN },
	{ 0x00, 0x51, KEY_CHANNELDOWN },
	{ 0x00, 0x52, KEY_CHANNELUP },
};

static u8 af9015_ir_table_leadtek[] = {
	0x03, 0xfc, 0x00, 0xff, 0x1a, 0x01, 0x00,
	0x03, 0xfc, 0x56, 0xa9, 0x00, 0x00, 0x00,
	0x03, 0xfc, 0x4b, 0xb4, 0x00, 0x00, 0x00,
	0x03, 0xfc, 0x4c, 0xb3, 0xb2, 0x04, 0x00,
	0x03, 0xfc, 0x4d, 0xb2, 0x00, 0x00, 0x00,
	0x03, 0xfc, 0x4e, 0xb1, 0x00, 0x00, 0x00,
	0x03, 0xfc, 0x1f, 0xe0, 0x3d, 0x00, 0x00,
	0x03, 0xfc, 0x40, 0xbf, 0x13, 0x01, 0x00,
	0x03, 0xfc, 0x14, 0xeb, 0x10, 0x00, 0x00,
	0x03, 0xfc, 0x49, 0xb6, 0x05, 0x01, 0x00,
	0x03, 0xfc, 0x50, 0xaf, 0x29, 0x00, 0x00,
	0x03, 0xfc, 0x0c, 0xf3, 0x52, 0x00, 0x00,
	0x03, 0xfc, 0x03, 0xfc, 0x09, 0x00, 0x00,
	0x03, 0xfc, 0x08, 0xf7, 0x50, 0x00, 0x00,
	0x03, 0xfc, 0x13, 0xec, 0x28, 0x00, 0x00,
	0x03, 0xfc, 0x04, 0xfb, 0x4f, 0x00, 0x00,
	0x03, 0xfc, 0x4f, 0xb0, 0x0f, 0x01, 0x00,
	0x03, 0xfc, 0x10, 0xef, 0x51, 0x00, 0x00,
	0x03, 0xfc, 0x51, 0xae, 0x3f, 0x00, 0x00,
	0x03, 0xfc, 0x42, 0xbd, 0x13, 0x00, 0x00,
	0x03, 0xfc, 0x43, 0xbc, 0x00, 0x00, 0x00,
	0x03, 0xfc, 0x44, 0xbb, 0x11, 0x00, 0x00,
	0x03, 0xfc, 0x52, 0xad, 0x19, 0x00, 0x00,
	0x03, 0xfc, 0x54, 0xab, 0x05, 0x00, 0x00,
	0x03, 0xfc, 0x46, 0xb9, 0x29, 0x00, 0x00,
	0x03, 0xfc, 0x55, 0xaa, 0x2b, 0x00, 0x00,
	0x03, 0xfc, 0x53, 0xac, 0x41, 0x00, 0x00,
	0x03, 0xfc, 0x05, 0xfa, 0x1e, 0x00, 0x00,
	0x03, 0xfc, 0x06, 0xf9, 0x1f, 0x00, 0x00,
	0x03, 0xfc, 0x07, 0xf8, 0x20, 0x00, 0x00,
	0x03, 0xfc, 0x1e, 0xe1, 0x19, 0x00, 0x00,
	0x03, 0xfc, 0x09, 0xf6, 0x21, 0x00, 0x00,
	0x03, 0xfc, 0x0a, 0xf5, 0x22, 0x00, 0x00,
	0x03, 0xfc, 0x0b, 0xf4, 0x23, 0x00, 0x00,
	0x03, 0xfc, 0x1b, 0xe4, 0x16, 0x00, 0x00,
	0x03, 0xfc, 0x0d, 0xf2, 0x24, 0x00, 0x00,
	0x03, 0xfc, 0x0e, 0xf1, 0x25, 0x00, 0x00,
	0x03, 0xfc, 0x0f, 0xf0, 0x26, 0x00, 0x00,
	0x03, 0xfc, 0x16, 0xe9, 0x28, 0x00, 0x00,
	0x03, 0xfc, 0x41, 0xbe, 0x37, 0x00, 0x00,
	0x03, 0xfc, 0x12, 0xed, 0x27, 0x00, 0x00,
	0x03, 0xfc, 0x11, 0xee, 0x2a, 0x00, 0x00,
	0x03, 0xfc, 0x48, 0xb7, 0x2c, 0x00, 0x00,
	0x03, 0xfc, 0x4a, 0xb5, 0x3c, 0x00, 0x00,
	0x03, 0xfc, 0x47, 0xb8, 0x15, 0x01, 0x00,
	0x03, 0xfc, 0x45, 0xba, 0x0b, 0x01, 0x00,
	0x03, 0xfc, 0x5e, 0xa1, 0x43, 0x00, 0x00,
	0x03, 0xfc, 0x5a, 0xa5, 0x42, 0x00, 0x00,
	0x03, 0xfc, 0x5b, 0xa4, 0x4b, 0x00, 0x00,
	0x03, 0xfc, 0x5f, 0xa0, 0x4e, 0x00, 0x00,
};

/* TwinHan AzureWave AD-TU700(704J) */
static struct dvb_usb_rc_key af9015_rc_keys_twinhan[] = {
	{ 0x05, 0x3f, KEY_POWER },
	{ 0x00, 0x19, KEY_FAVORITES },    /* Favorite List */
	{ 0x00, 0x04, KEY_TEXT },         /* Teletext */
	{ 0x00, 0x0e, KEY_POWER },
	{ 0x00, 0x0e, KEY_INFO },         /* Preview */
	{ 0x00, 0x08, KEY_EPG },          /* Info/EPG */
	{ 0x00, 0x0f, KEY_LIST },         /* Record List */
	{ 0x00, 0x1e, KEY_1 },
	{ 0x00, 0x1f, KEY_2 },
	{ 0x00, 0x20, KEY_3 },
	{ 0x00, 0x21, KEY_4 },
	{ 0x00, 0x22, KEY_5 },
	{ 0x00, 0x23, KEY_6 },
	{ 0x00, 0x24, KEY_7 },
	{ 0x00, 0x25, KEY_8 },
	{ 0x00, 0x26, KEY_9 },
	{ 0x00, 0x27, KEY_0 },
	{ 0x00, 0x29, KEY_CANCEL },       /* Cancel */
	{ 0x00, 0x4c, KEY_CLEAR },        /* Clear */
	{ 0x00, 0x2a, KEY_BACK },         /* Back */
	{ 0x00, 0x2b, KEY_TAB },          /* Tab */
	{ 0x00, 0x52, KEY_UP },           /* up arrow */
	{ 0x00, 0x51, KEY_DOWN },         /* down arrow */
	{ 0x00, 0x4f, KEY_RIGHT },        /* right arrow */
	{ 0x00, 0x50, KEY_LEFT },         /* left arrow */
	{ 0x00, 0x28, KEY_ENTER },        /* Enter / ok */
	{ 0x02, 0x52, KEY_VOLUMEUP },
	{ 0x02, 0x51, KEY_VOLUMEDOWN },
	{ 0x00, 0x4e, KEY_CHANNELDOWN },
	{ 0x00, 0x4b, KEY_CHANNELUP },
	{ 0x00, 0x4a, KEY_RECORD },
	{ 0x01, 0x11, KEY_PLAY },
	{ 0x00, 0x17, KEY_PAUSE },
	{ 0x00, 0x0c, KEY_REWIND },       /* FR << */
	{ 0x00, 0x11, KEY_FASTFORWARD },  /* FF >> */
	{ 0x01, 0x15, KEY_PREVIOUS },     /* Replay */
	{ 0x01, 0x0e, KEY_NEXT },         /* Skip */
	{ 0x00, 0x13, KEY_CAMERA },       /* Capture */
	{ 0x01, 0x0f, KEY_LANGUAGE },     /* SAP */
	{ 0x01, 0x13, KEY_TV2 },          /* PIP */
	{ 0x00, 0x1d, KEY_ZOOM },         /* Full Screen */
	{ 0x01, 0x17, KEY_SUBTITLE },     /* Subtitle / CC */
	{ 0x00, 0x10, KEY_MUTE },
	{ 0x01, 0x19, KEY_AUDIO },        /* L/R */ /* TODO better event */
	{ 0x01, 0x16, KEY_SLEEP },        /* Hibernate */
	{ 0x01, 0x16, KEY_SWITCHVIDEOMODE },
					  /* A/V */ /* TODO does not work */
	{ 0x00, 0x06, KEY_AGAIN },        /* Recall */
	{ 0x01, 0x16, KEY_KPPLUS },       /* Zoom+ */ /* TODO does not work */
	{ 0x01, 0x16, KEY_KPMINUS },      /* Zoom- */ /* TODO does not work */
	{ 0x02, 0x15, KEY_RED },
	{ 0x02, 0x0a, KEY_GREEN },
	{ 0x02, 0x1c, KEY_YELLOW },
	{ 0x02, 0x05, KEY_BLUE },
};

static u8 af9015_ir_table_twinhan[] = {
	0x00, 0xff, 0x16, 0xe9, 0x3f, 0x05, 0x00,
	0x00, 0xff, 0x07, 0xf8, 0x16, 0x01, 0x00,
	0x00, 0xff, 0x14, 0xeb, 0x11, 0x01, 0x00,
	0x00, 0xff, 0x1a, 0xe5, 0x4d, 0x00, 0x00,
	0x00, 0xff, 0x4c, 0xb3, 0x17, 0x00, 0x00,
	0x00, 0xff, 0x12, 0xed, 0x11, 0x00, 0x00,
	0x00, 0xff, 0x40, 0xbf, 0x0c, 0x00, 0x00,
	0x00, 0xff, 0x11, 0xee, 0x4a, 0x00, 0x00,
	0x00, 0xff, 0x54, 0xab, 0x13, 0x00, 0x00,
	0x00, 0xff, 0x41, 0xbe, 0x15, 0x01, 0x00,
	0x00, 0xff, 0x42, 0xbd, 0x0e, 0x01, 0x00,
	0x00, 0xff, 0x43, 0xbc, 0x17, 0x01, 0x00,
	0x00, 0xff, 0x50, 0xaf, 0x0f, 0x01, 0x00,
	0x00, 0xff, 0x4d, 0xb2, 0x1d, 0x00, 0x00,
	0x00, 0xff, 0x47, 0xb8, 0x13, 0x01, 0x00,
	0x00, 0xff, 0x05, 0xfa, 0x4b, 0x00, 0x00,
	0x00, 0xff, 0x02, 0xfd, 0x4e, 0x00, 0x00,
	0x00, 0xff, 0x0e, 0xf1, 0x06, 0x00, 0x00,
	0x00, 0xff, 0x1e, 0xe1, 0x52, 0x02, 0x00,
	0x00, 0xff, 0x0a, 0xf5, 0x51, 0x02, 0x00,
	0x00, 0xff, 0x10, 0xef, 0x10, 0x00, 0x00,
	0x00, 0xff, 0x49, 0xb6, 0x19, 0x01, 0x00,
	0x00, 0xff, 0x15, 0xea, 0x27, 0x00, 0x00,
	0x00, 0xff, 0x03, 0xfc, 0x1e, 0x00, 0x00,
	0x00, 0xff, 0x01, 0xfe, 0x1f, 0x00, 0x00,
	0x00, 0xff, 0x06, 0xf9, 0x20, 0x00, 0x00,
	0x00, 0xff, 0x09, 0xf6, 0x21, 0x00, 0x00,
	0x00, 0xff, 0x1d, 0xe2, 0x22, 0x00, 0x00,
	0x00, 0xff, 0x1f, 0xe0, 0x23, 0x00, 0x00,
	0x00, 0xff, 0x0d, 0xf2, 0x24, 0x00, 0x00,
	0x00, 0xff, 0x19, 0xe6, 0x25, 0x00, 0x00,
	0x00, 0xff, 0x1b, 0xe4, 0x26, 0x00, 0x00,
	0x00, 0xff, 0x00, 0xff, 0x2b, 0x00, 0x00,
	0x00, 0xff, 0x4a, 0xb5, 0x4c, 0x00, 0x00,
	0x00, 0xff, 0x4b, 0xb4, 0x52, 0x00, 0x00,
	0x00, 0xff, 0x51, 0xae, 0x51, 0x00, 0x00,
	0x00, 0xff, 0x52, 0xad, 0x4f, 0x00, 0x00,
	0x00, 0xff, 0x4e, 0xb1, 0x50, 0x00, 0x00,
	0x00, 0xff, 0x0c, 0xf3, 0x29, 0x00, 0x00,
	0x00, 0xff, 0x4f, 0xb0, 0x28, 0x00, 0x00,
	0x00, 0xff, 0x13, 0xec, 0x2a, 0x00, 0x00,
	0x00, 0xff, 0x17, 0xe8, 0x19, 0x00, 0x00,
	0x00, 0xff, 0x04, 0xfb, 0x0f, 0x00, 0x00,
	0x00, 0xff, 0x48, 0xb7, 0x0e, 0x00, 0x00,
	0x00, 0xff, 0x0f, 0xf0, 0x04, 0x00, 0x00,
	0x00, 0xff, 0x1c, 0xe3, 0x08, 0x00, 0x00,
	0x00, 0xff, 0x18, 0xe7, 0x15, 0x02, 0x00,
	0x00, 0xff, 0x53, 0xac, 0x0a, 0x02, 0x00,
	0x00, 0xff, 0x5e, 0xa1, 0x1c, 0x02, 0x00,
	0x00, 0xff, 0x5f, 0xa0, 0x05, 0x02, 0x00,
};

/* A-Link DTU(m) */
static struct dvb_usb_rc_key af9015_rc_keys_a_link[] = {
	{ 0x00, 0x1e, KEY_1 },
	{ 0x00, 0x1f, KEY_2 },
	{ 0x00, 0x20, KEY_3 },
	{ 0x00, 0x21, KEY_4 },
	{ 0x00, 0x22, KEY_5 },
	{ 0x00, 0x23, KEY_6 },
	{ 0x00, 0x24, KEY_7 },
	{ 0x00, 0x25, KEY_8 },
	{ 0x00, 0x26, KEY_9 },
	{ 0x00, 0x27, KEY_0 },
	{ 0x00, 0x2e, KEY_CHANNELUP },
	{ 0x00, 0x2d, KEY_CHANNELDOWN },
	{ 0x04, 0x28, KEY_ZOOM },
	{ 0x00, 0x41, KEY_MUTE },
	{ 0x00, 0x42, KEY_VOLUMEDOWN },
	{ 0x00, 0x43, KEY_VOLUMEUP },
	{ 0x00, 0x44, KEY_GOTO },         /* jump */
	{ 0x05, 0x45, KEY_POWER },
};

static u8 af9015_ir_table_a_link[] = {
	0x08, 0xf7, 0x12, 0xed, 0x45, 0x05, 0x00, /* power */
	0x08, 0xf7, 0x1a, 0xe5, 0x41, 0x00, 0x00, /* mute */
	0x08, 0xf7, 0x01, 0xfe, 0x1e, 0x00, 0x00, /* 1 */
	0x08, 0xf7, 0x1c, 0xe3, 0x21, 0x00, 0x00, /* 4 */
	0x08, 0xf7, 0x03, 0xfc, 0x24, 0x00, 0x00, /* 7 */
	0x08, 0xf7, 0x05, 0xfa, 0x28, 0x04, 0x00, /* zoom */
	0x08, 0xf7, 0x00, 0xff, 0x43, 0x00, 0x00, /* volume up */
	0x08, 0xf7, 0x16, 0xe9, 0x42, 0x00, 0x00, /* volume down */
	0x08, 0xf7, 0x0f, 0xf0, 0x1f, 0x00, 0x00, /* 2 */
	0x08, 0xf7, 0x0d, 0xf2, 0x22, 0x00, 0x00, /* 5 */
	0x08, 0xf7, 0x1b, 0xe4, 0x25, 0x00, 0x00, /* 8 */
	0x08, 0xf7, 0x06, 0xf9, 0x27, 0x00, 0x00, /* 0 */
	0x08, 0xf7, 0x14, 0xeb, 0x2e, 0x00, 0x00, /* channel up */
	0x08, 0xf7, 0x1d, 0xe2, 0x2d, 0x00, 0x00, /* channel down */
	0x08, 0xf7, 0x02, 0xfd, 0x20, 0x00, 0x00, /* 3 */
	0x08, 0xf7, 0x18, 0xe7, 0x23, 0x00, 0x00, /* 6 */
	0x08, 0xf7, 0x04, 0xfb, 0x26, 0x00, 0x00, /* 9 */
	0x08, 0xf7, 0x07, 0xf8, 0x44, 0x00, 0x00, /* jump */
};

/* MSI DIGIVOX mini II V3.0 */
static struct dvb_usb_rc_key af9015_rc_keys_msi[] = {
	{ 0x00, 0x1e, KEY_1 },
	{ 0x00, 0x1f, KEY_2 },
	{ 0x00, 0x20, KEY_3 },
	{ 0x00, 0x21, KEY_4 },
	{ 0x00, 0x22, KEY_5 },
	{ 0x00, 0x23, KEY_6 },
	{ 0x00, 0x24, KEY_7 },
	{ 0x00, 0x25, KEY_8 },
	{ 0x00, 0x26, KEY_9 },
	{ 0x00, 0x27, KEY_0 },
	{ 0x03, 0x0f, KEY_CHANNELUP },
	{ 0x03, 0x0e, KEY_CHANNELDOWN },
	{ 0x00, 0x42, KEY_VOLUMEDOWN },
	{ 0x00, 0x43, KEY_VOLUMEUP },
	{ 0x05, 0x45, KEY_POWER },
	{ 0x00, 0x52, KEY_UP },           /* up */
	{ 0x00, 0x51, KEY_DOWN },         /* down */
	{ 0x00, 0x28, KEY_ENTER },
};

static u8 af9015_ir_table_msi[] = {
	0x03, 0xfc, 0x17, 0xe8, 0x45, 0x05, 0x00, /* power */
	0x03, 0xfc, 0x0d, 0xf2, 0x51, 0x00, 0x00, /* down */
	0x03, 0xfc, 0x03, 0xfc, 0x52, 0x00, 0x00, /* up */
	0x03, 0xfc, 0x1a, 0xe5, 0x1e, 0x00, 0x00, /* 1 */
	0x03, 0xfc, 0x02, 0xfd, 0x1f, 0x00, 0x00, /* 2 */
	0x03, 0xfc, 0x04, 0xfb, 0x20, 0x00, 0x00, /* 3 */
	0x03, 0xfc, 0x1c, 0xe3, 0x21, 0x00, 0x00, /* 4 */
	0x03, 0xfc, 0x08, 0xf7, 0x22, 0x00, 0x00, /* 5 */
	0x03, 0xfc, 0x1d, 0xe2, 0x23, 0x00, 0x00, /* 6 */
	0x03, 0xfc, 0x11, 0xee, 0x24, 0x00, 0x00, /* 7 */
	0x03, 0xfc, 0x0b, 0xf4, 0x25, 0x00, 0x00, /* 8 */
	0x03, 0xfc, 0x10, 0xef, 0x26, 0x00, 0x00, /* 9 */
	0x03, 0xfc, 0x09, 0xf6, 0x27, 0x00, 0x00, /* 0 */
	0x03, 0xfc, 0x14, 0xeb, 0x43, 0x00, 0x00, /* volume up */
	0x03, 0xfc, 0x1f, 0xe0, 0x42, 0x00, 0x00, /* volume down */
	0x03, 0xfc, 0x15, 0xea, 0x0f, 0x03, 0x00, /* channel up */
	0x03, 0xfc, 0x05, 0xfa, 0x0e, 0x03, 0x00, /* channel down */
	0x03, 0xfc, 0x16, 0xe9, 0x28, 0x00, 0x00, /* enter */
};

/* MYGICTV U718 */
static struct dvb_usb_rc_key af9015_rc_keys_mygictv[] = {
	{ 0x00, 0x3d, KEY_SWITCHVIDEOMODE },
					  /* TV / AV */
	{ 0x05, 0x45, KEY_POWER },
	{ 0x00, 0x1e, KEY_1 },
	{ 0x00, 0x1f, KEY_2 },
	{ 0x00, 0x20, KEY_3 },
	{ 0x00, 0x21, KEY_4 },
	{ 0x00, 0x22, KEY_5 },
	{ 0x00, 0x23, KEY_6 },
	{ 0x00, 0x24, KEY_7 },
	{ 0x00, 0x25, KEY_8 },
	{ 0x00, 0x26, KEY_9 },
	{ 0x00, 0x27, KEY_0 },
	{ 0x00, 0x41, KEY_MUTE },
	{ 0x00, 0x2a, KEY_ESC },          /* Esc */
	{ 0x00, 0x2e, KEY_CHANNELUP },
	{ 0x00, 0x2d, KEY_CHANNELDOWN },
	{ 0x00, 0x42, KEY_VOLUMEDOWN },
	{ 0x00, 0x43, KEY_VOLUMEUP },
	{ 0x00, 0x52, KEY_UP },           /* up arrow */
	{ 0x00, 0x51, KEY_DOWN },         /* down arrow */
	{ 0x00, 0x4f, KEY_RIGHT },        /* right arrow */
	{ 0x00, 0x50, KEY_LEFT },         /* left arrow */
	{ 0x00, 0x28, KEY_ENTER },        /* ok */
	{ 0x01, 0x15, KEY_RECORD },
	{ 0x03, 0x13, KEY_PLAY },
	{ 0x01, 0x13, KEY_PAUSE },
	{ 0x01, 0x16, KEY_STOP },
	{ 0x03, 0x07, KEY_REWIND },       /* FR << */
	{ 0x03, 0x09, KEY_FASTFORWARD },  /* FF >> */
	{ 0x00, 0x3b, KEY_TIME },         /* TimeShift */
	{ 0x00, 0x3e, KEY_CAMERA },       /* Snapshot */
	{ 0x03, 0x16, KEY_CYCLEWINDOWS }, /* yellow, min / max */
	{ 0x00, 0x00, KEY_ZOOM },         /* 'select' (?) */
	{ 0x03, 0x16, KEY_SHUFFLE },      /* Shuffle */
	{ 0x03, 0x45, KEY_POWER },
};

static u8 af9015_ir_table_mygictv[] = {
	0x02, 0xbd, 0x0c, 0xf3, 0x3d, 0x00, 0x00, /* TV / AV */
	0x02, 0xbd, 0x14, 0xeb, 0x45, 0x05, 0x00, /* power */
	0x02, 0xbd, 0x00, 0xff, 0x1e, 0x00, 0x00, /* 1 */
	0x02, 0xbd, 0x01, 0xfe, 0x1f, 0x00, 0x00, /* 2 */
	0x02, 0xbd, 0x02, 0xfd, 0x20, 0x00, 0x00, /* 3 */
	0x02, 0xbd, 0x03, 0xfc, 0x21, 0x00, 0x00, /* 4 */
	0x02, 0xbd, 0x04, 0xfb, 0x22, 0x00, 0x00, /* 5 */
	0x02, 0xbd, 0x05, 0xfa, 0x23, 0x00, 0x00, /* 6 */
	0x02, 0xbd, 0x06, 0xf9, 0x24, 0x00, 0x00, /* 7 */
	0x02, 0xbd, 0x07, 0xf8, 0x25, 0x00, 0x00, /* 8 */
	0x02, 0xbd, 0x08, 0xf7, 0x26, 0x00, 0x00, /* 9 */
	0x02, 0xbd, 0x09, 0xf6, 0x27, 0x00, 0x00, /* 0 */
	0x02, 0xbd, 0x0a, 0xf5, 0x41, 0x00, 0x00, /* mute */
	0x02, 0xbd, 0x1c, 0xe3, 0x2a, 0x00, 0x00, /* esc */
	0x02, 0xbd, 0x1f, 0xe0, 0x43, 0x00, 0x00, /* volume up */
	0x02, 0xbd, 0x12, 0xed, 0x52, 0x00, 0x00, /* up arrow */
	0x02, 0xbd, 0x11, 0xee, 0x50, 0x00, 0x00, /* left arrow */
	0x02, 0xbd, 0x15, 0xea, 0x28, 0x00, 0x00, /* ok */
	0x02, 0xbd, 0x10, 0xef, 0x4f, 0x00, 0x00, /* right arrow */
	0x02, 0xbd, 0x13, 0xec, 0x51, 0x00, 0x00, /* down arrow */
	0x02, 0xbd, 0x0e, 0xf1, 0x42, 0x00, 0x00, /* volume down */
	0x02, 0xbd, 0x19, 0xe6, 0x15, 0x01, 0x00, /* record */
	0x02, 0xbd, 0x1e, 0xe1, 0x13, 0x03, 0x00, /* play */
	0x02, 0xbd, 0x16, 0xe9, 0x16, 0x01, 0x00, /* stop */
	0x02, 0xbd, 0x0b, 0xf4, 0x28, 0x04, 0x00, /* yellow, min / max */
	0x02, 0xbd, 0x0f, 0xf0, 0x3b, 0x00, 0x00, /* time shift */
	0x02, 0xbd, 0x18, 0xe7, 0x2e, 0x00, 0x00, /* channel up */
	0x02, 0xbd, 0x1a, 0xe5, 0x2d, 0x00, 0x00, /* channel down */
	0x02, 0xbd, 0x17, 0xe8, 0x3e, 0x00, 0x00, /* snapshot */
	0x02, 0xbd, 0x40, 0xbf, 0x13, 0x01, 0x00, /* pause */
	0x02, 0xbd, 0x41, 0xbe, 0x09, 0x03, 0x00, /* FF >> */
	0x02, 0xbd, 0x42, 0xbd, 0x07, 0x03, 0x00, /* FR << */
	0x02, 0xbd, 0x43, 0xbc, 0x00, 0x00, 0x00, /* 'select' (?) */
	0x02, 0xbd, 0x44, 0xbb, 0x16, 0x03, 0x00, /* shuffle */
	0x02, 0xbd, 0x45, 0xba, 0x45, 0x03, 0x00, /* power */
};

/* KWorld PlusTV Dual DVB-T Stick (DVB-T 399U) */
static u8 af9015_ir_table_kworld[] = {
	0x86, 0x6b, 0x0c, 0xf3, 0x2e, 0x07, 0x00,
	0x86, 0x6b, 0x16, 0xe9, 0x2d, 0x07, 0x00,
	0x86, 0x6b, 0x1d, 0xe2, 0x37, 0x07, 0x00,
	0x86, 0x6b, 0x00, 0xff, 0x1e, 0x07, 0x00,
	0x86, 0x6b, 0x01, 0xfe, 0x1f, 0x07, 0x00,
	0x86, 0x6b, 0x02, 0xfd, 0x20, 0x07, 0x00,
	0x86, 0x6b, 0x03, 0xfc, 0x21, 0x07, 0x00,
	0x86, 0x6b, 0x04, 0xfb, 0x22, 0x07, 0x00,
	0x86, 0x6b, 0x05, 0xfa, 0x23, 0x07, 0x00,
	0x86, 0x6b, 0x06, 0xf9, 0x24, 0x07, 0x00,
	0x86, 0x6b, 0x07, 0xf8, 0x25, 0x07, 0x00,
	0x86, 0x6b, 0x08, 0xf7, 0x26, 0x07, 0x00,
	0x86, 0x6b, 0x09, 0xf6, 0x4d, 0x07, 0x00,
	0x86, 0x6b, 0x0a, 0xf5, 0x4e, 0x07, 0x00,
	0x86, 0x6b, 0x14, 0xeb, 0x4f, 0x07, 0x00,
	0x86, 0x6b, 0x1e, 0xe1, 0x50, 0x07, 0x00,
	0x86, 0x6b, 0x17, 0xe8, 0x52, 0x07, 0x00,
	0x86, 0x6b, 0x1f, 0xe0, 0x51, 0x07, 0x00,
	0x86, 0x6b, 0x0e, 0xf1, 0x0b, 0x07, 0x00,
	0x86, 0x6b, 0x20, 0xdf, 0x0c, 0x07, 0x00,
	0x86, 0x6b, 0x42, 0xbd, 0x0d, 0x07, 0x00,
	0x86, 0x6b, 0x0b, 0xf4, 0x0e, 0x07, 0x00,
	0x86, 0x6b, 0x43, 0xbc, 0x0f, 0x07, 0x00,
	0x86, 0x6b, 0x10, 0xef, 0x10, 0x07, 0x00,
	0x86, 0x6b, 0x21, 0xde, 0x11, 0x07, 0x00,
	0x86, 0x6b, 0x13, 0xec, 0x12, 0x07, 0x00,
	0x86, 0x6b, 0x11, 0xee, 0x13, 0x07, 0x00,
	0x86, 0x6b, 0x12, 0xed, 0x14, 0x07, 0x00,
	0x86, 0x6b, 0x19, 0xe6, 0x15, 0x07, 0x00,
	0x86, 0x6b, 0x1a, 0xe5, 0x16, 0x07, 0x00,
	0x86, 0x6b, 0x1b, 0xe4, 0x17, 0x07, 0x00,
	0x86, 0x6b, 0x4b, 0xb4, 0x18, 0x07, 0x00,
	0x86, 0x6b, 0x40, 0xbf, 0x19, 0x07, 0x00,
	0x86, 0x6b, 0x44, 0xbb, 0x1a, 0x07, 0x00,
	0x86, 0x6b, 0x41, 0xbe, 0x1b, 0x07, 0x00,
	0x86, 0x6b, 0x22, 0xdd, 0x1c, 0x07, 0x00,
	0x86, 0x6b, 0x15, 0xea, 0x1d, 0x07, 0x00,
	0x86, 0x6b, 0x0f, 0xf0, 0x3f, 0x07, 0x00,
	0x86, 0x6b, 0x1c, 0xe3, 0x40, 0x07, 0x00,
	0x86, 0x6b, 0x4a, 0xb5, 0x41, 0x07, 0x00,
	0x86, 0x6b, 0x48, 0xb7, 0x42, 0x07, 0x00,
	0x86, 0x6b, 0x49, 0xb6, 0x43, 0x07, 0x00,
	0x86, 0x6b, 0x18, 0xe7, 0x44, 0x07, 0x00,
	0x86, 0x6b, 0x23, 0xdc, 0x45, 0x07, 0x00,
};

/* AverMedia Volar X */
static struct dvb_usb_rc_key af9015_rc_keys_avermedia[] = {
	{ 0x05, 0x3d, KEY_PROG1 },       /* SOURCE */
	{ 0x05, 0x12, KEY_POWER },       /* POWER */
	{ 0x05, 0x1e, KEY_1 },           /* 1 */
	{ 0x05, 0x1f, KEY_2 },           /* 2 */
	{ 0x05, 0x20, KEY_3 },           /* 3 */
	{ 0x05, 0x21, KEY_4 },           /* 4 */
	{ 0x05, 0x22, KEY_5 },           /* 5 */
	{ 0x05, 0x23, KEY_6 },           /* 6 */
	{ 0x05, 0x24, KEY_7 },           /* 7 */
	{ 0x05, 0x25, KEY_8 },           /* 8 */
	{ 0x05, 0x26, KEY_9 },           /* 9 */
	{ 0x05, 0x3f, KEY_LEFT },        /* L / DISPLAY */
	{ 0x05, 0x27, KEY_0 },           /* 0 */
	{ 0x05, 0x0f, KEY_RIGHT },       /* R / CH RTN */
	{ 0x05, 0x18, KEY_PROG2 },       /* SNAP SHOT */
	{ 0x05, 0x1c, KEY_PROG3 },       /* 16-CH PREV */
	{ 0x05, 0x2d, KEY_VOLUMEDOWN },  /* VOL DOWN */
	{ 0x05, 0x3e, KEY_ZOOM },        /* FULL SCREEN */
	{ 0x05, 0x2e, KEY_VOLUMEUP },    /* VOL UP */
	{ 0x05, 0x10, KEY_MUTE },        /* MUTE */
	{ 0x05, 0x04, KEY_AUDIO },       /* AUDIO */
	{ 0x05, 0x15, KEY_RECORD },      /* RECORD */
	{ 0x05, 0x11, KEY_PLAY },        /* PLAY */
	{ 0x05, 0x16, KEY_STOP },        /* STOP */
	{ 0x05, 0x0c, KEY_PLAYPAUSE },   /* TIMESHIFT / PAUSE */
	{ 0x05, 0x05, KEY_BACK },        /* << / RED */
	{ 0x05, 0x09, KEY_FORWARD },     /* >> / YELLOW */
	{ 0x05, 0x17, KEY_TEXT },        /* TELETEXT */
	{ 0x05, 0x0a, KEY_EPG },         /* EPG */
	{ 0x05, 0x13, KEY_MENU },        /* MENU */

	{ 0x05, 0x0e, KEY_CHANNELUP },   /* CH UP */
	{ 0x05, 0x0d, KEY_CHANNELDOWN }, /* CH DOWN */
	{ 0x05, 0x19, KEY_FIRST },       /* |<< / GREEN */
	{ 0x05, 0x08, KEY_LAST },        /* >>| / BLUE */
};

static u8 af9015_ir_table_avermedia[] = {
	0x02, 0xfd, 0x00, 0xff, 0x12, 0x05, 0x00,
	0x02, 0xfd, 0x01, 0xfe, 0x3d, 0x05, 0x00,
	0x02, 0xfd, 0x03, 0xfc, 0x17, 0x05, 0x00,
	0x02, 0xfd, 0x04, 0xfb, 0x0a, 0x05, 0x00,
	0x02, 0xfd, 0x05, 0xfa, 0x1e, 0x05, 0x00,
	0x02, 0xfd, 0x06, 0xf9, 0x1f, 0x05, 0x00,
	0x02, 0xfd, 0x07, 0xf8, 0x20, 0x05, 0x00,
	0x02, 0xfd, 0x09, 0xf6, 0x21, 0x05, 0x00,
	0x02, 0xfd, 0x0a, 0xf5, 0x22, 0x05, 0x00,
	0x02, 0xfd, 0x0b, 0xf4, 0x23, 0x05, 0x00,
	0x02, 0xfd, 0x0d, 0xf2, 0x24, 0x05, 0x00,
	0x02, 0xfd, 0x0e, 0xf1, 0x25, 0x05, 0x00,
	0x02, 0xfd, 0x0f, 0xf0, 0x26, 0x05, 0x00,
	0x02, 0xfd, 0x11, 0xee, 0x27, 0x05, 0x00,
	0x02, 0xfd, 0x08, 0xf7, 0x04, 0x05, 0x00,
	0x02, 0xfd, 0x0c, 0xf3, 0x3e, 0x05, 0x00,
	0x02, 0xfd, 0x10, 0xef, 0x1c, 0x05, 0x00,
	0x02, 0xfd, 0x12, 0xed, 0x3f, 0x05, 0x00,
	0x02, 0xfd, 0x13, 0xec, 0x0f, 0x05, 0x00,
	0x02, 0xfd, 0x14, 0xeb, 0x10, 0x05, 0x00,
	0x02, 0xfd, 0x15, 0xea, 0x13, 0x05, 0x00,
	0x02, 0xfd, 0x17, 0xe8, 0x18, 0x05, 0x00,
	0x02, 0xfd, 0x18, 0xe7, 0x11, 0x05, 0x00,
	0x02, 0xfd, 0x19, 0xe6, 0x15, 0x05, 0x00,
	0x02, 0xfd, 0x1a, 0xe5, 0x0c, 0x05, 0x00,
	0x02, 0xfd, 0x1b, 0xe4, 0x16, 0x05, 0x00,
	0x02, 0xfd, 0x1c, 0xe3, 0x09, 0x05, 0x00,
	0x02, 0xfd, 0x1d, 0xe2, 0x05, 0x05, 0x00,
	0x02, 0xfd, 0x1e, 0xe1, 0x2d, 0x05, 0x00,
	0x02, 0xfd, 0x1f, 0xe0, 0x2e, 0x05, 0x00,
	0x03, 0xfc, 0x00, 0xff, 0x08, 0x05, 0x00,
	0x03, 0xfc, 0x01, 0xfe, 0x19, 0x05, 0x00,
	0x03, 0xfc, 0x02, 0xfd, 0x0d, 0x05, 0x00,
	0x03, 0xfc, 0x03, 0xfc, 0x0e, 0x05, 0x00,
};

/* Digittrade DVB-T USB Stick */
static struct dvb_usb_rc_key af9015_rc_keys_digittrade[] = {
	{ 0x01, 0x0f, KEY_LAST },	/* RETURN */
	{ 0x05, 0x17, KEY_TEXT },	/* TELETEXT */
	{ 0x01, 0x08, KEY_EPG },	/* EPG */
	{ 0x05, 0x13, KEY_POWER },	/* POWER */
	{ 0x01, 0x09, KEY_ZOOM },	/* FULLSCREEN */
	{ 0x00, 0x40, KEY_AUDIO },	/* DUAL SOUND */
	{ 0x00, 0x2c, KEY_PRINT },	/* SNAPSHOT */
	{ 0x05, 0x16, KEY_SUBTITLE },	/* SUBTITLE */
	{ 0x00, 0x52, KEY_CHANNELUP },	/* CH Up */
	{ 0x00, 0x51, KEY_CHANNELDOWN },/* Ch Dn */
	{ 0x00, 0x57, KEY_VOLUMEUP },	/* Vol Up */
	{ 0x00, 0x56, KEY_VOLUMEDOWN },	/* Vol Dn */
	{ 0x01, 0x10, KEY_MUTE },	/* MUTE */
	{ 0x00, 0x27, KEY_0 },
	{ 0x00, 0x1e, KEY_1 },
	{ 0x00, 0x1f, KEY_2 },
	{ 0x00, 0x20, KEY_3 },
	{ 0x00, 0x21, KEY_4 },
	{ 0x00, 0x22, KEY_5 },
	{ 0x00, 0x23, KEY_6 },
	{ 0x00, 0x24, KEY_7 },
	{ 0x00, 0x25, KEY_8 },
	{ 0x00, 0x26, KEY_9 },
	{ 0x01, 0x17, KEY_PLAYPAUSE },	/* TIMESHIFT */
	{ 0x01, 0x15, KEY_RECORD },	/* RECORD */
	{ 0x03, 0x13, KEY_PLAY },	/* PLAY */
	{ 0x01, 0x16, KEY_STOP },	/* STOP */
	{ 0x01, 0x13, KEY_PAUSE },	/* PAUSE */
};

static u8 af9015_ir_table_digittrade[] = {
	0x00, 0xff, 0x06, 0xf9, 0x13, 0x05, 0x00,
	0x00, 0xff, 0x4d, 0xb2, 0x17, 0x01, 0x00,
	0x00, 0xff, 0x1f, 0xe0, 0x2c, 0x00, 0x00,
	0x00, 0xff, 0x0a, 0xf5, 0x15, 0x01, 0x00,
	0x00, 0xff, 0x0e, 0xf1, 0x16, 0x01, 0x00,
	0x00, 0xff, 0x09, 0xf6, 0x09, 0x01, 0x00,
	0x00, 0xff, 0x01, 0xfe, 0x08, 0x01, 0x00,
	0x00, 0xff, 0x05, 0xfa, 0x10, 0x01, 0x00,
	0x00, 0xff, 0x02, 0xfd, 0x56, 0x00, 0x00,
	0x00, 0xff, 0x40, 0xbf, 0x57, 0x00, 0x00,
	0x00, 0xff, 0x19, 0xe6, 0x52, 0x00, 0x00,
	0x00, 0xff, 0x17, 0xe8, 0x51, 0x00, 0x00,
	0x00, 0xff, 0x10, 0xef, 0x0f, 0x01, 0x00,
	0x00, 0xff, 0x54, 0xab, 0x27, 0x00, 0x00,
	0x00, 0xff, 0x1b, 0xe4, 0x1e, 0x00, 0x00,
	0x00, 0xff, 0x11, 0xee, 0x1f, 0x00, 0x00,
	0x00, 0xff, 0x15, 0xea, 0x20, 0x00, 0x00,
	0x00, 0xff, 0x12, 0xed, 0x21, 0x00, 0x00,
	0x00, 0xff, 0x16, 0xe9, 0x22, 0x00, 0x00,
	0x00, 0xff, 0x4c, 0xb3, 0x23, 0x00, 0x00,
	0x00, 0xff, 0x48, 0xb7, 0x24, 0x00, 0x00,
	0x00, 0xff, 0x04, 0xfb, 0x25, 0x00, 0x00,
	0x00, 0xff, 0x00, 0xff, 0x26, 0x00, 0x00,
	0x00, 0xff, 0x1e, 0xe1, 0x13, 0x03, 0x00,
	0x00, 0xff, 0x1a, 0xe5, 0x13, 0x01, 0x00,
	0x00, 0xff, 0x03, 0xfc, 0x17, 0x05, 0x00,
	0x00, 0xff, 0x0d, 0xf2, 0x16, 0x05, 0x00,
	0x00, 0xff, 0x1d, 0xe2, 0x40, 0x00, 0x00,
};

#endif
