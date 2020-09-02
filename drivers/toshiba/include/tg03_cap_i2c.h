/*
GPIO-i2c Driver (for Capacitive touch panel)

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

#ifndef CAP_I2C_H
#define CAP_I2C_H

/* I2C Control Informatino Sturcture */
typedef struct{
    uint8_t     slave_addr;     /* slave address                      */
    uint8_t     *buf_ptr;       /* pointer to buffer in caller space  */
    uint16_t    offset;         /* offset in I2C device to read/write */
    uint16_t    len;            /* count of bytes to transfer         */
} I2C_INFO;

/* prototypes                       */
#ifdef __cplusplus
extern "C" {
#endif

extern bool cap_i2c_init(void);
extern bool cap_i2c_read( I2C_INFO *cmd_ptr );
extern bool cap_i2c_write( I2C_INFO *cmd_ptr );

#ifdef __cplusplus
}
#endif


#endif /* CAP_I2C_H */
