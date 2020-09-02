/*
  GPIO I2C Driver

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

#ifndef Camsensor_gpio_H
#define Camsensor_gpio_H

/**
 * This struct defines a i2c command for I2C slave I/O operation.
 */
#define RET_OK      0
#define RET_ERR     -1

#define boolean int

struct CameraSensorI2CCmdType {
    unsigned char   slave_addr;         // Slave address
    unsigned char   *pwdata;            // Pointer to Write data buffer
    unsigned short  wlen;               // Count of bytes to transfer
    unsigned char   *prdata;            // Pointer to Write data buffer
    unsigned short  rlen;               // Count of bytes to transfer
} __attribute__((packed, aligned(4)));

extern int camsensor_gpioi2c_read(struct CameraSensorI2CCmdType *);
extern int camsensor_gpioi2c_write(struct CameraSensorI2CCmdType *);

#endif /* Camsensor_gpio_H */
