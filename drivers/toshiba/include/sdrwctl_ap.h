/*
  sdrwctl_apl.h

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

struct sdrwctl_reg_rw {
    unsigned short addr;
    unsigned short val;
    unsigned short mask;
};

struct sdrwctl_reg_rw_mul {
    unsigned short info_addr;
    unsigned short chkbit;
    unsigned short fifo_addr;
    unsigned short *val;
    unsigned long cnt;
};

struct sdrwctl_reg_info {
    unsigned char sd_rca[2];
    unsigned char sd_csd[16];
    unsigned char sd_cid[16];
};

struct sdrwctl_reg_sec_rw {
    unsigned short info_addr;
    unsigned short chkbit;
    unsigned short addr;
    unsigned short *val;
    unsigned long cnt;
};

struct sdrwctl_reg_cmd_exec {
    unsigned short cmd_addr;
    unsigned short cmd;
    unsigned short info_addr;
    unsigned short info_check;
};

struct sdrwctl_reg_sec_codec {
    unsigned short info_addr;
    unsigned short chkbit_r;
    unsigned short chkbit_w;
    unsigned short addr_r;
    unsigned short addr_w;
    unsigned short *val;
    unsigned long cnt;
};

#define SDRWCTL_MAGIC 'a'
#define SDRWCTL_GET_STATE           _IOR(SDRWCTL_MAGIC, 0, unsigned long)
#define SDRWCTL_GET_INFO            _IOR(SDRWCTL_MAGIC, 1, unsigned long)
#define SDRWCTL_GET_PROSTATE            _IOR(SDRWCTL_MAGIC, 2, unsigned long)
#define SDRWCTL_SET_STATE           _IOW(SDRWCTL_MAGIC, 3, unsigned long)
#define SDRWCTL_GET_CHECK           _IOR(SDRWCTL_MAGIC, 4, unsigned long)
#define SDRWCTL_WRITE_REG16         _IOW(SDRWCTL_MAGIC, 5, struct sdrwctl_reg_rw)
#define SDRWCTL_READ_REG16          _IOR(SDRWCTL_MAGIC, 6, struct sdrwctl_reg_rw)
#define SDRWCTL_WRITE_REG16_MASK        _IOW(SDRWCTL_MAGIC, 7, struct sdrwctl_reg_rw)
#define SDRWCTL_WRITE_REG16_MUL         _IOW(SDRWCTL_MAGIC, 8, struct sdrwctl_reg_rw_mul)
#define SDRWCTL_READ_REG16_MUL          _IOR(SDRWCTL_MAGIC, 9, struct sdrwctl_reg_rw_mul)
#define SDRWCTL_GET_EXT             _IOR(SDRWCTL_MAGIC, 10, unsigned long)
#define SDRWCTL_WRITE_SECREG16_MUL      _IOW(SDRWCTL_MAGIC, 11, struct sdrwctl_reg_rw)
#define SDRWCTL_READ_SECREG16_MUL       _IOR(SDRWCTL_MAGIC, 12, struct sdrwctl_reg_rw)
#define SDRWCTL_GET_CHECK_CLR           _IO(SDRWCTL_MAGIC, 13)
#define SDRWCTL_LOCK                _IO(SDRWCTL_MAGIC, 14)
#define SDRWCTL_UNLOCK              _IO(SDRWCTL_MAGIC, 15)
#define SDRWCTL_CMD             _IOR(SDRWCTL_MAGIC, 16, struct sdrwctl_reg_cmd_exec)
#define SDRWCTL_INIT                _IO(SDRWCTL_MAGIC, 17)
#define SDRWCTL_DEINIT              _IO(SDRWCTL_MAGIC, 18)
#define SDRWCTL_TRANS_SECREG16_MUL      _IOWR(SDRWCTL_MAGIC, 19, struct sdrwctl_reg_sec_codec)
#define SDRWCTL_GET_BIT             _IOR(SDRWCTL_MAGIC, 20, int)
#define SDRWCTL_SET_BIT             _IOW(SDRWCTL_MAGIC, 21, int)
#define SDRWCTL_WRITE_REG16_SEC         _IOW(SDRWCTL_MAGIC, 22, struct sdrwctl_reg_rw_mul)
#define SDRWCTL_READ_REG16_SEC          _IOR(SDRWCTL_MAGIC, 23, struct sdrwctl_reg_rw_mul)

#define SDRWCTL_TRANSFER        1
#define SDRWCTL_OTHER           0

#define SDRWCTL_SLP_IMPOS       1
#define SDRWCTL_SLP_POS         0

#define SDRWCTL_PROTECT         1
#define SDRWCTL_NO_PROTECT      0

#define SDRWCTL_CHANGED         1
#define SDRWCTL_NO_CHANGE       0

#define SDRWCTL_EXT         1
#define SDRWCTL_NOT_EXT         0
