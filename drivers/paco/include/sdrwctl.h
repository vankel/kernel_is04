/*
  sdrwctl.h

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

struct sdrwctl_ops {
    int (*get_state)(struct mmc_host *, int *);
    int (*set_state)(struct mmc_host *, unsigned short *);
    int (*read_reg)(struct mmc_host *, struct sdrwctl_reg_rw *);
    int (*write_reg)(struct mmc_host *, struct sdrwctl_reg_rw *);
    int (*write_reg_mas)(struct mmc_host *, struct sdrwctl_reg_rw *);
    int (*read_reg_mul)(struct mmc_host *, struct sdrwctl_reg_rw_mul *);
    int (*write_reg_mul)(struct mmc_host *, struct sdrwctl_reg_rw_mul *);
    int (*get_info)(struct mmc_host *, struct sdrwctl_reg_info *);
    int (*get_prostate)(struct mmc_host *, unsigned short *);
    int (*get_check)(struct mmc_host *, int *);
    int (*get_ext)(struct mmc_host *, int *);
    int (*secure_write)(struct mmc_host *, struct sdrwctl_reg_sec_rw *);
    int (*secure_read)(struct mmc_host *, struct sdrwctl_reg_sec_rw *);
    int (*flg_init)(struct mmc_host *);
    int (*lock)(struct mmc_host *);
    int (*unlock)(struct mmc_host *);
    int (*cmd)(struct mmc_host *, struct sdrwctl_reg_cmd_exec *);
    int (*init)(struct mmc_host *);
    int (*deinit)(struct mmc_host *);
    int (*secure_codec)(struct mmc_host *, struct sdrwctl_reg_sec_codec *);
    int (*get_bit)(struct mmc_host *, int *);
    int (*put_bit)(struct mmc_host *, int *);
    int (*read_reg_sec)(struct mmc_host *, struct sdrwctl_reg_rw_mul *);
    int (*write_reg_sec)(struct mmc_host *, struct sdrwctl_reg_rw_mul *);
};

