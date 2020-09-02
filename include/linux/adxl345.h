/*

* Certain software is contributed or developed by TOSHIBA CORPORATION.
*
* Copyright (C) 2010 TOSHIBA CORPORATION All rights reserved.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by FSF, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* This code is based on adxl345.h.
* The original copyright and notice are described below.
*/

/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef LINUX_ADXL345_MODULE_H
#define LINUX_ADXL345_MODULE_H

/**
 * struct bma150_platform_data - data to set up bma150 driver
 *
 * @setup: optional callback to activate the driver.
 * @teardown: optional callback to invalidate the driver.
 *
**/

#define  NV_TSB_TOP_ITEMS_I  10000
#define  NV_ACCM_X_OFFSET_I  (NV_TSB_TOP_ITEMS_I+37)
#define  NV_ACCM_Y_OFFSET_I  (NV_TSB_TOP_ITEMS_I+38)
#define  NV_ACCM_Z_OFFSET_I  (NV_TSB_TOP_ITEMS_I+39)


struct adxl345_platform_data {
	int (*setup)(struct device *);
	void (*teardown)(struct device *);
    int (*read_nvitem)(unsigned int id, unsigned long *data);
    int (*write_nvitem)(unsigned int id, unsigned long *data);
};

#endif /* LINUX_ADXL345_MODULE_H */
