/*
Model Driver

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

#ifndef __TSB_MODEL_H
#define __TSB_MODEL_H

#define TSB_MODEL_NO_0		0
#define TSB_MODEL_NO_1		1
#define TSB_MODEL_NO_11		2
#define TSB_MODEL_NO_2		3
#define TSB_MODEL_NO_3		4
#define TSB_MODEL_NO_PR		5
#define TSB_MODEL_NO_CM		6

#define TSB_MODEL_OTHER_0	0
#define TSB_MODEL_OTHER_1	1

#define TSB_MODEL_VARIANT_0	0
#define TSB_MODEL_VARIANT_1	1

#define TSB_MODEL_SUB_0		0
#define TSB_MODEL_SUB_1		1

#define TSB_MODEL_SD_X		0
#define TSB_MODEL_SD_S		1

void tsb_model_init_model_table();
int tsb_model_get_model_no();
int tsb_model_get_model_other();
int tsb_model_get_model_variant();
int tsb_model_get_model_sub();
int tsb_model_get_model_sd();

#endif
