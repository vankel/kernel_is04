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


#include <linux/kernel.h>
#include <linux/module.h>

#include "smd_private.h"
#include "dev_model.h"

MODULE_LICENSE("GPL");

static uint32_t model_no;
static uint32_t model_other;
static uint32_t model_variant;
static uint32_t model_sub;
static uint32_t model_sd;

void dev_model_init_model_table(void)
{
	unsigned char *smem_ptr=NULL;

	smem_ptr = (unsigned char *)smem_alloc( SMEM_OEM_015, sizeof(unsigned char)); 
	if(smem_ptr != NULL) {
		pr_info("dev_model smem : 0x%02x\n", *smem_ptr);

		switch(*smem_ptr) {
			case 0x60:
				model_no = DEV_MODEL_NO_0;
				break;
			case 0x61:
			case 0x71:
				model_no = DEV_MODEL_NO_1;
				break;
			case 0x62:
			case 0x72:
				model_no = DEV_MODEL_NO_11;
				break;
			case 0x63:
			case 0x73:
				model_no = DEV_MODEL_NO_2;
				break;
			case 0x64:
			case 0x74:
				model_no = DEV_MODEL_NO_3;
				break;
			case 0x65:
			case 0x75:
				model_no = DEV_MODEL_NO_PR;
				break;
			case 0x66:
			case 0x76:
				model_no = DEV_MODEL_NO_CM;
				break;
			default:
				model_no = DEV_MODEL_NO_CM;
				break;
		}

		switch(*smem_ptr) {
			case 0x60:
			case 0x61:
			case 0x62:
			case 0x63:
			case 0x64:
				model_variant = DEV_MODEL_VARIANT_0;
				break;
			case 0x71:
			case 0x72:
			case 0x73:
			case 0x74:
				model_variant = DEV_MODEL_VARIANT_1;
				break;
			default:
				model_variant = DEV_MODEL_VARIANT_0;
				break;
		}

		switch(*smem_ptr) {
			case 0x62:
			case 0x63:
			case 0x64:
				model_sub = DEV_MODEL_SUB_0;
				break;
			case 0x72:
			case 0x73:
			case 0x74:
				model_sub = DEV_MODEL_SUB_1;
				break;
			default:
				model_sub = DEV_MODEL_SUB_0;
				break;
		}
	}
	else {
		model_no = DEV_MODEL_NO_0;
		model_variant = DEV_MODEL_VARIANT_0;
		model_sub = DEV_MODEL_SUB_0;
	}


	smem_ptr = (unsigned char *)smem_alloc( SMEM_OEM_022, sizeof(unsigned char)); 
	if(smem_ptr != NULL) {
		pr_info("dev_model_mem smem : 0x%02x\n", *smem_ptr);

		switch(*smem_ptr) {
			case 0x03:
				model_other = DEV_MODEL_OTHER_0;
				break;
			case 0x04:
				model_other = DEV_MODEL_OTHER_1;
				break;
			default:
				model_other = DEV_MODEL_OTHER_1;
				break;
		}
	}
	else {
		model_other = DEV_MODEL_OTHER_1;
	}


	smem_ptr = (unsigned char *)smem_alloc( SMEM_OEM_021, sizeof(unsigned char)); 
	if(smem_ptr != NULL) {
		pr_info("dev_model_sd smem : 0x%02x\n", *smem_ptr);

		switch(*smem_ptr) {
			case 0x00:
				model_sd = DEV_MODEL_SD_X;
				break;
			case 0x01:
				model_sd = DEV_MODEL_SD_S;
				break;
			default:
				model_sd = DEV_MODEL_SD_S;
				break;
		}
	}
	else {
		model_sd = DEV_MODEL_SD_S;
	}


	pr_info("dev_model no : %d , other : %d , variant : %d , sub : %d , sd : %d\n",
		model_no, model_other, model_variant, model_sub, model_sd);
}
EXPORT_SYMBOL(dev_model_init_model_table);


int dev_model_get_model_no(void)
{
	return model_no;
}
EXPORT_SYMBOL(dev_model_get_model_no);


int dev_model_get_model_other(void)
{
	return model_other;
}
EXPORT_SYMBOL(dev_model_get_model_other);


int dev_model_get_model_variant(void)
{
	return model_variant;
}
EXPORT_SYMBOL(dev_model_get_model_variant);


int dev_model_get_model_sub(void)
{
	return model_sub;
}
EXPORT_SYMBOL(dev_model_get_model_sub);

int dev_model_get_model_sd(void)
{
	return model_sd;
}
EXPORT_SYMBOL(dev_model_get_model_sd);



