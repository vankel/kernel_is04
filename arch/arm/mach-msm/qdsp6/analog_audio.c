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
 * This code is based on analog_audio.c.
 * The original copyright and notice are described below.
 */
/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/gpio.h>
#include <mach/pmic.h>
#include <mach/msm_qdsp6_audio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/input.h>

#define GPIO_HEADSET_AMP 157

// analog mic control -->
static bool cur_amic_state;
static bool amic_for_txpath;
static bool amic_for_hswitch;
static bool headset_key_state;
static struct input_dev *in_dev;

static void headsetkey_pressed(void){
	if(headset_key_state) return;
	headset_key_state = true;
	input_report_key(in_dev, 79, 1);
	input_sync(in_dev);
}

static void headsetkey_released(void){
	if(!headset_key_state) return;
	headset_key_state = false;
	input_report_key(in_dev, 79, 0);
	input_sync(in_dev);
}

static void _audio_analog_mic_ctrl(void)
{
	if(cur_amic_state == true){
		if(!(amic_for_txpath || amic_for_hswitch)){ //analog mic off
			pmic_mic_en(0);
			cur_amic_state = false;
		}
	}else{
		if(amic_for_txpath || amic_for_hswitch){    //analog mic on
			pmic_mic_en(1);
			cur_amic_state = true;
		}
	}
}
// <-- analog mic control

void analog_init(void)
{
	int ret;
	/* stereo pmic init */
#if defined (CONFIG_MACH_QSD8X50_TG03)
#else
	pmic_spkr_set_gain(LEFT_SPKR, SPKR_GAIN_PLUS12DB);
	pmic_spkr_set_gain(RIGHT_SPKR, SPKR_GAIN_PLUS12DB);
#endif
//	pmic_mic_set_volt(MIC_VOLT_1_80V);
	pmic_mic_set_volt(MIC_VOLT_2_00V);
#if defined (CONFIG_MACH_QSD8X50_TG03)
#else
	gpio_direction_output(GPIO_HEADSET_AMP, 1);
	gpio_set_value(GPIO_HEADSET_AMP, 0);
#endif
	cur_amic_state = false;
	amic_for_txpath = false;
	amic_for_hswitch = false;
	headset_key_state = false;

	in_dev = input_allocate_device();
	if (in_dev == NULL) return;
	in_dev->name = "h2w headset";
	in_dev->evbit[0] = BIT_MASK(EV_KEY);
	in_dev->keybit[BIT_WORD(79)] = BIT_MASK(79);
	ret = input_register_device(in_dev);
	if (ret < 0) input_free_device(in_dev);
}

void analog_headset_enable(int en)
{
	/* enable audio amp */
#if defined (CONFIG_MACH_QSD8X50_TG03)
#else
	gpio_set_value(GPIO_HEADSET_AMP, !!en);
#endif
}

void analog_speaker_enable(int en)
{
#if defined (CONFIG_MACH_QSD8X50_TG03)
#else
	struct spkr_config_mode scm;
	memset(&scm, 0, sizeof(scm));

	if (en) {
		scm.is_right_chan_en = 1;
		scm.is_left_chan_en = 1;
		scm.is_stereo_en = 1;
		scm.is_hpf_en = 1;
		pmic_spkr_en_mute(LEFT_SPKR, 0);
		pmic_spkr_en_mute(RIGHT_SPKR, 0);
		pmic_set_spkr_configuration(&scm);
		pmic_spkr_en(LEFT_SPKR, 1);
		pmic_spkr_en(RIGHT_SPKR, 1);
		
		/* unmute */
		pmic_spkr_en_mute(LEFT_SPKR, 1);
		pmic_spkr_en_mute(RIGHT_SPKR, 1);
	} else {
		pmic_spkr_en_mute(LEFT_SPKR, 0);
		pmic_spkr_en_mute(RIGHT_SPKR, 0);

		pmic_spkr_en(LEFT_SPKR, 0);
		pmic_spkr_en(RIGHT_SPKR, 0);

		pmic_set_spkr_configuration(&scm);
	}
#endif
}

void analog_mic_enable(int en)
{
	// analog mic control -->
	//pmic_mic_en(en); del
	if (en == 0) amic_for_txpath = false;
	else amic_for_txpath = true;
	_audio_analog_mic_ctrl();
	// <-- analog mic control
}

static bool gpio103check_thread_break;

static void gpio103check_thread(void * unused)
{
	int pre_value,value,i,h;

	gpio103check_thread_break = false;

	for(i=0;i<3;i++){
		//gpio103 check
		if( (value = gpio_get_value(103) ) != 0 ) goto NEXT_STEP;
		mdelay(25);
		if(gpio103check_thread_break) return;
		if( (value = gpio_get_value(103) ) != 0 ) goto NEXT_STEP;
		mdelay(25);
		if(gpio103check_thread_break) return;
		if( (value = gpio_get_value(103) ) != 0 ) goto NEXT_STEP;

		//gpio103 'L' --> enable headsethook
		while(!gpio103check_thread_break){
			if(gpio103check_thread_break) { headsetkey_released(); return; }
			mdelay(40);
			pre_value = value;
			value = gpio_get_value(103);
			if(pre_value != value){
				if(value == 0 && headset_key_state){
					for(h=0;h<5;h++){
						if(gpio103check_thread_break) { headsetkey_released(); return; }
						mdelay(40);
						pre_value = value;
						value = gpio_get_value(103);
						if(value != 0) break;
					}
					if(h == 5) headsetkey_released();
				}else if(value != 0 && !headset_key_state){
					headsetkey_pressed();
				}
			}
		}
		return;
NEXT_STEP:
		//1 sec wait.
		for(h=0;h<10;h++){
			mdelay(100);
			if(gpio103check_thread_break) return;
		}
	}
}

// analog mic control -->
void analog_headset_switch_enable(int en)
{
	// nop check
	if(amic_for_hswitch == true && en == 1)  return;
	if(amic_for_hswitch == false && en == 0) return;

	headsetkey_released();

	// mic bias control
	if (en == 0) amic_for_hswitch = false;
	else amic_for_hswitch = true;
	_audio_analog_mic_ctrl();

	mdelay(20);

	// gpio control
	if (en == 0) gpio_tlmm_config(GPIO_CFG( 103, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE); // pull_down
	else         gpio_tlmm_config(GPIO_CFG( 103, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);   // pull_up

	//headsethook enable thread start.
	if (en == 0){
		gpio103check_thread_break = true;
	} else {
		struct task_struct *task;
		task = kthread_run(gpio103check_thread, NULL, "analogaudio");
		if (IS_ERR(task)) {
			printk(KERN_ERR "unable to create kernel thread: %ld\n", PTR_ERR(task));
		}
	}
}
// <-- analog mic control

static struct q6audio_analog_ops ops = {
	.init = analog_init,
	.speaker_enable = analog_speaker_enable,
	.headset_enable = analog_headset_enable,
	.int_mic_enable = analog_mic_enable,
	.ext_mic_enable = analog_mic_enable,
	.headset_switch_enable = analog_headset_switch_enable, // analog mic control
};

static int __init init(void)
{
	q6audio_register_analog_ops(&ops);
	return 0;
}

device_initcall(init);
