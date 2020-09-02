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
 * This code is based on arch/arm/mach-msm/rpc_server_time_remote.c.
 * The original copyright and notice are described below.
 */
/* arch/arm/mach-msm/rpc_server_time_remote.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2010 Code Aurora Forum. All rights reserved.
 * Author: Iliyan Malchev <ibm@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <mach/msm_rpcrouter.h>
#include "rpc_server_time_remote.h"
#include <linux/rtc.h>
#include <linux/fs.h>
#include <linux/android_alarm.h>
#include <linux/syscalls.h>

/* time_remote_mtoa server definitions. */

#define TIME_REMOTE_MTOA_PROG 0x3000005d
#define TIME_REMOTE_MTOA_VERS_OLD 0
#define TIME_REMOTE_MTOA_VERS 0x9202a8e4
#define TIME_REMOTE_MTOA_VERS_COMP 0x00010002
#define RPC_TIME_REMOTE_MTOA_NULL   0
#define RPC_TIME_TOD_SET_APPS_BASES 2
#define RPC_TIME_GET_APPS_USER_TIME 3

struct rpc_time_tod_set_apps_bases_args {
	uint32_t tick;
	uint64_t stamp;
};

static int read_rtc0_time(struct msm_rpc_server *server,
		   struct rpc_request_hdr *req,
		   unsigned len)
{
	int err;
	unsigned long tm_sec;
	uint32_t size = 0;
	void *reply;
	uint32_t output_valid;
	uint32_t rpc_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
	struct rtc_time tm;
	struct rtc_device *rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);

	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		goto send_reply;
	}

	err = rtc_read_time(rtc, &tm);
	if (err) {
		pr_err("%s: Error reading rtc device (%s) : %d\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE, err);
		goto close_dev;
	}

	err = rtc_valid_tm(&tm);
	if (err) {
		pr_err("%s: Invalid RTC time (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		goto close_dev;
	}

	rtc_tm_to_time(&tm, &tm_sec);
	rpc_status = RPC_ACCEPTSTAT_SUCCESS;

close_dev:
	rtc_class_close(rtc);

send_reply:
	reply = msm_rpc_server_start_accepted_reply(server, req->xid,
						    rpc_status);
	if (rpc_status == RPC_ACCEPTSTAT_SUCCESS) {
		output_valid = *((uint32_t *)(req + 1));
		*(uint32_t *)reply = output_valid;
		size = sizeof(uint32_t);
		if (be32_to_cpu(output_valid)) {
			reply += sizeof(uint32_t);
			*(uint32_t *)reply = cpu_to_be32(tm_sec);
			size += sizeof(uint32_t);
		}
	}
	err = msm_rpc_server_send_accepted_reply(server, size);
	if (err)
		pr_err("%s: send accepted reply failed: %d\n", __func__, err);

	return 1;
}

static unsigned int rpc_remote_autosettime;
static unsigned int rpc_remote_lastsectime;

static int rpc_android_alarm_set_rtc(void)
{
	int err;
	struct rtc_time tm;
	struct rtc_device *rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);

	if (rtc == NULL) {
		pr_err("%s: Unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		goto return_fun;
	}

	err = rtc_read_time(rtc, &tm);
	if (err) {
		pr_err("rpc_android_alarm_set_rtc: "
			"Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, err);
		goto close_dev;
	}

	err = rtc_valid_tm(&tm);
	if (err) {
		pr_err("rpc_android_alarm_set_rtc: "
			"Invalid RTC time (%s)\n",
			CONFIG_RTC_HCTOSYS_DEVICE);
		goto close_dev;
	}
	else {
		struct timespec tv;
		unsigned int tm_delta;
		long res = -1;

		tv.tv_nsec = NSEC_PER_SEC >> 1;
		rtc_tm_to_time(&tm, &tv.tv_sec);
		tm_delta = (unsigned int) tv.tv_sec;

		tm_delta = (tm_delta >= rpc_remote_lastsectime) ?
			(tm_delta - rpc_remote_lastsectime) :
			(rpc_remote_lastsectime - tm_delta);

		pr_debug("rpc_remote_autosettime=%d, tm_delta=%d\n",
			rpc_remote_autosettime, tm_delta);

		if ((rpc_remote_autosettime == 0) || (tm_delta >= 60)) {
			long fd = sys_open("/dev/alarm", O_RDWR, 0);
			if(fd < 0) {
				pr_err("rpc_android_alarm_set_rtc: "
					"Unable to open alarm driver\n");
			}
			else {
				pr_debug("rpc_android_alarm_set_rtc: "
					"ANDROID_ALARM_SET_RTC\n");

				res = sys_ioctl((unsigned int)fd, 
					ANDROID_ALARM_SET_RTC, (unsigned long)&tv);
				sys_close((unsigned int)fd);
			}
		}
		else {
			pr_info("rpc_android_alarm_set_rtc: skipped to do_settimeofday\n");
			res = 0;
		}

		if(res < 0) {
	        pr_err("rpc_android_alarm_set_rtc: "
				"Failed(%ld) to set system clock to "
				"%d-%02d-%02d %02d:%02d:%02d UTC (%u)\n",
				res, tm.tm_year + 1900, tm.tm_mon + 1,
				tm.tm_mday, tm.tm_hour, tm.tm_min,
				tm.tm_sec, (unsigned int) tv.tv_sec);
		}
		else {
			rpc_remote_autosettime = 1;
			rpc_remote_lastsectime = (unsigned int) tv.tv_sec;

			pr_info("rpc_android_alarm_set_rtc: "
				"Succeeded to set system clock to "
				"%d-%02d-%02d %02d:%02d:%02d UTC (%u)\n",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec,
				(unsigned int) tv.tv_sec);
		}
	}

close_dev:
	rtc_class_close(rtc);

return_fun:
	return 0;
}

static int handle_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	switch (req->procedure) {
	case RPC_TIME_REMOTE_MTOA_NULL:
		return 0;

	case RPC_TIME_TOD_SET_APPS_BASES: {
		struct rpc_time_tod_set_apps_bases_args *args;
		struct kstat temp_state;

		if (vfs_stat("/data/data/com.android.settings/files"
				"/autotime.disabled", &temp_state) == 0) {
			rpc_remote_autosettime = 0;
			printk(KERN_INFO "RPC_TIME_TOD_SET_APPS_BASES:\n"
				"\tautotime.disabled\n");
			return 0;
		}

		args = (struct rpc_time_tod_set_apps_bases_args *)(req + 1);
		args->tick = be32_to_cpu(args->tick);
		args->stamp = be64_to_cpu(args->stamp);
		printk(KERN_INFO "RPC_TIME_TOD_SET_APPS_BASES:\n"
		       "\ttick = %d\n"
		       "\tstamp = %lld\n",
		       args->tick, args->stamp);
		rpc_android_alarm_set_rtc();
		return 0;
	}

	case RPC_TIME_GET_APPS_USER_TIME:
		return read_rtc0_time(server, req, len);

	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server rpc_server[] = {
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS_OLD,
		.rpc_call = handle_rpc_call,
	},
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS,
		.rpc_call = handle_rpc_call,
	},
	{
		.prog = TIME_REMOTE_MTOA_PROG,
		.vers = TIME_REMOTE_MTOA_VERS_COMP,
		.rpc_call = handle_rpc_call,
	},
};

static int __init rpc_server_init(void)
{
	/* Dual server registration to support backwards compatibility vers */
	int ret;

	rpc_remote_autosettime = 0;
	rpc_remote_lastsectime = 0;

	ret = msm_rpc_create_server(&rpc_server[2]);
	if (ret < 0)
		return ret;
	ret = msm_rpc_create_server(&rpc_server[1]);
	if (ret < 0)
		return ret;
	printk(KERN_ERR "Using very old AMSS modem firmware.\n");
	return msm_rpc_create_server(&rpc_server[0]);
}


module_init(rpc_server_init);
