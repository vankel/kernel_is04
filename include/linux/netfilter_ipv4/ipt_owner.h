/*
 * Certain software is contributed or developed by 
 * FUJITSU TOSHIBA MOBILE COMMUNICATIONS LIMITED.
 *
 * COPYRIGHT(C) FUJITSU TOSHIBA MOBILE COMMUNICATIONS LIMITED 2011
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
 * This code is based on ipt_owner.h.
 * The original copyright and notice are described below.
 */
#ifndef _IPT_OWNER_H
#define _IPT_OWNER_H

/* match and invert flags */
#define IPT_OWNER_UID	0x01
#define IPT_OWNER_GID	0x02
#define IPT_OWNER_PID	0x04
#define IPT_OWNER_SID	0x08
#define IPT_OWNER_COMM	0x10

struct ipt_owner_info {
    uid_t uid;
    gid_t gid;
    pid_t pid;
    pid_t sid;
    char comm[16];
    u_int8_t match, invert;	/* flags */
};

#endif /*_IPT_OWNER_H*/
