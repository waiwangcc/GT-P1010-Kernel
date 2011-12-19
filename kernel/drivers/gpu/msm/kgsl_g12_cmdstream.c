/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/msm_kgsl.h>
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_cmdstream.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_cmdstream.h"

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_log.h"

#include "g12_reg.h"

int kgsl_g12_cmdstream_check_timestamp(struct kgsl_device *device,
					unsigned int timestamp)
{
	int ts_diff;

	ts_diff = device->timestamp - timestamp;

	return (ts_diff >= 0) || (ts_diff < -20000);
}

static int room_in_rb(struct kgsl_device *device)
{
	int ts_diff;

	ts_diff = device->current_timestamp - device->timestamp;

	return ts_diff < KGSL_G12_PACKET_COUNT;
}

static inline unsigned int rb_offset(unsigned int index)
{
	return index*sizeof(unsigned int)*KGSL_G12_PACKET_SIZE;
}

static void addcmd(struct kgsl_g12_z1xx *z1xx, unsigned int index,
			unsigned int cmd, unsigned int nextcnt)
{
	unsigned int *p = z1xx->cmdbufdesc.hostptr + rb_offset(index);

	*p++ = 0x7C000176;
	*p++ = 5;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = 0x7C000275;
	*p++ = cmd;
	*p++ = 0x1000 | nextcnt;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
}

int
kgsl_g12_cmdstream_issueibcmds(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable,
			int drawctxt_index,
			uint32_t ibaddr,
			int sizedwords,
			int *timestamp,
			unsigned int ctrl)
{
	unsigned int result = 0;
	unsigned int ofs        = PACKETSIZE_STATESTREAM * sizeof(unsigned int);
	unsigned int cnt        = 5;
	unsigned int nextaddr   = 0;
	unsigned int index	= 0;
	unsigned int nextcnt    = 0x9000 | 5;
	struct kgsl_memdesc tmp = {0};
	unsigned int cmd;

	cmd = ibaddr;

	tmp.hostptr = (void *)*timestamp;

	KGSL_CMD_INFO("ctxt %d ibaddr 0x%08x sizedwords %d",
			drawctxt_index, ibaddr, sizedwords);
	/* context switch */
	if (drawctxt_index != (int)g_z1xx.prevctx) {
		kgsl_mmu_setstate(device, pagetable);
		cnt = PACKETSIZE_STATESTREAM;
		ofs = 0;
	} else {
		kgsl_setstate(device, device->mmu.tlb_flags);
	}

	result = wait_event_interruptible_timeout(device->wait_timestamp_wq,
				  room_in_rb(device),
				  msecs_to_jiffies(KGSL_TIMEOUT_DEFAULT));
	if (result < 0) {
		KGSL_CMD_ERR("failed waiting for ringbuffer. result %d",
			     result);
		goto error;
	}
	result = 0;

	index = device->current_timestamp % KGSL_G12_PACKET_COUNT;
	device->current_timestamp++;
	*timestamp = device->current_timestamp;

	g_z1xx.prevctx = drawctxt_index;

	addcmd(&g_z1xx, index, cmd + ofs, cnt);

	nextaddr = g_z1xx.cmdbufdesc.physaddr
		 + rb_offset((index + 1) % KGSL_G12_PACKET_COUNT);

	tmp.hostptr = (void *)(tmp.hostptr +
			(sizedwords * sizeof(unsigned int)));
	tmp.size = 12;

	kgsl_sharedmem_writel(&tmp, 4, nextaddr);
	kgsl_sharedmem_writel(&tmp, 8, nextcnt);

	kgsl_g12_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, 1);
	kgsl_g12_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, 0);
error:
	return result;
}

