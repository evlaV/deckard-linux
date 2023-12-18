/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _BUFFERS_H_
#define _BUFFERS_H_

#include <linux/types.h>

u32 video_raw_buffer_size(u32 colorformat,
			  u32 pix_width,
			  u32 pix_height);

#endif
