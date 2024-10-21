/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-metadata.h
 *
 * Qualcomm MSM Camera Subsystem - ISP Metadata Module
 *
 * Copyright (c) 2022, Arcturus Industries.
 */
#ifndef QC_MSM_CAMSS_METADATA_H
#define QC_MSM_CAMSS_METADATA_H
#include <linux/clk.h>
#include <linux/spinlock_types.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "camss-video.h"

struct metadata_output {
    u8 wm_num;
    u8 wm_id[3];

};

struct metadata_device {
    struct camss *camss;
    u8 id;
    struct v4l2_subdev subdev;
    struct camss_video metadata_frame_out;
    struct metadata_output output;
    struct v4l2_ctrl_handler ctrls;
    u32 fmt;
    void __iomem *base;
    struct camss_video video_out;
    struct media_pad pad;
    struct list_head metadata;
    spinlock_t lock;
    

};

struct resources;

/*
 * metadata_format_get_fmt_entry - Find metadata_format_format entry with matching format code
 * @formats: Array of format metadata_format_format entries
 * @nformats: Length of @nformats array
 * @code: Desired format code
 *
 * Return formats[0] on failure to find code
 */
const struct metadata_format *metadata_get_fmt_entry(const struct metadata_format *formats,
					     unsigned int nformats,
					     u32 code);

//TODO: Use or refactor out resources
int msm_metadata_subdev_init(struct camss *camss, struct metadata_device *metadata, const struct resources *res, u8 id);

int msm_metadata_register_entity(struct metadata_device *metadata_format,
			     struct v4l2_device *v4l2_dev);

void msm_metadata_unregister_entity(struct metadata_device *metadata_format);

void msm_metadata_format_get_metadata_format_id(struct media_entity *entity, u8 *id);

//send the metadata
void metadata_send(struct metadata_device *metadata, 
                   u64 wm_done_timestamp,
                   u64 sof_isp_timestamp,
                   u64 eof_isp_timestamp,
                   u64 sof_isr_timestamp,
                   u64 eof_isr_timestamp,
                   u32 sequence,
                   void* sensorDataBuffer,
                   void* frameheaderData,
                   void* imgHistFrameheaderData,
                   void* imgHistData,
                   void* bayerHistFrameheaderData,
                   void* bayerHistData);

#endif