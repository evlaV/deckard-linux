/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-video.h
 *
 * Qualcomm MSM Camera Subsystem - V4L2 device node
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 */
#ifndef QC_MSM_CAMSS_VIDEO_H
#define QC_MSM_CAMSS_VIDEO_H

#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mediabus.h>
#include <media/videobuf2-v4l2.h>

// Private ioctls to access CSID/VFE registers
#define VIDIOC_PRIV_S_REGS _IOW('V', BASE_VIDIOC_PRIVATE + 1, struct csid_vfe_private_registers_config)
#define VIDIOC_PRIV_G_REGS _IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct csid_vfe_private_registers_config)
#define VIDIOC_PRIV_S_EXTRA_OUTPUT_PLANE_CFG _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct camss_extra_output_plane_cfg)
// TODO: add layout config ioctl (to set plane stride and offsets)

enum camss_extra_output_type
{
	CAMSS_EXTRA_OUTPUT_TYPE_META,
	CAMSS_EXTRA_OUTPUT_TYPE_STATS_IHIST,
	CAMSS_EXTRA_OUTPUT_TYPE_STATS_BHIST,
	CAMSS_EXTRA_OUTPUT_TYPE_DS4,
	CAMSS_EXTRA_OUTPUT_TYPE_DS16,
	CAMSS_EXTRA_OUTPUT_TYPE_MAX
};

struct camss_extra_output_plane_cfg
{
	u32 plane_size;
	// For the regular image output frameheader
	u32 frameheader_en;
	u32 frameheader_offset;
	// To enable additional outputs (downscaled output, statistics, frameheaders) streamed into the plane
	u32 extra_output_en[CAMSS_EXTRA_OUTPUT_TYPE_MAX];
	u32 extra_output_offset[CAMSS_EXTRA_OUTPUT_TYPE_MAX];
	u32 extra_output_frameheader_en[CAMSS_EXTRA_OUTPUT_TYPE_MAX];
	u32 extra_output_frameheader_offset[CAMSS_EXTRA_OUTPUT_TYPE_MAX];
};

// Minimal metadata structure for CAMSS_EXTRA_OUTPUT_TYPE_META that only includes SOF/EOF info and sensor specific metadata
struct camss_metadata {
	u64 sof_timestamp;
	u64 eof_timestamp;
	u64 sof_isr_timestamp;
	u64 eof_isr_timestamp;
	s64 isp_to_kernel_raw_ts_offset;
	uint8_t sensorMetaDataBuffer[1208];
};

struct camss_buffer {
	struct vb2_v4l2_buffer vb;
	dma_addr_t addr[3];
	struct list_head queue;

	// Keep track of the composite group DONE events received for this buffer
	uint32_t comp_done_status;
	// Extra buffer (used for metadata/frameheaders/statistics streaming)
	void* extrabuffer_vaddr; 
	dma_addr_t extrabuffer_dma_addr; 
	uint32_t extrabuffer_size;

	// Extra output plane (replaces the driver-local extra buffer above)
	dma_addr_t extra_output_plane_dma_addr;
	struct sg_table *extra_output_plane_sgt; 
};

struct camss_video;

struct camss_video_ops {
	int (*queue_buffer)(struct camss_video *vid, struct camss_buffer *buf);
	int (*flush_buffers)(struct camss_video *vid,
			     enum vb2_buffer_state state);
};

struct camss_video {
	struct camss *camss;
	struct vb2_queue vb2_q;
	struct video_device vdev;
	struct media_pad pad;
	struct v4l2_format active_fmt;
	enum v4l2_buf_type type;
	struct media_pipeline pipe;
	const struct camss_video_ops *ops;
	struct mutex lock;
	struct mutex q_lock;
	unsigned int bpl_alignment;
	unsigned int line_based;
	const struct camss_format_info *formats;
	unsigned int nformats;

	// Size requirement for the extra buffer or 0 when no buffer needs to be created
	u32 required_extrabuffer_size;

	// Extra output plane (replaces the driver-local extra buffer above)
	bool has_extra_output_plane_cfg;
	struct camss_extra_output_plane_cfg extra_output_plane_cfg;
	u32 extra_output_addr_idx;
};

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name);

void msm_video_unregister(struct camss_video *video);

#endif /* QC_MSM_CAMSS_VIDEO_H */
