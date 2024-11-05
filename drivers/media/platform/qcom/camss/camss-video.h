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

// Extended configuration ioctl
#define VIDIOC_PRIV_S_EXT_CFG _IOW('V', BASE_VIDIOC_PRIVATE + 1, struct camss_ext_config)
#define VIDIOC_PRIV_S_REGS _IOW('V', BASE_VIDIOC_PRIVATE + 2, struct csid_vfe_private_registers_config)
#define VIDIOC_PRIV_G_REGS _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct csid_vfe_private_registers_config)

// TODO: need ioctl to request required output size for a defined output?
#define CAMSS_VIDEO_MAX_NUM_PLANES 32

// TODO: extend with additional RDIs and IPP statistics output like AWB chromacity statistics
enum camss_ext_vfe_output_type
{
	CAMSS_EXT_VFE_OUTPUT_TYPE_RDI0 = 0,
	CAMSS_EXT_VFE_OUTPUT_TYPE_RDI1,
	CAMSS_EXT_VFE_OUTPUT_TYPE_RDI2,
	// (only RDIs that are available on all VFEs are listed above)
	CAMSS_EXT_VFE_OUTPUT_TYPE_META, // written by kernel directly, stores sof/eof timestamps and sensor metadata when available
	CAMSS_EXT_VFE_OUTPUT_TYPE_FULL_Y,
	CAMSS_EXT_VFE_OUTPUT_TYPE_FULL_C,
	CAMSS_EXT_VFE_OUTPUT_TYPE_DS4,
	CAMSS_EXT_VFE_OUTPUT_TYPE_DS16,
	CAMSS_EXT_VFE_OUTPUT_TYPE_STATS_IHIST,
	CAMSS_EXT_VFE_OUTPUT_TYPE_STATS_BHIST,
	CAMSS_EXT_VFE_OUTPUT_TYPE_MAX
};

struct camss_ext_vfe_output_config
{
	u32 en;
	u32 plane_idx;
	u32 offset; // offset into the plane memory
	u32 width;
	u32 height;
	u32 stride;
	// optional frameheader (not supported on output types META, DS4 and DS16)
	u32 frameheader_en;
	u32 frameheader_plane_idx;
	u32 frameheader_offset;

    // TODO: add additional fields for UBWC streaming (metadata plane output and configuration)
};

enum camss_ext_csid_path_type
{
	CAMSS_EXT_CSID_PATH_RDI0 = 0,
	CAMSS_EXT_CSID_PATH_RDI1, // written by kernel directly, stores sof/eof timestamps and sensor metadata when available
	CAMSS_EXT_CSID_PATH_RDI2,
	CAMSS_EXT_CSID_PATH_IPP0,
	// only PPP0 and RDI4 are missing but we never used them (non-lite CSID only)
	CAMSS_EXT_CSID_PATH_TYPE_MAX
};

struct camss_ext_csid_path_config
{
	u32 en;
	u32 vc;
	u32 dt_id;
	// only allow vc to differ in the multi_vcdt config
	u32 multi_vcdt_en;
	u32 multi_vcdt_vc;
	u32 trigger_post_sof_work;
};

struct camss_ext_config
{
	u32 plane_count;
	u32 plane_sizes[CAMSS_VIDEO_MAX_NUM_PLANES];
	struct camss_ext_vfe_output_config vfe_output_configs[CAMSS_EXT_VFE_OUTPUT_TYPE_MAX];
	struct camss_ext_csid_path_config csid_path_configs[CAMSS_EXT_CSID_PATH_TYPE_MAX];
};

// Minimal metadata structure for CAMSS_EXT_VFE_OUTPUT_TYPE_META that only includes SOF/EOF info and sensor specific metadata
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

    // -- for extended configuration --
	dma_addr_t ext_addr[CAMSS_EXT_VFE_OUTPUT_TYPE_MAX];
	dma_addr_t ext_fh_addr[CAMSS_EXT_VFE_OUTPUT_TYPE_MAX];
    struct sg_table *ext_metadata_sgt; // needed for cpu access to the vb2 plane used by the metadata output
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

    // -- for extended configuration --
	u32 has_extended_cfg;
	struct camss_ext_config extended_cfg;
};

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name);

void msm_video_unregister(struct camss_video *video);

#endif /* QC_MSM_CAMSS_VIDEO_H */
