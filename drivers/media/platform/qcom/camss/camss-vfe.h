/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-vfe.h
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 */
#ifndef QC_MSM_CAMSS_VFE_H
#define QC_MSM_CAMSS_VFE_H

#include <linux/clk.h>
#include <linux/spinlock_types.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "camss-video.h"
#include "camss-vfe-gen1.h"

#define MSM_VFE_PAD_SINK 0
#define MSM_VFE_PAD_SRC 1
#define MSM_VFE_PAD_SRC_DS4 2
#define MSM_VFE_PAD_SRC_DS16 3
#define MSM_VFE_PAD_SRC_STATS 4
#define MSM_VFE_PAD_SINK_PARAMS 5
#define MSM_VFE_PAD_NUM_RDI 2
#define MSM_VFE_PAD_NUM_PIX 6
#define MSM_VFE_MAX_PADS MSM_VFE_PAD_NUM_PIX
#define MSM_VFE_NUM_OUTPUTS (MSM_VFE_MAX_PADS - 1)	/* every pad but the sink */

#define MSM_VFE_IMAGE_MASTERS_NUM 7
#define MSM_VFE_COMPOSITE_IRQ_NUM 4

/* VFE halt timeout */
#define VFE_HALT_TIMEOUT_MS 100
/* Frame drop value. VAL + UPDATES - 1 should not exceed 31 */
#define VFE_FRAME_DROP_VAL 30

#define vfe_line_array(ptr_line)	\
	((const struct vfe_line (*)[]) &(ptr_line)[-(ptr_line)->id])

#define to_vfe(ptr_line)	\
	container_of(vfe_line_array(ptr_line), struct vfe_device, line)

enum vfe_output_state {
	VFE_OUTPUT_OFF,
	VFE_OUTPUT_RESERVED,
	VFE_OUTPUT_SINGLE,
	VFE_OUTPUT_CONTINUOUS,
	VFE_OUTPUT_IDLE,
	VFE_OUTPUT_STOPPING,
	VFE_OUTPUT_ON,
};

enum vfe_line_id {
	VFE_LINE_NONE = -1,
	VFE_LINE_RDI0 = 0,
	VFE_LINE_RDI1 = 1,
	VFE_LINE_RDI2 = 2,
	VFE_LINE_RDI3 = 3,	/* IFE lite */
	VFE_LINE_PIX = 3,	/* full IFE, when line->is_pix */
	VFE_LINE_NUM_MAX = 4
};

enum vfe_packer_format_v3 {
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_128 = 0,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_8,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_8_ODD_EVEN,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_8_LSB_MSB_10,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_8_LSB_MSB_10_ODD_EVEN,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_16_10BPP,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_16_12BPP,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_16_14BPP,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_16_16BPP,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_32,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN_64,
	VFE_BUS_WM_PACKER_FMT_V3_TP_10,
	VFE_BUS_WM_PACKER_FMT_V3_MIPI10,
	VFE_BUS_WM_PACKER_FMT_V3_MIPI12,
	VFE_BUS_WM_PACKER_FMT_V3_MIPI14,
	VFE_BUS_WM_PACKER_FMT_V3_MIPI20,
	VFE_BUS_WM_PACKER_FMT_V3_PLAIN32_20BPP,
	VFE_BUS_WM_PACKER_FMT_V3_MAX,
};

enum vfe_v3_comp_group {
	VFE_V3_COMP_GRP_0,
	VFE_V3_COMP_GRP_1,
	VFE_V3_COMP_GRP_2,
	VFE_V3_COMP_GRP_3,
	VFE_V3_COMP_GRP_4,
	VFE_V3_COMP_GRP_5,
	VFE_V3_COMP_GRP_6,
	VFE_V3_COMP_GRP_7,
	VFE_V3_COMP_GRP_8,
	VFE_V3_COMP_GRP_9,
	VFE_V3_COMP_GRP_10,
	VFE_V3_COMP_GRP_11,
	VFE_V3_COMP_GRP_12,
	VFE_V3_COMP_GRP_13,
	VFE_V3_COMP_GRP_14,
	VFE_V3_COMP_GRP_15,
	VFE_V3_COMP_GRP_16,
	VFE_V3_COMP_GRP_MAX,
};

struct vfe_wm {
	u8 bus_client;
	u8 plane;
};

#define VFE_MAX_WM_PER_OUTPUT 3

struct vfe_output {
	u8 pad;			/* index into line->pads[] this output serves */
	u8 comp_group;
	u8 wm_num;
	struct vfe_wm wm[VFE_MAX_WM_PER_OUTPUT];

	struct camss_buffer *buf[2];
	struct camss_buffer *last_buffer;
	struct list_head pending_bufs;

	unsigned int drop_update_idx;

	union {
		struct {
			int active_buf;
			int wait_sof;
		} gen1;
		struct {
			int active_num;
		} gen2;
	};
	enum vfe_output_state state;
	unsigned int sequence;

	int wait_reg_update;
	struct completion sof;
	struct completion reg_update;

	struct vfe_line *line;
	struct camss_video video_out;

	/*
	 * Kernel buffer the write masters fill, for outputs the CPU copies
	 * to userspace at completion (statistics)
	 */
	void *dma_vaddr;
	dma_addr_t dma_addr;
	size_t dma_size;
};

struct vfe_line {
	enum vfe_line_id id;
	bool is_pix;
	struct v4l2_subdev subdev;
	struct media_pad pads[MSM_VFE_MAX_PADS];
	struct v4l2_mbus_framefmt fmt[MSM_VFE_MAX_PADS];
	u32 num_pads;
	struct v4l2_rect compose;
	struct v4l2_rect crop;
	struct vfe_output output[MSM_VFE_NUM_OUTPUTS];
	u32 num_outputs;
	const struct camss_format_info *formats;
	unsigned int nformats;
};

struct vfe_device;
struct camss_buf_done_info;

struct vfe_hw_ops {
	void (*enable_irq)(struct vfe_device *vfe);
	void (*global_reset)(struct vfe_device *vfe);
	u32 (*hw_version)(struct vfe_device *vfe);
	irqreturn_t (*isr)(int irq, void *dev);
	void (*isr_read)(struct vfe_device *vfe, u32 *value0, u32 *value1);
	void (*pm_domain_off)(struct vfe_device *vfe);
	int (*pm_domain_on)(struct vfe_device *vfe);
	void (*reg_update)(struct vfe_device *vfe, enum vfe_line_id line_id);
	void (*reg_update_clear)(struct vfe_device *vfe,
				 enum vfe_line_id line_id);
	void (*subdev_init)(struct device *dev, struct vfe_device *vfe);
	int (*vfe_disable)(struct vfe_line *line);
	int (*vfe_enable)(struct vfe_line *line);
	int (*vfe_halt)(struct vfe_device *vfe);
	void (*violation_read)(struct vfe_device *vfe);

	/* RDI based API - deprecated */
	void (*vfe_wm_start)(struct vfe_device *vfe, u8 wm,
			     struct vfe_line *line);
	void (*vfe_wm_stop)(struct vfe_device *vfe, u8 wm,
			    struct vfe_line *line);
	void (*vfe_buf_done)(struct vfe_device *vfe, int port_id,
			     struct camss_buf_done_info *buf_done_info);
	void (*vfe_wm_update)(struct vfe_device *vfe, u8 wm, struct camss_buffer *buffer,
			      struct vfe_line *line);

	/*
	 * Optional: program line-level hardware once the line's outputs are
	 * reserved, outside output_lock. Runs before vfe_wm_start() and the
	 * reg update that latches it.
	 */
	void (*vfe_line_prepare)(struct vfe_line *line);

	/* Output based API - new and shiny */
	void (*vfe_output_start)(struct vfe_device *vfe, struct vfe_output *output);
	void (*vfe_output_stop)(struct vfe_device *vfe, struct vfe_output *output);
	void (*vfe_output_buf_done)(struct vfe_device *vfe, struct vfe_output *output);
	void (*vfe_output_update)(struct vfe_device *vfe, struct vfe_output *output,
				  struct camss_buffer *buf);

};

struct vfe_isr_ops {
	void (*reset_ack)(struct vfe_device *vfe);
	void (*halt_ack)(struct vfe_device *vfe);
	void (*reg_update)(struct vfe_device *vfe, enum vfe_line_id line_id);
	void (*sof)(struct vfe_device *vfe, enum vfe_line_id line_id);
	void (*comp_done)(struct vfe_device *vfe, u8 comp);
	void (*wm_done)(struct vfe_device *vfe, u8 wm);
};

struct vfe_subdev_resources {
	bool is_lite;
	u8 line_num;
	bool has_pd;
	char *pd_name;
	bool has_vbif;
	char *vbif_name;
	const struct vfe_hw_ops *hw_ops;
	const struct camss_formats *formats_rdi;
	const struct camss_formats *formats_pix;
};

struct vfe_device {
	struct camss *camss;
	u8 id;
	void __iomem *base;
	void __iomem *vbif_base;
	u32 irq;
	char irq_name[30];
	struct camss_clock *clock;
	int nclocks;
	struct completion reset_complete;
	struct completion halt_complete;
	struct mutex power_lock;
	int power_count;
	struct mutex stream_lock;
	int stream_count;
	spinlock_t output_lock;
	enum vfe_line_id wm_output_map[MSM_VFE_IMAGE_MASTERS_NUM];
	struct vfe_line line[VFE_LINE_NUM_MAX];
	u32 reg_update;
	u8 was_streaming;
	const struct vfe_subdev_resources *res;
	const struct vfe_hw_ops_gen1 *ops_gen1;
	struct vfe_isr_ops isr_ops;
	struct camss_video_ops video_ops;
	struct device *genpd;
	struct device_link *genpd_link;
};

struct camss_subdev_resources;

int msm_vfe_subdev_init(struct camss *camss, struct vfe_device *vfe,
			const struct camss_subdev_resources *res, u8 id);

void msm_vfe_genpd_cleanup(struct vfe_device *vfe);

int msm_vfe_register_entities(struct vfe_device *vfe,
			      struct v4l2_device *v4l2_dev);

void msm_vfe_unregister_entities(struct vfe_device *vfe);

/*
 * vfe_buf_add_pending - Add output buffer to list of pending
 * @output: VFE output
 * @buffer: Video buffer
 */
void vfe_buf_add_pending(struct vfe_output *output, struct camss_buffer *buffer);

struct camss_buffer *vfe_buf_get_pending(struct vfe_output *output);

int vfe_flush_buffers(struct camss_video *vid, enum vb2_buffer_state state);

/*
 * vfe_isr_comp_done - Process composite image done interrupt
 * @vfe: VFE Device
 * @comp: Composite image id
 */
void vfe_isr_comp_done(struct vfe_device *vfe, u8 comp);

void vfe_isr_reset_ack(struct vfe_device *vfe);
int vfe_put_output(struct vfe_line *line);
int vfe_release_wm(struct vfe_device *vfe, u8 wm);
int vfe_reserve_wm(struct vfe_device *vfe, enum vfe_line_id line_id);

/*
 * vfe_reset - Trigger reset on VFE module and wait to complete
 * @vfe: VFE device
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_reset(struct vfe_device *vfe);

/*
 * vfe_disable - Disable streaming on VFE line
 * @line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_disable(struct vfe_line *line);

/*
 * vfe_pm_domain_off - Disable power domains specific to this VFE.
 * @vfe: VFE Device
 */
void vfe_pm_domain_off(struct vfe_device *vfe);

/*
 * vfe_pm_domain_on - Enable power domains specific to this VFE.
 * @vfe: VFE Device
 */
int vfe_pm_domain_on(struct vfe_device *vfe);

extern const struct camss_formats vfe_formats_rdi_8x16;
extern const struct camss_formats vfe_formats_pix_8x16;
extern const struct camss_formats vfe_formats_rdi_8x96;
extern const struct camss_formats vfe_formats_pix_8x96;
extern const struct camss_formats vfe_formats_rdi_845;
extern const struct camss_formats vfe_formats_pix_845;

extern const struct vfe_hw_ops vfe_ops_4_1;
extern const struct vfe_hw_ops vfe_ops_4_7;
extern const struct vfe_hw_ops vfe_ops_4_8;
extern const struct vfe_hw_ops vfe_ops_170;
extern const struct vfe_hw_ops vfe_ops_340;
extern const struct vfe_hw_ops vfe_ops_480;
extern const struct vfe_hw_ops vfe_ops_680;
extern const struct vfe_hw_ops vfe_ops_690;
extern const struct vfe_hw_ops vfe_ops_780;

int vfe_get(struct vfe_device *vfe);
void vfe_put(struct vfe_device *vfe);

/*
 * vfe_is_lite - Return if VFE is VFE lite.
 * @vfe: VFE Device
 *
 * Some VFE lites have a different register layout.
 *
 * Return whether VFE is VFE lite
 */
bool vfe_is_lite(struct vfe_device *vfe);

/*
 * vfe_hw_version - Process write master done interrupt
 * @vfe: VFE Device
 *
 * Return vfe hw version
 */
u32 vfe_hw_version(struct vfe_device *vfe);
/*
 * vfe_enable - Enable streaming on VFE line
 * @line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_enable_v2(struct vfe_line *line);

/*
 * vfe_buf_done - Process write master done interrupt
 * @vfe: VFE Device
 * @wm: Write master id
 */
void vfe_buf_done(struct vfe_device *vfe, int wm, struct camss_buf_done_info *buf_done_info);

/*
 * vfe_get_output_v2 - Get vfe output line
 * line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_get_output_v2(struct vfe_line *line);

/*
 * vfe_enable_output_v2 - Enable vfe output line
 * line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_enable_output_v2(struct vfe_line *line);

/*
 * vfe_queue_buffer_v2 - Add empty buffer
 * @vid: Video device structure
 * @buf: Buffer to be enqueued
 *
 * Add an empty buffer - depending on the current number of buffers it will be
 * put in pending buffer queue or directly given to the hardware to be filled.
 *
 * Return 0 on success or a negative error code otherwise
 */
int vfe_queue_buffer_v2(struct camss_video *vid,
			struct camss_buffer *buf);

/*
 * vfe_bus_v3_packer_fmt
 *
 * @v4l2_fmt: v4l2 format
 * @plane: plane for the given format
 * @align_shift: bit position of the PLAIN16 LSB alignment flag
 *
 * Return: the bus packer format for the register
 */
u32 vfe_bus_v3_packer_fmt(u32 v4l2_fmt, u8 plane, u8 align_shift);

void vfe_add_output(struct vfe_output *output, u8 pad, u8 comp_group,
		    unsigned int wm_num, const u8 *bus_client);

void vfe_add_rdi(struct vfe_output *output, u8 bus_client, u8 comp_group);

void vfe_add_yuv(struct vfe_output *output, u8 bus_client_y,
		 u8 bus_client_c, u8 comp_group);

void vfe_add_ds(struct vfe_output *output, u8 pad, u8 bus_client,
		u8 comp_group);

void vfe_add_stats(struct vfe_output *output, u8 bus_client, u8 comp_group);
int vfe_alloc_stats(struct vfe_device *vfe, struct vfe_output *output);

void vfe_add_params(struct vfe_output *output, u8 comp_group);

void vfe_set_pix(struct vfe_line *line, int num_outputs);

#endif /* QC_MSM_CAMSS_VFE_H */
