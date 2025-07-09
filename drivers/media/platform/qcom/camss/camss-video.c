// SPDX-License-Identifier: GPL-2.0
/*
 * camss-video.c
 *
 * Qualcomm MSM Camera Subsystem - V4L2 device node
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 */
#include <linux/slab.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-sg.h>

#include "camss-video.h"
#include "camss.h"
#include "camss-csiphy.h"

#define CAMSS_FRAME_MIN_WIDTH		1
#define CAMSS_FRAME_MAX_WIDTH		8191
#define CAMSS_FRAME_MIN_HEIGHT		1
#define CAMSS_FRAME_MAX_HEIGHT_RDI	8191
#define CAMSS_FRAME_MAX_HEIGHT_PIX	4096

/* -----------------------------------------------------------------------------
 * Helper functions
 */

/*
 * video_mbus_to_pix_mp - Convert v4l2_mbus_framefmt to v4l2_pix_format_mplane
 * @mbus: v4l2_mbus_framefmt format (input)
 * @pix: v4l2_pix_format_mplane format (output)
 * @f: a pointer to formats array element to be used for the conversion
 * @alignment: bytesperline alignment value
 *
 * Fill the output pix structure with information from the input mbus format.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int video_mbus_to_pix_mp(const struct v4l2_mbus_framefmt *mbus,
				struct v4l2_pix_format_mplane *pix,
				const struct camss_format_info *f,
				unsigned int alignment)
{
	unsigned int i;
	u32 bytesperline;

	memset(pix, 0, sizeof(*pix));
	v4l2_fill_pix_format_mplane(pix, mbus);
	pix->pixelformat = f->pixelformat;
	pix->num_planes = f->planes;
	for (i = 0; i < pix->num_planes; i++) {
		bytesperline = pix->width / f->hsub[i].numerator *
			f->hsub[i].denominator * f->bpp[i] / 8;
		bytesperline = ALIGN(bytesperline, alignment);
		pix->plane_fmt[i].bytesperline = bytesperline;
		pix->plane_fmt[i].sizeimage = pix->height /
				f->vsub[i].numerator * f->vsub[i].denominator *
				bytesperline;
	}

	return 0;
}

/*
 * Helper functions to identify and manage CSIPHY devices
 */
static bool is_csiphy_subdev(struct v4l2_subdev *subdev)
{
	return strstr(subdev->name, "csiphy") != NULL;
}

static int csiphy_get_lanes_enable_count(struct v4l2_subdev *subdev)
{
	struct csiphy_device *csiphy;
	
	if (!is_csiphy_subdev(subdev))
		return -1;
		
	csiphy = v4l2_get_subdevdata(subdev);
	if (!csiphy)
		return -1;
		
	return csiphy->lanes_enable_count;
}

static void csiphy_increment_lanes_enable_count(struct v4l2_subdev *subdev)
{
	struct csiphy_device *csiphy;
	
	if (!is_csiphy_subdev(subdev))
		return;
		
	csiphy = v4l2_get_subdevdata(subdev);
	if (csiphy)
		csiphy->lanes_enable_count++;
}

static void csiphy_decrement_lanes_enable_count(struct v4l2_subdev *subdev)
{
	struct csiphy_device *csiphy;
	
	if (!is_csiphy_subdev(subdev))
		return;
		
	csiphy = v4l2_get_subdevdata(subdev);
	if (csiphy) {
		csiphy->lanes_enable_count--;
		if (csiphy->lanes_enable_count < 0) {
			dev_warn(csiphy->camss->dev, "csiphy%d lanes_enable_count went negative, resetting to 0\n", csiphy->id);
			csiphy->lanes_enable_count = 0;
		}
	}
}

static struct v4l2_subdev *video_remote_subdev(struct camss_video *video,
					       u32 *pad)
{
	struct media_pad *remote;

	remote = media_pad_remote_pad_first(&video->pad);

	if (!remote || !is_media_entity_v4l2_subdev(remote->entity))
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int video_get_subdev_format(struct camss_video *video,
				   struct v4l2_format *format)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EPIPE;

	fmt.pad = pad;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	ret = camss_format_find_format(fmt.format.code, format->fmt.pix_mp.pixelformat,
				       video->formats, video->nformats);
	if (ret < 0)
		return ret;

	format->type = video->type;

	return video_mbus_to_pix_mp(&fmt.format, &format->fmt.pix_mp,
				    &video->formats[ret], video->bpl_alignment);
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 */

static int video_queue_setup(struct vb2_queue *q,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], struct device *alloc_devs[])
{
	struct camss_video *video = vb2_get_drv_priv(q);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	unsigned int i;

	if (*num_planes) {
		if (*num_planes != format->num_planes)
			return -EINVAL;

		for (i = 0; i < *num_planes; i++)
			if (sizes[i] < format->plane_fmt[i].sizeimage)
				return -EINVAL;

		return 0;
	}

	*num_planes = format->num_planes;

	for (i = 0; i < *num_planes; i++)
		sizes[i] = format->plane_fmt[i].sizeimage;

	return 0;
}

static int video_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	struct sg_table *sgt;
	unsigned int i;

	for (i = 0; i < format->num_planes; i++) {
		sgt = vb2_dma_sg_plane_desc(vb, i);
		if (!sgt)
			return -EFAULT;

		buffer->addr[i] = sg_dma_address(sgt->sgl);

		// Keep track of the sg_table for the extra output plane to be able to copy data from the kernel into the plane
		// (we always get a single plane from camss, so assume that this is the extra plane if i!=0)
		if(i != 0) {
			buffer->extra_output_plane_sgt = sgt;
			// set the addr here since addr[1] might be overwritten with the cbcr plane addr for NV12 case (even if we have a single plane)
			buffer->addr[video->extra_output_addr_idx] = sg_dma_address(sgt->sgl);
		}
	}

	// Setup an extra buffer for metadata/statistics streaming if required
	if(video->required_extrabuffer_size)
	{
		buffer->extrabuffer_size = video->required_extrabuffer_size;
		// Allocate the extra buffer
		buffer->extrabuffer_vaddr =  kzalloc(buffer->extrabuffer_size, GFP_KERNEL);
		if (buffer->extrabuffer_vaddr == NULL) {
			return -ENOMEM;
		}
		memset(buffer->extrabuffer_vaddr, 0, buffer->extrabuffer_size);
		// Create a DMA mapping so the buffer can be accessed by the VFE
		buffer->extrabuffer_dma_addr = dma_map_single(video->camss->dev, buffer->extrabuffer_vaddr, buffer->extrabuffer_size, DMA_BIDIRECTIONAL);
		if(dma_mapping_error(video->camss->dev, buffer->extrabuffer_dma_addr))
		{
			dev_err(video->camss->dev, "Failed to create a DMA mapping for the camss extra buffer \n");
			buffer->extrabuffer_dma_addr = 0;
			kfree(buffer->extrabuffer_vaddr);
			buffer->extrabuffer_vaddr = NULL;
			buffer->extrabuffer_size = 0;
			return -ENOMEM;
  		}
 	}

	if (format->pixelformat == V4L2_PIX_FMT_NV12 ||
			format->pixelformat == V4L2_PIX_FMT_NV21 ||
			format->pixelformat == V4L2_PIX_FMT_NV16 ||
			format->pixelformat == V4L2_PIX_FMT_NV61)
		buffer->addr[1] = buffer->addr[0] +
				format->plane_fmt[0].bytesperline *
				format->height;

	return 0;
}

static int video_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	unsigned int i;

	for (i = 0; i < format->num_planes; i++) {
		if (format->plane_fmt[i].sizeimage > vb2_plane_size(vb, i))
			return -EINVAL;

		vb2_set_plane_payload(vb, i, format->plane_fmt[i].sizeimage);
	}

	vbuf->field = V4L2_FIELD_NONE;

	return 0;
}

static void video_buf_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer, vb);

	// Cleanup the extra buffer if we had one
	if(buffer->extrabuffer_vaddr && buffer->extrabuffer_dma_addr)
	{
		dma_unmap_single(video->camss->dev, buffer->extrabuffer_dma_addr, buffer->extrabuffer_size, DMA_BIDIRECTIONAL);
		kfree(buffer->extrabuffer_vaddr);
		buffer->extrabuffer_size = 0;
	}
}

static void video_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);

	video->ops->queue_buffer(video, buffer);
}

static int video_check_format(struct camss_video *video)
{
	struct v4l2_pix_format_mplane *pix = &video->active_fmt.fmt.pix_mp;
	struct v4l2_format format;
	struct v4l2_pix_format_mplane *sd_pix = &format.fmt.pix_mp;
	int ret;

	sd_pix->pixelformat = pix->pixelformat;
	ret = video_get_subdev_format(video, &format);
	if (ret < 0)
		return ret;

	if (pix->pixelformat != sd_pix->pixelformat ||
	    pix->height != sd_pix->height ||
	    pix->width != sd_pix->width ||
	    pix->num_planes != sd_pix->num_planes ||
	    pix->field != format.fmt.pix_mp.field)
		return -EPIPE;

	return 0;
}

static int video_prepare_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;
	int ret;

	ret = v4l2_pipeline_pm_get(&vdev->entity);
	if (ret < 0) {
		dev_err(video->camss->dev, "Failed to power up pipeline: %d\n",
			ret);
	}

	return ret;
}

static int video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret, i;

	ret = video_device_pipeline_alloc_start(vdev);
	if (ret < 0) {
		dev_err(video->camss->dev, "Failed to start media pipeline: %d\n", ret);
		goto flush_buffers;
	}

	// Ignore this check when using an extra plane
	if(!video->has_extra_output_plane_cfg) {
		ret = video_check_format(video);
		if (ret < 0)
			goto error;
	}

	entity = &vdev->entity;
	pad = &entity->pads[0];
	while (1) {
		pad = media_pad_remote_pad_first(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		bool should_stream = true;
		
		if (is_csiphy_subdev(subdev)) {
			int current_count = csiphy_get_lanes_enable_count(subdev);
			if (current_count < 0) {
				dev_err(video->camss->dev, "Failed to get CSIPHY lanes enable count\n");
				goto error;
			}
			
			// Only stream if this is the first enable (0 -> 1 transition)
			should_stream = (current_count == 0);
			
			// Increment count regardless
			csiphy_increment_lanes_enable_count(subdev);
			
			if (!should_stream) {
				dev_info(video->camss->dev, "Skipping already streaming CSIPHY subdev %s when starting video pipeline %s\n", subdev->name, vdev->name);
			}
		}

		if (should_stream) {
			ret = v4l2_subdev_call(subdev, video, s_stream, 1);
			if (ret < 0 && ret != -ENOIOCTLCMD) {
				dev_err(video->camss->dev, "Failed to start subdev %s: %d\n", subdev->name, ret);
				goto error;
			}
		}

		/* find corresponding sink pad */
		for (i = 0; i < entity->num_pads; i++) {
			if (!(entity->pads[i].flags & MEDIA_PAD_FL_SINK))
				continue;

			if (entity->ops && entity->ops->has_pad_interdep &&
			    !entity->ops->has_pad_interdep(entity, pad->index, i))
				continue;

			pad = &entity->pads[i];
			break;
		}

		if (i == entity->num_pads)
			break;
	}

	return 0;

error:
	video_device_pipeline_stop(vdev);

flush_buffers:
	video->ops->flush_buffers(video, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void video_stop_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret, i;

	entity = &vdev->entity;
	pad = &entity->pads[0];
	while (1) {
		pad = media_pad_remote_pad_first(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		bool should_stream = true;
		
		if (is_csiphy_subdev(subdev)) {
			int current_count = csiphy_get_lanes_enable_count(subdev);
			if (current_count < 0) {
				dev_err(video->camss->dev, "Failed to get CSIPHY lanes enable count\n");
				return;
			}
			
			// Only stream if this will be the last disable (1 -> 0 transition)
			should_stream = (current_count == 1);
			
			// Decrement count regardless
			csiphy_decrement_lanes_enable_count(subdev);
			
			if (!should_stream) {
				dev_info(video->camss->dev, "Skipping already idle CSIPHY subdev %s when stopping video pipeline %s\n", subdev->name, vdev->name);
			}
		}

		if (should_stream) {
			ret = v4l2_subdev_call(subdev, video, s_stream, 0);
			if (ret) {
				dev_err(video->camss->dev, "Video pipeline stop failed for subdev %s: %d\n", subdev->name, ret);
				return;
			}
		}

		/* find corresponding sink pad */
		for (i = 0; i < entity->num_pads; i++) {
			if (!(entity->pads[i].flags & MEDIA_PAD_FL_SINK))
				continue;

			if (entity->ops && entity->ops->has_pad_interdep &&
			    !entity->ops->has_pad_interdep(entity, pad->index, i))
				continue;

			pad = &entity->pads[i];
			break;
		}

		if (i == entity->num_pads)
			break;
	}

	video_device_pipeline_stop(vdev);

	video->ops->flush_buffers(video, VB2_BUF_STATE_ERROR);
}

static void video_unprepare_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;

	v4l2_pipeline_pm_put(&vdev->entity);
}

static const struct vb2_ops msm_video_vb2_q_ops = {
	.queue_setup     = video_queue_setup,
	.buf_init        = video_buf_init,
	.buf_prepare     = video_buf_prepare,
	.buf_cleanup     = video_buf_cleanup,
	.buf_queue       = video_buf_queue,
	.prepare_streaming = video_prepare_streaming,
	.start_streaming = video_start_streaming,
	.stop_streaming  = video_stop_streaming,
	.unprepare_streaming = video_unprepare_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int video_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "qcom-camss", sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Camera Subsystem", sizeof(cap->card));

	return 0;
}

static int video_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct camss_video *video = video_drvdata(file);
	int i, j, k;
	u32 mcode = f->mbus_code;

	if (f->type != video->type)
		return -EINVAL;

	if (f->index >= video->nformats)
		return -EINVAL;

	/*
	 * Find index "i" of "k"th unique pixelformat in formats array.
	 *
	 * If f->mbus_code passed to video_enum_fmt() is not zero, a device
	 * with V4L2_CAP_IO_MC capability restricts enumeration to only the
	 * pixel formats that can be produced from that media bus code.
	 * This is implemented by skipping video->formats[] entries with
	 * code != f->mbus_code (if f->mbus_code is not zero).
	 * If the f->mbus_code passed to video_enum_fmt() is not supported,
	 * -EINVAL is returned.
	 * If f->mbus_code is zero, all the pixel formats are enumerated.
	 */
	k = -1;
	for (i = 0; i < video->nformats; i++) {
		if (mcode != 0 && video->formats[i].code != mcode)
			continue;

		for (j = 0; j < i; j++) {
			if (mcode != 0 && video->formats[j].code != mcode)
				continue;
			if (video->formats[i].pixelformat ==
					video->formats[j].pixelformat)
				break;
		}

		if (j == i)
			k++;

		if (k == f->index)
			break;
	}

	if (k == -1 || k < f->index)
		/*
		 * All the unique pixel formats matching the arguments
		 * have been enumerated (k >= 0 and f->index > 0), or
		 * no pixel formats match the non-zero f->mbus_code (k == -1).
		 */
		return -EINVAL;

	f->pixelformat = video->formats[i].pixelformat;

	return 0;
}

static int video_enum_framesizes(struct file *file, void *fh,
				 struct v4l2_frmsizeenum *fsize)
{
	struct camss_video *video = video_drvdata(file);
	int i;

	if (fsize->index)
		return -EINVAL;

	/* Only accept pixel format present in the formats[] table */
	for (i = 0; i < video->nformats; i++) {
		if (video->formats[i].pixelformat == fsize->pixel_format)
			break;
	}

	if (i == video->nformats)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = CAMSS_FRAME_MIN_WIDTH;
	fsize->stepwise.max_width = CAMSS_FRAME_MAX_WIDTH;
	fsize->stepwise.min_height = CAMSS_FRAME_MIN_HEIGHT;
	fsize->stepwise.max_height = (video->line_based) ?
		CAMSS_FRAME_MAX_HEIGHT_PIX : CAMSS_FRAME_MAX_HEIGHT_RDI;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int video_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	*f = video->active_fmt;

	return 0;
}

static int __video_try_fmt(struct camss_video *video, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp;
	const struct camss_format_info *fi;
	struct v4l2_plane_pix_format *p;
	u32 bytesperline[3] = { 0 };
	u32 sizeimage[3] = { 0 };
	u32 width, height;
	u32 bpl, lines;
	int i, j;

	pix_mp = &f->fmt.pix_mp;

	if (video->line_based)
		for (i = 0; i < pix_mp->num_planes && i < 3; i++) {
			p = &pix_mp->plane_fmt[i];
			bytesperline[i] = clamp_t(u32, p->bytesperline,
						  1, 65528);
			sizeimage[i] = clamp_t(u32, p->sizeimage,
					       bytesperline[i],
					       bytesperline[i] * CAMSS_FRAME_MAX_HEIGHT_PIX);
		}

	for (j = 0; j < video->nformats; j++)
		if (pix_mp->pixelformat == video->formats[j].pixelformat)
			break;

	if (j == video->nformats)
		j = 0; /* default format */

	fi = &video->formats[j];
	width = pix_mp->width;
	height = pix_mp->height;

	memset(pix_mp, 0, sizeof(*pix_mp));

	pix_mp->pixelformat = fi->pixelformat;
	pix_mp->width = clamp_t(u32, width, 1, CAMSS_FRAME_MAX_WIDTH);
	pix_mp->height = clamp_t(u32, height, 1, CAMSS_FRAME_MAX_HEIGHT_RDI);
	pix_mp->num_planes = fi->planes;
	for (i = 0; i < pix_mp->num_planes; i++) {
		bpl = pix_mp->width / fi->hsub[i].numerator *
			fi->hsub[i].denominator * fi->bpp[i] / 8;
		bpl = ALIGN(bpl, video->bpl_alignment);
		pix_mp->plane_fmt[i].bytesperline = bpl;
		pix_mp->plane_fmt[i].sizeimage = pix_mp->height /
			fi->vsub[i].numerator * fi->vsub[i].denominator * bpl;
	}

	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->colorspace = V4L2_COLORSPACE_SRGB;
	pix_mp->flags = 0;
	pix_mp->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix_mp->colorspace);
	pix_mp->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
					pix_mp->colorspace, pix_mp->ycbcr_enc);
	pix_mp->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix_mp->colorspace);

	if (video->line_based)
		for (i = 0; i < pix_mp->num_planes; i++) {
			p = &pix_mp->plane_fmt[i];
			p->bytesperline = clamp_t(u32, p->bytesperline,
						  1, 65528);
			p->sizeimage = clamp_t(u32, p->sizeimage,
					       p->bytesperline,
					       p->bytesperline * CAMSS_FRAME_MAX_HEIGHT_PIX);
			lines = p->sizeimage / p->bytesperline;

			if (p->bytesperline < bytesperline[i])
				p->bytesperline = ALIGN(bytesperline[i], 8);

			if (p->sizeimage < p->bytesperline * lines)
				p->sizeimage = p->bytesperline * lines;

			if (p->sizeimage < sizeimage[i])
				p->sizeimage = sizeimage[i];
		}

	return 0;
}

static int video_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	return __video_try_fmt(video, f);
}

static int video_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&video->vb2_q))
		return -EBUSY;

	// clear any previous additional output plane configuration when setting the format
	// (extra output config act as an extension of the format so should be used after the format has been set)
	video->has_extra_output_plane_cfg = false;
	memset(&video->extra_output_plane_cfg, 0, sizeof(video->extra_output_plane_cfg));

	ret = __video_try_fmt(video, f);
	if (ret < 0)
		return ret;

	video->active_fmt = *f;

	return 0;
}

static int video_enum_input(struct file *file, void *fh,
			    struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strscpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int video_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static struct vfe_device* video_get_vfe_device(struct camss_video *video)
{
	struct media_pad *vfe_src_pad = media_entity_remote_source_pad_unique(&video->vdev.entity);
	if(IS_ERR(vfe_src_pad)) {
		dev_err(video->camss->dev, "Failed to get vfe source pad");
		return (void*)vfe_src_pad;
	}

	struct v4l2_subdev *vfe_subdev = media_entity_to_v4l2_subdev(vfe_src_pad->entity);
	if(!vfe_subdev) {
		dev_err(video->camss->dev, "Failed to get vfe v4l2 subdev");
		return ERR_PTR(-ENODEV);
	}

	struct vfe_device *vfe = v4l2_get_subdevdata(vfe_subdev);
	if(!vfe) {
		dev_err(video->camss->dev, "Failed to get vfe subdev data");
		return ERR_PTR(-ENODEV);
	}

	return vfe;
}

static struct csid_device* video_get_csid_device(struct camss_video *video)
{
	struct media_pad *vfe_src_pad = media_entity_remote_source_pad_unique(&video->vdev.entity);
	if(IS_ERR(vfe_src_pad)) {
		dev_err(video->camss->dev, "Failed to get vfe source pad");
		return (void*)vfe_src_pad;
	}

	struct media_pad *csid_src_pad = media_entity_remote_source_pad_unique(vfe_src_pad->entity);
	if(IS_ERR(csid_src_pad)) {
		dev_err(video->camss->dev, "Failed to get csid source pad");
		return (void*)csid_src_pad;
	}

	struct v4l2_subdev *csid_subdev = media_entity_to_v4l2_subdev(csid_src_pad->entity);
	if(!csid_subdev) {
		dev_err(video->camss->dev, "Failed to get csid v4l2 subdev");
		return ERR_PTR(-ENODEV);
	}

	struct csid_device *csid = v4l2_get_subdevdata(csid_subdev);
	if(!csid) {
		dev_err(video->camss->dev, "Failed to get csid subdev data");
		return ERR_PTR(-ENODEV);
	}

	return csid;
}

static int video_set_extra_output_plane_config(struct camss_video *video, void* arg, bool is_legacy)
{
	// cannot configure output if queue is already active and has buffers
	if (vb2_is_busy(&video->vb2_q)) {
		dev_warn(video->camss->dev, "camss-video: cannot configure extra output plane when queue is active \n");
		return -EBUSY;
	}

	// camss always configure a single plane for the output image data so expect 1 active plane when no extra output has been configured
	if(!video->has_extra_output_plane_cfg && (video->active_fmt.fmt.pix_mp.num_planes != 1)) {
		dev_warn(video->camss->dev, "camss-video: unexpected plane count (%d) in the active format. Expected 1.\n", video->active_fmt.fmt.pix_mp.num_planes);
		return -EINVAL;
	}

	video->has_extra_output_plane_cfg = true;
	// If using the legacy ioctl and structure build the new structure from the old version, otherwise just copy the input
	if(is_legacy) {
		struct camss_extra_output_plane_cfg_legacy *legacy_config = arg;
		memset(&video->extra_output_plane_cfg, 0, sizeof(struct camss_extra_output_plane_cfg));
		video->extra_output_plane_cfg.plane_size = legacy_config->plane_size;
		video->extra_output_plane_cfg.frameheader_en = legacy_config->frameheader_en;
		video->extra_output_plane_cfg.frameheader_offset = legacy_config->frameheader_offset;
		for(int i=0; i<CAMSS_EXTRA_OUTPUT_TYPE_MAX_LEGACY; i++) {
			video->extra_output_plane_cfg.extra_output_en[i] = legacy_config->extra_output_en[i];
			video->extra_output_plane_cfg.extra_output_offset[i] = legacy_config->extra_output_offset[i];
			video->extra_output_plane_cfg.extra_output_frameheader_en[i] = legacy_config->extra_output_frameheader_en[i];
			video->extra_output_plane_cfg.extra_output_frameheader_offset[i] = legacy_config->extra_output_frameheader_offset[i];
		}
	}
	else
	{
		struct camss_extra_output_plane_cfg *config = arg;
		video->extra_output_plane_cfg = *config;
	}

	// Check the new config for unsupported types being used, ignore the configuration
	bool valid_config = true;
	for(int i=CAMSS_EXTRA_OUTPUT_TYPE_END; i<CAMSS_EXTRA_OUTPUT_TYPE_MAX; i++) {
		if((video->extra_output_plane_cfg.extra_output_en[i] != 0) ||
		   (video->extra_output_plane_cfg.extra_output_offset[i] != 0) ||
		   (video->extra_output_plane_cfg.extra_output_frameheader_en[i] != 0) ||
		   (video->extra_output_plane_cfg.extra_output_frameheader_offset[i] != 0))
		{
			dev_warn(video->camss->dev, "camss-video: found non-empty extra output config for unsupported type %d \n", i);
			valid_config = false;
		}
	}
	if(!valid_config) {
		dev_warn(video->camss->dev, "camss-video: invalid configuration provided, configuration will be ignored \n");
		video->has_extra_output_plane_cfg = false;
		memset(&video->extra_output_plane_cfg, 0, sizeof(struct camss_extra_output_plane_cfg));
		return -EINVAL;
	}

	// update the plane count to have the additional plane and update its size requirement
	video->active_fmt.fmt.pix_mp.num_planes = 2; // always put the extra output in plane 1 (never used in camss formats)
	video->active_fmt.fmt.pix_mp.plane_fmt[1].sizeimage = video->extra_output_plane_cfg.plane_size;
	// get which index is used to store the additional plane address, NV12 format will use 2 addresses (Y/CbCr planes)
	video->extra_output_addr_idx = (video->active_fmt.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_NV12) ? 2 : 1;

	dev_warn(video->camss->dev, "camss-video: extra output plane configured \n");

	return 0;
}

static int video_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_CAMSS_HW_ERROR)
		return -EINVAL;

	// No id expected for V4L2_EVENT_CAMSS_HW_ERROR so we should receive 0
	if (sub->id != 0)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 0, NULL);
}

static long video_ioctl(struct file *file, void *fh, bool valid_prio, unsigned int cmd, void *arg)
{
	struct camss_video *video = video_drvdata(file);
	long ret = 0;

	// Grab the vfe and csid device since these are needed for the extended
	// configuration and to get/set hardware registers
	struct vfe_device *vfe = video_get_vfe_device(video);
	if(IS_ERR(vfe)) {
		dev_err(video->camss->dev, "Failed to get the vfe from the video device");
		return PTR_ERR(vfe);
	}

	struct csid_device *csid = video_get_csid_device(video);
	if(IS_ERR(csid)) {
		dev_err(video->camss->dev, "Failed to get the csid from the video device");
		return PTR_ERR(csid);
	}

	switch(cmd)
	{
		case VIDIOC_PRIV_S_REGS:
			if(csid->res->hw_ops->set_csid_vfe_registers) {
				ret = csid->res->hw_ops->set_csid_vfe_registers(csid, arg);
			}
			else {
				// Operation not supported
				return -EINVAL;
			}
			break;
		case VIDIOC_PRIV_G_REGS:
			if(csid->res->hw_ops->get_csid_vfe_registers) {
				ret = csid->res->hw_ops->get_csid_vfe_registers(csid, arg);
			}
			else {
				// Operation not supported
				return -EINVAL;
			}
			break;
		case VIDIOC_PRIV_S_REGS_DELAYED:
			if(csid->res->hw_ops->set_csid_vfe_registers_delayed) {
				ret = csid->res->hw_ops->set_csid_vfe_registers_delayed(csid, arg);
			}
			else {
				// Operation not supported
				return -EINVAL;
			}
			break;
		case VIDIOC_PRIV_S_EXTRA_OUTPUT_PLANE_CFG_LEGACY:
			ret = video_set_extra_output_plane_config(video, arg, true);
			break;
		case VIDIOC_PRIV_S_EXTRA_OUTPUT_PLANE_CFG:
			ret = video_set_extra_output_plane_config(video, arg, false);
			break;
		case VIDIOC_PRIV_S_MULTI_VCDT_CFG:
			// request the csid multi-vcdt feature to be enabled with the vc passed as parameter
			csid->multi_vcdt_en = 1;
			csid->multi_vcdt_vc = (uintptr_t)arg;
			break;
		default:
			ret = -ENOTTY;
	}

	return ret;
}

static const struct v4l2_ioctl_ops msm_vid_ioctl_ops = {
	.vidioc_querycap		= video_querycap,
	.vidioc_enum_fmt_vid_cap	= video_enum_fmt,
	.vidioc_enum_framesizes		= video_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane	= video_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= video_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= video_try_fmt,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_enum_input		= video_enum_input,
	.vidioc_g_input			= video_g_input,
	.vidioc_s_input			= video_s_input,
	.vidioc_subscribe_event = video_subscribe_event,
	.vidioc_default			= video_ioctl,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static const struct v4l2_file_operations msm_vid_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.poll           = vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.read		= vb2_fop_read,
};

/* -----------------------------------------------------------------------------
 * CAMSS video core
 */

static void msm_video_release(struct video_device *vdev)
{
	struct camss_video *video = video_get_drvdata(vdev);

	media_entity_cleanup(&vdev->entity);

	mutex_destroy(&video->q_lock);
	mutex_destroy(&video->lock);

	if (atomic_dec_and_test(&video->camss->ref_count))
		camss_delete(video->camss);
}

/*
 * msm_video_init_format - Helper function to initialize format
 * @video: struct camss_video
 *
 * Initialize pad format with default value.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int msm_video_init_format(struct camss_video *video)
{
	int ret;
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.fmt.pix_mp = {
			.width = 1920,
			.height = 1080,
			.pixelformat = video->formats[0].pixelformat,
		},
	};

	ret = __video_try_fmt(video, &format);
	if (ret < 0)
		return ret;

	video->active_fmt = format;

	return 0;
}

/*
 * msm_video_register - Register a video device node
 * @video: struct camss_video
 * @v4l2_dev: V4L2 device
 * @name: name to be used for the video device node
 *
 * Initialize and register a video device node to a V4L2 device. Also
 * initialize the vb2 queue.
 *
 * Return 0 on success or a negative error code otherwise
 */

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name)
{
	struct media_pad *pad = &video->pad;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret;

	vdev = &video->vdev;

	mutex_init(&video->q_lock);

	q = &video->vb2_q;
	q->drv_priv = video;
	q->mem_ops = &vb2_dma_sg_memops;
	q->ops = &msm_video_vb2_q_ops;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_DMABUF | VB2_MMAP | VB2_READ;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct camss_buffer);
	q->dev = video->camss->dev;
	q->lock = &video->q_lock;
	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init vb2 queue: %d\n", ret);
		goto error_vb2_init;
	}

	pad->flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, pad);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init video entity: %d\n",
			ret);
		goto error_vb2_init;
	}

	mutex_init(&video->lock);

	ret = msm_video_init_format(video);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init format: %d\n", ret);
		goto error_video_register;
	}

	vdev->fops = &msm_vid_fops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING
			  | V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;
	vdev->ioctl_ops = &msm_vid_ioctl_ops;
	vdev->release = msm_video_release;
	vdev->v4l2_dev = v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->queue = &video->vb2_q;
	vdev->lock = &video->lock;
	strscpy(vdev->name, name, sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to register video device: %d\n",
			ret);
		goto error_video_register;
	}

	video_set_drvdata(vdev, video);
	atomic_inc(&video->camss->ref_count);

	return 0;

error_video_register:
	media_entity_cleanup(&vdev->entity);
	mutex_destroy(&video->lock);
error_vb2_init:
	mutex_destroy(&video->q_lock);

	return ret;
}

void msm_video_unregister(struct camss_video *video)
{
	atomic_inc(&video->camss->ref_count);
	vb2_video_unregister_device(&video->vdev);
	atomic_dec(&video->camss->ref_count);
}


