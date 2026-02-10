#include "camss-metadata.h"
#include "camss.h"
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-sg.h>

#define MSM_METADA_NAME "msm_metadata"

// This could be improved by linking this device to the appropriate VFE
static const struct v4l2_subdev_ops metadata_v4l2_ops = {
	// .core = &csid_core_ops,
	// .video = &csid_video_ops,
	// .pad = &csid_pad_ops,
};



struct metdata_data_buffer {
	u64 sof_timestamp;
	u64 eof_timestamp;
	u64 sof_isr_timestamp;
	s64 isp_to_kernel_raw_ts_offset_floor;
	uint8_t sensorMetaDataBuffer[1208];
    uint8_t frameheaderData[16];
    uint8_t imgHistFrameheaderData[16];
    uint8_t imgHistData[2048];
    uint8_t bayerHistFrameheaderData[16];
    uint8_t bayerHistData[2048];
};

struct metadata_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head queue;
	u32 buff_addr[VIDEO_MAX_PLANES];
};

static int video_queue_setup(struct vb2_queue *vq,
					unsigned int *num_buffers,
					unsigned int *num_planes,
					unsigned int sizes[],
					struct device *alloc_devs[])
{
	*num_planes = 1;

	*num_buffers = clamp_t(u32, *num_buffers, 2,
			       32);

	sizes[0] = sizeof(struct metdata_data_buffer);

	return 0;
}

static int video_buf_prepare(struct vb2_buffer *vb)
{
	if (vb2_plane_size(vb, 0) < sizeof(struct metdata_data_buffer))
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, sizeof(struct metdata_data_buffer));

	return 0;
}


static void metadata_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct metadata_buffer *metada_buf = container_of(vbuf, struct metadata_buffer, vb);
	struct vb2_queue *vq = vb->vb2_queue;
	struct metadata_device *metadata = vq->drv_priv;

	spin_lock_irq(&metadata->lock);
	list_add_tail(&metada_buf->queue, &metadata->metadata);
	spin_unlock_irq(&metadata->lock);
}

void metadata_send(struct metadata_device *metadata,
                   u64 wm_done_timestamp,
                   u64 sof_isp_timestamp,
                   u64 eof_isp_timestamp,
                   u64 sof_isr_timestamp,
                   s64 isp_to_kernel_raw_ts_offset_floor,
                   u32 sequence,
                   void* sensorDataBuffer,
                   void* frameheaderData,
                   void* imgHistFrameheaderData,
                   void* imgHistData,
                   void* bayerHistFrameheaderData,
                   void* bayerHistData)
{
	struct metdata_data_buffer *cur_metadata_data_buffer;
	struct metadata_buffer *cur_buf = NULL;
	u64 timestamp = ktime_get_ns()/5;

	/* get one empty buffer */
	if (!list_empty(&metadata->metadata)) {
		cur_buf = list_first_entry(&metadata->metadata,
					   struct metadata_buffer, queue);
		list_del(&cur_buf->queue);
	}

	if (!cur_buf) {
		return;
	}
	cur_metadata_data_buffer = (struct metdata_data_buffer *)
			vb2_plane_vaddr(&cur_buf->vb.vb2_buf, 0);

	cur_metadata_data_buffer->sof_timestamp = sof_isp_timestamp;
	cur_metadata_data_buffer->eof_timestamp = eof_isp_timestamp;
	cur_metadata_data_buffer->isp_to_kernel_raw_ts_offset_floor = isp_to_kernel_raw_ts_offset_floor;
	cur_metadata_data_buffer->sof_isr_timestamp = sof_isr_timestamp;
	if(sensorDataBuffer != NULL) {
		memcpy(cur_metadata_data_buffer->sensorMetaDataBuffer,sensorDataBuffer,1208);
	}
    if(frameheaderData) {
		memcpy(cur_metadata_data_buffer->frameheaderData, frameheaderData, 16);
    }
    if(imgHistData && imgHistFrameheaderData) {
		memcpy(cur_metadata_data_buffer->imgHistFrameheaderData, imgHistFrameheaderData, 16);
		memcpy(cur_metadata_data_buffer->imgHistData, imgHistData, 2048);
    }
    if(bayerHistData && bayerHistFrameheaderData)
    {
		memcpy(cur_metadata_data_buffer->bayerHistFrameheaderData, bayerHistFrameheaderData, 16);
		memcpy(cur_metadata_data_buffer->bayerHistData, bayerHistData, 4096);
    }
	vb2_set_plane_payload(&cur_buf->vb.vb2_buf, 0,
			      sizeof(struct metdata_data_buffer)); //TODO: use a struct of a defined size
	cur_buf->vb.sequence = sequence;
	cur_buf->vb.vb2_buf.timestamp = timestamp;
	vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

}

static void video_stop_streaming(struct vb2_queue *q)
{
	struct metadata_device *metadata = q->drv_priv;
	struct metadata_buffer *metada_buf;
	unsigned int i;
	spin_lock_irq(&metadata->lock);
	for (i = 0; i < 32; i++) {
		if (list_empty(&metadata->metadata))
			break;
		metada_buf = list_first_entry(&metadata->metadata,
				       struct metadata_buffer, queue);
		list_del(&metada_buf->queue);
		vb2_buffer_done(&metada_buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irq(&metadata->lock);
}

static const struct vb2_ops msm_video_vb2_q_ops = {
	.queue_setup     = video_queue_setup,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
	//.buf_init        = video_buf_init,
	.buf_prepare     = video_buf_prepare,
	.buf_queue       = metadata_vb2_buf_queue,
	//.start_streaming = video_start_streaming, // this isn't need for a metadata device
	.stop_streaming  = video_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static const struct v4l2_file_operations msm_vid_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release, // shouldn't need these
	.poll           = vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.read		= vb2_fop_read,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int video_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	struct metadata_device *meta = video_drvdata(file);

	strscpy(cap->driver, "qcom-camss", sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Camera Subsystem", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
	dev_name(meta->camss->dev));

	return 0;
}

static int metadata_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct video_device *video = video_devdata(file);
	if (f->type != video->queue->type) {
		printk("fmt does not match f->type:%d, video->queue->type:%d",f->type,video->queue->type);
	}

	if (f->index > 0) {
		return -EINVAL;
	}

	f->pixelformat = V4L2_META_FMT_RK_ISP1_STAT_3A; //Possible improvement: Make a new metadata format. For now this allows us to get data out
	return 0;
}

static int metadata_g_fmt_meta_cap(struct file *file, void *priv,
				       struct v4l2_format *f)
{
	struct video_device *video = video_devdata(file);
	struct v4l2_meta_format *meta = &f->fmt.meta;

	if (f->type != video->queue->type)
		return -EINVAL;

	memset(meta, 0, sizeof(*meta));
	meta->dataformat = V4L2_META_FMT_RK_ISP1_STAT_3A;
	meta->buffersize = sizeof(struct metdata_data_buffer);//Improvement: get this set for real //stats->vdev_fmt.fmt.meta.buffersize;

	return 0;
}

static const struct v4l2_ioctl_ops msm_meta_ioctl_ops = {
	.vidioc_querycap		= video_querycap,
	.vidioc_enum_fmt_vid_cap	= metadata_enum_fmt,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_enum_fmt_meta_cap = metadata_enum_fmt,
	.vidioc_g_fmt_meta_cap = metadata_g_fmt_meta_cap,
	.vidioc_s_fmt_meta_cap = metadata_g_fmt_meta_cap,
	.vidioc_try_fmt_meta_cap = metadata_g_fmt_meta_cap,
};

static void msm_video_release(struct video_device *vdev)
{
	struct camss_video *video = video_get_drvdata(vdev);

	media_entity_cleanup(&vdev->entity);

	mutex_destroy(&video->q_lock);
	mutex_destroy(&video->lock);

	if (atomic_dec_and_test(&video->camss->ref_count))
		camss_delete(video->camss);
}


int msm_metadata_subdev_init(struct camss *camss, struct metadata_device *metadata, const struct resources *res, u8 id) {
	metadata->id = id;

	//metadata->base = camss->vfe[id].base; //TODO - possibly bake IS_LITE offset in here?
	
	metadata->camss = camss;
	return 0;
}

int msm_metadata_register_entity(struct metadata_device *metadata, struct v4l2_device *v4l2_dev) {
	
	struct v4l2_subdev *sd = &metadata->subdev;
	
	struct device *dev = metadata->camss->dev;
	int ret;
	struct camss_video *video_out; 
    struct video_device *vdev = NULL;
    struct vb2_queue *q = NULL;
    struct media_pad *pad = NULL;
	char name[32];

	v4l2_subdev_init(sd,&metadata_v4l2_ops);

	

	snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
		 MSM_METADA_NAME, metadata->id);

	//sets the subdev private data so that we can recover it later in the operations
	v4l2_set_subdevdata(sd, metadata);

	

	ret = v4l2_ctrl_handler_init(&metadata->ctrls, 1);
	if (ret < 0) {
		dev_err(dev, "Failed to init ctrl handler: %d\n", ret);
		return ret;
	}

	metadata->subdev.ctrl_handler = &metadata->ctrls;

	//ret = metadata_init_formats(sd, NULL); //TODO: Add pads and formats on pads 
	video_out = &metadata->video_out;

	snprintf(name, ARRAY_SIZE(name), "%s%d_%s%d",
			 MSM_METADA_NAME, metadata->id, "metadataout", 0);

	vdev = &video_out->vdev;

	mutex_init(&video_out->q_lock); /// maybe remove

	INIT_LIST_HEAD(&metadata->metadata);
	spin_lock_init(&metadata->lock);
	
	strscpy(vdev->name, name, sizeof(vdev->name));
	vdev->device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING;

	//new stuff
	video_out->camss = metadata->camss;
	q = &video_out->vb2_q;
	q->drv_priv = metadata;
	q->mem_ops = &vb2_dma_sg_memops;
	q->ops = &msm_video_vb2_q_ops;

	q->type = V4L2_BUF_TYPE_META_CAPTURE;
	q->io_modes = VB2_DMABUF | VB2_USERPTR | VB2_MMAP | VB2_READ;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct camss_buffer);
	q->dev = video_out->camss->dev;
	q->lock = &video_out->q_lock;

	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(dev, "Failed to init vb2 queue: %d\n", ret);
		return -1;
	}
	pad = &video_out->pad;
	pad->flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, pad);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init video entity: %d\n",
			ret);
		return -1;
	}


	if (ret < 0) {
		dev_err(dev, "Failed to register video node: %d\n", ret);
		return -1;
		goto error_reg_video; // TODO create error labels for goto
	}
	metadata->pad.flags = MEDIA_PAD_FL_SINK;

	mutex_init(&video_out->lock);



	vdev->fops = &msm_vid_fops;
	vdev->device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING;
	vdev->ioctl_ops = &msm_meta_ioctl_ops;
	vdev->release = msm_video_release;
	vdev->v4l2_dev = v4l2_dev;
	
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->queue = &video_out->vb2_q;
	vdev->lock = &video_out->lock;
	strscpy(vdev->name, name, sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to register video device: %d\n",
			ret);
		return -1;
		//goto error_video_register;
	}

	video_set_drvdata(vdev, video_out);
	atomic_inc(&video_out->camss->ref_count);

	return 0;

	error_reg_video:
		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
        sd = &metadata->subdev;
        msm_video_unregister(video_out);
        v4l2_device_unregister_subdev(sd);
        media_entity_cleanup(&sd->entity);

	return ret;
}