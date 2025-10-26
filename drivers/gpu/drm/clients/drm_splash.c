// SPDX-License-Identifier: GPL-2.0 or MIT
/*
 * Copyright (c) 2025 Francesco Valla <francesco@valla.it>
 * Stollen some bits from drm_log and simplefd
 * Copyright (c) 2025 Linaro Ltd
 */

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/iosys-map.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/platform_data/simplefb.h>
#include <linux/of_reserved_mem.h>

#include <drm/drm_client.h>
#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>

#include "drm_client_internal.h"
#include "drm_draw_internal.h"
#include "drm_internal.h"

#define DRM_SPLASH_BACKGROUND_FALLBACK_COLOUR	0x138FCD

struct drm_splash_scanout {
	struct drm_client_buffer *buffer;
	u32 color;
};

struct drm_splash {
	struct mutex lock;
	struct drm_client_dev client;
	u32 n_scanout;
	struct drm_splash_scanout *scanout;
	int fb_width;
	int fb_height;
	int fb_stride;
	const struct drm_format_info *fb_format;
	void *fb_base;
	size_t fb_size;
};

static struct drm_splash *client_to_drm_splash(struct drm_client_dev *client)
{
	return container_of_const(client, struct drm_splash, client);
}

static void drm_splash_fill(struct iosys_map *map, unsigned int dst_pitch,
			    unsigned int height, unsigned int width,
			    u32 px_width, u32 color)
{
	switch (px_width) {
	case 2:
		drm_draw_fill16(map, dst_pitch, height, width, color);
		break;
	case 3:
		drm_draw_fill24(map, dst_pitch, height, width, color);
		break;
	case 4:
		drm_draw_fill32(map, dst_pitch, height, width, color);
		break;
	default:
		WARN_ONCE(1, "Can't fill with pixel width %d\n", px_width);
	}
}

static int drm_splash_fill_solid_color(struct drm_splash_scanout *scanout)
{
	struct drm_framebuffer *fb = scanout->buffer->fb;
	struct drm_rect r = DRM_RECT_INIT(0, 0, fb->width, fb->height);
	u32 px_width = fb->format->cpp[0];
	struct iosys_map map;
	int ret;

	ret = drm_client_buffer_vmap_local(scanout->buffer, &map);
	if (ret)
		return ret;

	drm_splash_fill(&map, fb->pitches[0], drm_rect_height(&r),
			drm_rect_width(&r), px_width, scanout->color);

	drm_client_buffer_vunmap_local(scanout->buffer);

	return drm_client_framebuffer_flush(scanout->buffer, &r);
}

static int drm_splash_copy_simplefb(struct drm_splash *dsplash,
				    struct drm_splash_scanout *scanout)
{
	struct drm_framebuffer *fb = scanout->buffer->fb;
	struct drm_rect r = DRM_RECT_INIT(0, 0, fb->width, fb->height);
	struct iosys_map map;
	int line;
	int ret;

	ret = drm_client_buffer_vmap_local(scanout->buffer, &map);
	if (ret)
		return ret;

	for (line = 0; line < fb->height; ++line)
		iosys_map_memcpy_to(&map, fb->pitches[0] * line,
			dsplash->fb_base + (dsplash->fb_stride * line),
			min_t(unsigned int, fb->pitches[0], dsplash->fb_stride));

	drm_client_buffer_vunmap_local(scanout->buffer);

	return drm_client_framebuffer_flush(scanout->buffer, &r);
}

static u32 drm_splash_find_usable_format(struct drm_plane *plane)
{
	int i;

	for (i = 0; i < plane->format_count; i++)
		if (drm_draw_color_from_xrgb8888(0xffffff, plane->format_types[i]) != 0)
			return plane->format_types[i];

	return DRM_FORMAT_INVALID;
}

static int drm_splash_setup_modeset(struct drm_splash *dsplash,
				 struct drm_client_dev *client,
				 struct drm_mode_set *mode_set,
				 struct drm_splash_scanout *scanout)
{
	struct drm_crtc *crtc = mode_set->crtc;
	u32 width = mode_set->mode->hdisplay;
	u32 height = mode_set->mode->vdisplay;
	u32 format;

	if (dsplash->fb_base) {
		width = dsplash->fb_width;
		height = dsplash->fb_height;
		format = dsplash->fb_format->format;
	} else {
		format = drm_splash_find_usable_format(crtc->primary);
		if (format == DRM_FORMAT_INVALID)
			return -EINVAL;
		scanout->color =
			drm_draw_color_from_xrgb8888(DRM_SPLASH_BACKGROUND_FALLBACK_COLOUR,
						     format);
	}

	scanout->buffer = drm_client_framebuffer_create(client, width, height, format);
	if (IS_ERR(scanout->buffer)) {
		drm_warn(client->dev, "drm_splash can't create framebuffer %d %d %p4cc\n",
			 width, height, &format);
		return -ENOMEM;
	}

	mode_set->fb = scanout->buffer->fb;
	return 0;
}

static int drm_splash_count_modeset(struct drm_client_dev *client)
{
	struct drm_mode_set *mode_set;
	int count = 0;

	mutex_lock(&client->modeset_mutex);
	drm_client_for_each_modeset(mode_set, client)
		count++;
	mutex_unlock(&client->modeset_mutex);
	return count;
}

static void drm_splash_init_client(struct drm_splash *dsplash)
{
	struct drm_client_dev *client = &dsplash->client;
	struct drm_mode_set *mode_set;
	int i, max_modeset;
	int n_modeset = 0;

	if (drm_client_modeset_probe(client, 0, 0))
		return;

	max_modeset = drm_splash_count_modeset(client);
	if (!max_modeset)
		return;

	dsplash->scanout = kcalloc(max_modeset, sizeof(*dsplash->scanout), GFP_KERNEL);
	if (!dsplash->scanout)
		return;

	mutex_lock(&client->modeset_mutex);
	drm_client_for_each_modeset(mode_set, client) {
		if (!mode_set->mode)
			continue;
		if (drm_splash_setup_modeset(dsplash, client, mode_set, &dsplash->scanout[n_modeset]))
			continue;
		n_modeset++;
	}

	mutex_unlock(&client->modeset_mutex);
	if (n_modeset == 0)
		goto err_nomodeset;

	for (i = 0; i < n_modeset; i++) {
		if (!dsplash->scanout[i].buffer)
			continue;

		if (dsplash->fb_base)
			drm_splash_copy_simplefb(dsplash, &dsplash->scanout[i]);
		else
			drm_splash_fill_solid_color(&dsplash->scanout[i]);
	}

	if (drm_client_modeset_commit(client))
		goto err_failed_commit;

	dsplash->n_scanout = n_modeset;
	return;

err_failed_commit:
	for (i = 0; i < n_modeset; i++)
		drm_client_framebuffer_delete(dsplash->scanout[i].buffer);

err_nomodeset:
	kfree(dsplash->scanout);
	dsplash->scanout = NULL;
}

static void drm_splash_free_scanout(struct drm_client_dev *client)
{
	struct drm_splash *dsplash = client_to_drm_splash(client);
	int i;

	if (dsplash->n_scanout) {
		for (i = 0; i < dsplash->n_scanout; i++)
			drm_client_framebuffer_delete(dsplash->scanout[i].buffer);
		dsplash->n_scanout = 0;
		kfree(dsplash->scanout);
		dsplash->scanout = NULL;
	}
}

static void drm_splash_client_unregister(struct drm_client_dev *client)
{
	struct drm_splash *dsplash = client_to_drm_splash(client);
	struct drm_device *dev = client->dev;

	mutex_lock(&dsplash->lock);
	drm_splash_free_scanout(client);
	drm_client_release(client);
	mutex_unlock(&dsplash->lock);
	kfree(dsplash);
	drm_dbg(dev, "Unregistered with drm splash\n");
}

static int drm_splash_client_hotplug(struct drm_client_dev *client)
{
	struct drm_splash *dsplash = client_to_drm_splash(client);

	mutex_lock(&dsplash->lock);
	drm_splash_free_scanout(client);
	drm_splash_init_client(dsplash);
	mutex_unlock(&dsplash->lock);

	return 0;
}

static const struct drm_client_funcs drm_splash_client_funcs = {
	.owner		= THIS_MODULE,
	.hotplug	= drm_splash_client_hotplug,
	.unregister	= drm_splash_client_unregister,
};

static int drm_splash_get_width_of(struct device_node *of_node)
{
	u32 width;

	int ret = of_property_read_u32(of_node, "width", &width);
	if (ret)
		return ret;

	return width;
}

static int drm_splash_get_height_of(struct device_node *of_node)
{
	u32 height;

	int ret = of_property_read_u32(of_node, "height", &height);
	if (ret)
		return ret;

	return height;
}

static int drm_splash_get_stride_of(struct device_node *of_node)
{
	u32 stride;

	int ret = of_property_read_u32(of_node, "stride", &stride);
	if (ret)
		return ret;

	return stride;
}

static const struct drm_format_info *
drm_splash_get_validated_format(struct drm_device *dev, const char *format_name)
{
	static const struct simplefb_format formats[] = SIMPLEFB_FORMATS;
	const struct simplefb_format *fmt = formats;
	const struct simplefb_format *end = fmt + ARRAY_SIZE(formats);
	const struct drm_format_info *info;

	if (!format_name) {
		drm_err(dev, "drm_splash: missing framebuffer format\n");
		return ERR_PTR(-EINVAL);
	}

	while (fmt < end) {
		if (!strcmp(format_name, fmt->name)) {
			info = drm_format_info(fmt->fourcc);
			if (!info)
				return ERR_PTR(-EINVAL);
			return info;
		}
		++fmt;
	}

	drm_err(dev, "drm_splash: unknown framebuffer format %s\n",
		format_name);

	return ERR_PTR(-EINVAL);
}

static const struct drm_format_info *
drm_splash_get_format_of(struct drm_device *dev, struct device_node *of_node)
{
	const char *format;
	int ret;

	ret = of_property_read_string(of_node, "format", &format);
	if (ret)
		return ERR_PTR(ret);

	return drm_splash_get_validated_format(dev, format);
}

static struct resource *
drm_splash_get_memory_of(struct drm_device *dev, struct device_node *of_node)
{
	struct resource r, *res;
	int err;

	err = of_reserved_mem_region_to_resource(of_node, 0, &r);
	if (err)
		return NULL;

	res = devm_kmemdup(dev->dev, &r, sizeof(r), GFP_KERNEL);
	if (!res)
		return ERR_PTR(-ENOMEM);

	return res;
}

/**
 * drm_splash_register() - Register a drm device to drm_splash
 * @dev: the drm device to register.
 * @format: drm device preferred format.
 */
void drm_splash_register(struct drm_device *dev,
			 const struct drm_format_info *format)
{
	struct device_node *of_node;
	struct drm_splash *splash;

	splash = kzalloc(sizeof(*splash), GFP_KERNEL);
	if (!splash)
		goto err_warn;

	of_node = of_find_compatible_node(NULL, NULL, "simple-framebuffer");
	if (of_node && of_device_is_available(of_node)) {
		struct resource *mem = NULL;

		splash->fb_width = drm_splash_get_width_of(of_node);
		if (splash->fb_width < 0)
			goto no_simplefb;

		splash->fb_height = drm_splash_get_height_of(of_node);
		if (splash->fb_height < 0)
			goto no_simplefb;

		splash->fb_stride = drm_splash_get_stride_of(of_node);
		if (splash->fb_stride < 0)
			goto no_simplefb;

		splash->fb_format = drm_splash_get_format_of(dev, of_node);
		if (IS_ERR(splash->fb_format))
			goto no_simplefb;

		mem = drm_splash_get_memory_of(dev, of_node);
		if (IS_ERR_OR_NULL(mem))
			goto no_simplefb;

		splash->fb_size = resource_size(mem);
		splash->fb_base = devm_memremap(dev->dev, mem->start,
						splash->fb_size, MEMREMAP_WC);
		if (IS_ERR(splash->fb_base))
			goto no_simplefb;

		drm_info(dev, "drm_splash: found %dx%d/%d format %p4cc framebuffer\n",
			 splash->fb_width, splash->fb_height, splash->fb_stride,
			 &splash->fb_format->format);
	}

no_simplefb:
	of_node_put(of_node);

	mutex_init(&splash->lock);
	if (drm_client_init(dev, &splash->client, "drm_splash",
			    &drm_splash_client_funcs))
		goto err_free;

	drm_client_register(&splash->client);
	drm_dbg(dev, "Registered with drm splash");

	return;

err_free:
	kfree(splash);
err_warn:
	drm_warn(dev, "Failed to register with drm splash");
}
