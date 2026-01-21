// SPDX-License-Identifier: GPL-2.0 or MIT
/*
 * Based on drm_log from:
 * Copyright (c) 2024 Red Hat.
 * Author: Jocelyn Falempe <jfalempe@redhat.com>
 */

#include <linux/console.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/iosys-map.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/firmware.h>
#include <linux/console.h>
#include <linux/vt_kern.h>
#include <linux/vt_buffer.h>
#include <linux/input.h>
#include <linux/platform_data/simplefb.h>
#include <linux/of_reserved_mem.h>

#include <drm/drm_client.h>
#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>

#include "drm_client_internal.h"
#include "drm_draw_internal.h"
#include "drm_internal.h"

#include "vrterminal.h"
#include "vrterminal_drawing.h"

struct drm_vrconsole_scanout {
	struct drm_client_buffer *buffer;
	const struct font_desc *font;
};

struct drm_vrconsole {
	struct mutex lock;
	struct drm_client_dev client;
	bool registered;
	bool probed;
	u32 n_scanout;
	struct drm_vrconsole_scanout *scanout;
	int fb_width;
	int fb_height;
	int fb_stride;
	const struct drm_format_info *fb_format;
	void *fb_base;
	size_t fb_size;
	const struct firmware *vrfont_fw;
	const struct firmware *vrterminal_fw;
	VRFont_t vrfont;
	VRTerminal_t vrterminal;
	TerminalUpdate_t terminalUpdate;
	VRStereoBounds_t terminalBounds;
	struct consw csw;
	struct work_struct update_work;
	struct input_handler input_handler;
	bool switch_to_console;
	bool console_enabled;
};

static struct input_handler input_switch_handler;

static struct drm_vrconsole *client_to_drm_vrconsole(struct drm_client_dev *client)
{
	return container_of(client, struct drm_vrconsole, client);
}

static struct drm_vrconsole *consw_to_drm_vrconsole(const struct consw *csw)
{
	return container_of(csw, struct drm_vrconsole, csw);
}

static struct drm_vrconsole *input_handler_to_drm_vrconsole(struct input_handler *handler)
{
	return container_of(handler, struct drm_vrconsole, input_handler);
}

static int drm_vrconsole_copy_simplefb(struct drm_vrconsole *dvrconsole,
				    struct drm_vrconsole_scanout *scanout)
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
			dvrconsole->fb_base + (dvrconsole->fb_stride * line),
			min_t(unsigned int, fb->pitches[0], dvrconsole->fb_stride));

	drm_client_buffer_vunmap_local(scanout->buffer);

	return drm_client_buffer_flush(scanout->buffer, &r);
}

static int drm_vrconsole_setup_modeset(struct drm_vrconsole *dvrconsole,
				 struct drm_client_dev *client,
				 struct drm_mode_set *mode_set,
				 struct drm_vrconsole_scanout *scanout)
{
	u32 width = dvrconsole->fb_width;
	u32 height = dvrconsole->fb_height;
	u32 format = DRM_FORMAT_XRGB8888;

	scanout->buffer = drm_client_buffer_create_dumb(client, width, height, format);
	if (IS_ERR(scanout->buffer)) {
		drm_warn(client->dev, "drm_vrconsole can't create framebuffer %d %d %p4cc\n",
			 width, height, &format);
		return -ENOMEM;
	}
	mode_set->fb = scanout->buffer->fb;

	return 0;
}

static int drm_vrconsole_count_modeset(struct drm_client_dev *client)
{
	struct drm_mode_set *mode_set;
	int count = 0;

	mutex_lock(&client->modeset_mutex);
	drm_client_for_each_modeset(mode_set, client)
		count++;
	mutex_unlock(&client->modeset_mutex);
	return count;
}

static void drm_vrconsole_init_client(struct drm_vrconsole *dvrconsole)
{
	struct drm_client_dev *client = &dvrconsole->client;
	struct drm_mode_set *mode_set;
	int i, max_modeset;
	int n_modeset = 0;

	if (drm_client_modeset_probe(client, 0, 0))
		return;

	max_modeset = drm_vrconsole_count_modeset(client);
	if (!max_modeset)
		return;

	dvrconsole->scanout = kcalloc(max_modeset, sizeof(*dvrconsole->scanout), GFP_KERNEL);
	if (!dvrconsole->scanout)
		return;

	mutex_lock(&client->modeset_mutex);
	drm_client_for_each_modeset(mode_set, client) {
		if (!mode_set->mode)
			continue;
		if (drm_vrconsole_setup_modeset(dvrconsole, client, mode_set, &dvrconsole->scanout[n_modeset]))
			continue;
		n_modeset++;
	}
	mutex_unlock(&client->modeset_mutex);
	if (n_modeset == 0)
		goto err_nomodeset;

	if (!dvrconsole->console_enabled) {
		for (i = 0; i < n_modeset; i++) {
			if (!dvrconsole->scanout[i].buffer)
				continue;

			drm_vrconsole_copy_simplefb(dvrconsole, &dvrconsole->scanout[i]);
		}
	}

	if (drm_client_modeset_commit(client))
		goto err_failed_commit;

	dvrconsole->n_scanout = n_modeset;
	dvrconsole->probed = true;

	return;

err_failed_commit:
	for (i = 0; i < n_modeset; i++)
		drm_client_buffer_delete(dvrconsole->scanout[i].buffer);

err_nomodeset:
	kfree(dvrconsole->scanout);
	dvrconsole->scanout = NULL;
}

static void drm_vrconsole_free_scanout(struct drm_client_dev *client)
{
	struct drm_vrconsole *dvrconsole = client_to_drm_vrconsole(client);
	int i;

	if (dvrconsole->n_scanout) {
		for (i = 0; i < dvrconsole->n_scanout; i++) {
			drm_client_buffer_vunmap_local(dvrconsole->scanout[i].buffer);
			drm_client_buffer_delete(dvrconsole->scanout[i].buffer);
		}
		dvrconsole->n_scanout = 0;
		kfree(dvrconsole->scanout);
		dvrconsole->scanout = NULL;
		dvrconsole->probed = false;
	}
}

static void drm_vrconsole_client_unregister(struct drm_client_dev *client)
{
	struct drm_vrconsole *dvrconsole = client_to_drm_vrconsole(client);
	struct drm_device *dev = client->dev;

	input_unregister_handler(&dvrconsole->input_handler);

	console_lock();
	do_unbind_con_driver(&dvrconsole->csw, 0, MAX_NR_CONSOLES - 1, 1);
	do_unregister_con_driver(&dvrconsole->csw);
	console_unlock();

	release_firmware(dvrconsole->vrfont_fw);
	release_firmware(dvrconsole->vrterminal_fw);

	cancel_work_sync(&dvrconsole->update_work);

	mutex_lock(&dvrconsole->lock);
	drm_vrconsole_free_scanout(client);
	drm_client_release(client);
	mutex_unlock(&dvrconsole->lock);
	kfree(dvrconsole);
	drm_info(dev, "Unregistered with drm vrconsole\n");
}

static int drm_vrconsole_client_hotplug(struct drm_client_dev *client)
{
	struct drm_vrconsole *dvrconsole = client_to_drm_vrconsole(client);

	drm_info(client->dev, "Hotplug drm vrconsole\n");

	mutex_lock(&dvrconsole->lock);
	drm_vrconsole_free_scanout(client);
	if (!dvrconsole->console_enabled)
		drm_vrconsole_init_client(dvrconsole);
	mutex_unlock(&dvrconsole->lock);

	return 0;
}

static const struct drm_client_funcs drm_vrconsole_client_funcs = {
       .owner          = THIS_MODULE,
       .unregister     = drm_vrconsole_client_unregister,
       .hotplug        = drm_vrconsole_client_hotplug,
};

static void drm_vrconsole_update(struct drm_vrconsole *dvrconsole,
				 struct drm_vrconsole_scanout *scanout)
{
	struct iosys_map map;
	struct drm_rect r;
	Pixbuf pb;
	int ret;

	ret = drm_client_buffer_vmap_local(scanout->buffer, &map);
	if (ret) {
		drm_warn(dvrconsole->client.dev, "Terminal failed to vmap (%d)\n", ret);
		return;
	}

	pb.w = scanout->buffer->fb->width * scanout->buffer->fb->format->cpp[0];
	pb.h = scanout->buffer->fb->height;
	pb.stride = scanout->buffer->fb->pitches[0];
	pb.map = &map;

	if (dvrconsole->switch_to_console)
		clear_pixbuf(&pb, pb.w, pb.h);

	if ( dvrconsole->terminalUpdate.full_update )
	{
		clear_stereo_bounds( &pb, &dvrconsole->terminalBounds );
		for ( int row = 0; row < dvrconsole->vrterminal.height ; ++row )
		{
			for ( int col = 0; col < dvrconsole->vrterminal.width; ++col )
			{
				const VRCell_t *cell = &dvrconsole->vrterminal.vCells[ row * dvrconsole->vrterminal.width + col ];
				if (cell->ch == 0 || cell->ch == ' ')
					continue;
				VRTerminalPoint_t * pTerminalPoint = vrterminal_get_point( &dvrconsole->vrterminal, col, row );
				draw_glyph( &pb, pTerminalPoint, &dvrconsole->vrfont, cell->ch, cell->attr );
			}
		}
	}
	else
	{
		// Otherwise add only the new characters
		for ( int nUpdate = 0; nUpdate < (int) dvrconsole->terminalUpdate.update_count; ++nUpdate )
		{
			int row = dvrconsole->terminalUpdate.vUpdates[nUpdate].y;
			int col = dvrconsole->terminalUpdate.vUpdates[nUpdate].x;
			const VRCell_t *cell = &dvrconsole->vrterminal.vCells[ row * dvrconsole->vrterminal.width + col ];
			if (cell->ch == 0 || cell->ch == ' ')
				continue;
			VRTerminalPoint_t * pTerminalPoint = vrterminal_get_point( &dvrconsole->vrterminal, col, row );
			draw_glyph( &pb, pTerminalPoint, &dvrconsole->vrfont, cell->ch, cell->attr );
		}
	}

	// TODO: only damage the actual region changed
	for ( int neye = 0; neye<2; ++neye )
	{
		r = DRM_RECT_INIT(dvrconsole->terminalBounds.min_x[neye]/10, dvrconsole->terminalBounds.min_y[neye]/10,
				  dvrconsole->terminalBounds.max_x[neye]/10 - dvrconsole->terminalBounds.min_x[neye]/10,
				  dvrconsole->terminalBounds.max_y[neye]/10 - dvrconsole->terminalBounds.min_y[neye]/10);
		drm_client_buffer_flush(scanout->buffer, &r);
	}

	drm_client_buffer_vunmap_local(scanout->buffer);
}

static void vrcon_terminal_update_work(struct work_struct *work)
{
	struct drm_vrconsole *dvrconsole = container_of(work, struct drm_vrconsole,
							update_work);
	int i;

	if (!dvrconsole->console_enabled &&
	    !dvrconsole->switch_to_console)
		return;

	console_lock();

	if (!dvrconsole->switch_to_console &&
	    !dvrconsole->terminalUpdate.full_update &&
	    !dvrconsole->terminalUpdate.update_count) {
		console_unlock();
		return;
	}

	if (dvrconsole->switch_to_console)
		dvrconsole->console_enabled = 1;

	if (!dvrconsole->probed) {
		drm_vrconsole_init_client(dvrconsole);
		dvrconsole->terminalUpdate.full_update = 1;
	}

	if (drm_master_internal_acquire(dvrconsole->client.dev)) {
		drm_master_internal_release(dvrconsole->client.dev);

		for (i = 0; i < dvrconsole->n_scanout; i++)
			drm_vrconsole_update(dvrconsole, &dvrconsole->scanout[i]);
	}

	vrterminal_update_reset( &dvrconsole->terminalUpdate );

	if (dvrconsole->switch_to_console)
		dvrconsole->switch_to_console = 0;

	console_unlock();
}

static void vrcon_terminal_update(struct drm_vrconsole *dvrconsole)
{
	schedule_work(&dvrconsole->update_work);
}

static void vrcon_terminal_putcs(struct drm_vrconsole *dvrconsole,
				 const u16 *s, unsigned int count,
				 unsigned int ypos, unsigned int xpos)
{
	u16 c;

	while (count--) {
		c = scr_readw(s++);
		vrterminal_putc(&dvrconsole->terminalUpdate, &dvrconsole->vrterminal,
				xpos, ypos, c & 0xff, c >> 8);
		xpos++;
	}
}

static void vrcon_terminal_clear(struct drm_vrconsole *dvrconsole,
				 unsigned int sy, unsigned int sx,
				 unsigned int width)
{
	while (width--)
		vrterminal_putc(&dvrconsole->terminalUpdate, &dvrconsole->vrterminal,
				sx++, sy, 0, 0);
}

static void vrcon_terminal_scroll_up(struct drm_vrconsole *dvrconsole)
{
	vrterminal_scroll_up(&dvrconsole->terminalUpdate, &dvrconsole->vrterminal);
}

static void vrcon_putcs(struct vc_data *vc, const u16 *s, unsigned int count,
			unsigned int ypos, unsigned int xpos)
{
	struct drm_vrconsole *dvrconsole = consw_to_drm_vrconsole(vc->vc_sw);

	vrcon_terminal_putcs(dvrconsole, s, count, ypos, xpos);
	vrcon_terminal_update(dvrconsole);
}

static bool vrcon_blank(struct vc_data *vc, enum vesa_blank_mode blank,
			   bool mode_switch)
{
	return true;
}

static bool vrcon_switch(struct vc_data *vc)
{
	return true;
}

static const char *vrcon_startup(void)
{
	return "vrconsole";
}

static void vrcon_init(struct vc_data *vc, bool init)
{
	struct drm_vrconsole *dvrconsole = consw_to_drm_vrconsole(vc->vc_sw);

	vc->vc_can_do_color = 1;
	if (init) {
		vc->vc_cols = dvrconsole->vrterminal.width;
		vc->vc_rows = dvrconsole->vrterminal.height;
	} else
		vc_resize(vc, dvrconsole->vrterminal.width,
			  dvrconsole->vrterminal.height);
}

static void vrcon_deinit(struct vc_data *vc)
{
}

static void vrcon_clear(struct vc_data *vc, unsigned int sy, unsigned int sx,
			   unsigned int width)
{
	struct drm_vrconsole *dvrconsole = consw_to_drm_vrconsole(vc->vc_sw);

	vrcon_terminal_clear(dvrconsole, sy, sx, width);
	vrcon_terminal_update(dvrconsole);
}

// Show/hide cursor
static void vrcon_cursor(struct vc_data *vc, bool enable)
{
	/* TOFIX add support for showing current cursor (+ blinking) */
}

static bool vrcon_scroll(struct vc_data *vc, unsigned int top,
			    unsigned int bottom, enum con_scroll dir,
			    unsigned int lines)
{
	struct drm_vrconsole *dvrconsole = consw_to_drm_vrconsole(vc->vc_sw);

	/* TOFIX add scroll down */
	if (dir != SM_UP)
		return false;

	vrcon_terminal_scroll_up(dvrconsole);
	vrcon_terminal_update(dvrconsole);

	return true;
}

static u8 vrcon_build_attr(struct vc_data *c, u8 color,
			   enum vc_intensity intensity,
			   bool blink, bool underline, bool reverse,
			   bool italic)
{
	u8 attr = (color & 7);

	if (intensity == VCI_BOLD)
		attr |= (1 << 4);
	if (intensity == VCI_HALF_BRIGHT)
		attr |= (1 << 5);

	// TODO reverse

	return attr;
}

const struct consw vr_con = {
	.owner =	THIS_MODULE,
	.con_startup =	vrcon_startup,
	.con_init =	vrcon_init,
	.con_deinit =	vrcon_deinit,
	.con_clear =	vrcon_clear,
	// putc
	.con_putcs =	vrcon_putcs,
	.con_cursor =	vrcon_cursor,
	.con_scroll =	vrcon_scroll,
	.con_switch =	vrcon_switch,
	.con_blank =	vrcon_blank,
	// font...
	// resize
	// set_palette
	// scrolldelta
	// set_origin
	// save_screen
	.con_build_attr	= vrcon_build_attr,
	// invert_region
	// debug...
};

static int input_switch_connect(struct input_handler *handler, struct input_dev *dev,
			const struct input_device_id *id)
{
	struct drm_vrconsole *dvrconsole = input_handler_to_drm_vrconsole(handler);
	struct input_handle *handle;
	int error;

	/* If we already have switched, ignore new matching devices */
	if (dvrconsole->console_enabled || dvrconsole->switch_to_console)
		return 0;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "kbd";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	return 0;

 err_unregister_handle:
	input_unregister_handle(handle);
 err_free_handle:
	kfree(handle);
	return error;
}

static void input_switch_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static bool input_switch_filter(struct input_handle *handle, unsigned int event_type,
				unsigned int event_code, int value)
{
	struct drm_vrconsole *dvrconsole = input_handler_to_drm_vrconsole(handle->handler);

	/* If we already have switched, ignore any key input */
	if (dvrconsole->console_enabled || dvrconsole->switch_to_console)
		return false;

	if ((event_code == KEY_SELECT || event_code == KEY_ESC) && value) {
		pr_info("%s: do the switch\n", __func__);
		dvrconsole->switch_to_console = true;
		schedule_work(&dvrconsole->update_work);
	}

	/* Still pass event to other clients */
	return false;
}

static bool input_switch_match(struct input_handler *handler, struct input_dev *dev)
{
	struct drm_vrconsole *dvrconsole = input_handler_to_drm_vrconsole(handler);

	/* If we already have switched, stop matching new input devices */
	if (dvrconsole->console_enabled || dvrconsole->switch_to_console)
		return false;

	if (test_bit(EV_KEY, dev->evbit) && test_bit(KEY_SELECT, dev->keybit))
			return true;
	if (test_bit(EV_KEY, dev->evbit) && test_bit(KEY_ESC, dev->keybit))
			return true;

	return false;
}

static void input_switch_start(struct input_handle *handle)
{
	struct drm_vrconsole *dvrconsole = input_handler_to_drm_vrconsole(handle->handler);

	/* If we already have switched, return the start state and disconnect*/
	if (dvrconsole->console_enabled || dvrconsole->switch_to_console)
		return;

	if (test_bit(KEY_SELECT, handle->dev->key) ||
	     test_bit(KEY_ESC, handle->dev->key)) {
		pr_info("%s: do the switch\n", __func__);
		dvrconsole->switch_to_console = true;
		schedule_work(&dvrconsole->update_work);
	}
}

static const struct input_device_id input_switch_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },    /* Terminating entry */
};

MODULE_DEVICE_TABLE(input, input_switch_ids);

static struct input_handler input_switch_handler = {
	.filter		= input_switch_filter,
	.match		= input_switch_match,
	.connect	= input_switch_connect,
	.disconnect	= input_switch_disconnect,
	.start		= input_switch_start,
	.name		= "input_switch",
	.id_table	= input_switch_ids,
};

static int drm_vrconsole_get_width_of(struct device_node *of_node)
{
	u32 width;

	int ret = of_property_read_u32(of_node, "width", &width);
	if (ret)
		return ret;

	return width;
}

static int drm_vrconsole_get_height_of(struct device_node *of_node)
{
	u32 height;

	int ret = of_property_read_u32(of_node, "height", &height);
	if (ret)
		return ret;

	return height;
}

static int drm_vrconsole_get_stride_of(struct device_node *of_node)
{
	u32 stride;

	int ret = of_property_read_u32(of_node, "stride", &stride);
	if (ret)
		return ret;

	return stride;
}

static const struct drm_format_info *
drm_vrconsole_get_validated_format(struct drm_device *dev, const char *format_name)
{
	static const struct simplefb_format formats[] = SIMPLEFB_FORMATS;
	const struct simplefb_format *fmt = formats;
	const struct simplefb_format *end = fmt + ARRAY_SIZE(formats);
	const struct drm_format_info *info;

	if (!format_name) {
		drm_err(dev, "drm_vrconsole: missing framebuffer format\n");
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

	drm_err(dev, "drm_vrconsole: unknown framebuffer format %s\n",
		format_name);

	return ERR_PTR(-EINVAL);
}

static const struct drm_format_info *
drm_vrconsole_get_format_of(struct drm_device *dev, struct device_node *of_node)
{
	const char *format;
	int ret;

	ret = of_property_read_string(of_node, "format", &format);
	if (ret)
		return ERR_PTR(ret);

	return drm_vrconsole_get_validated_format(dev, format);
}

static struct resource *
drm_vrconsole_get_memory_of(struct drm_device *dev, struct device_node *of_node)
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
 * drm_vrconsole_register() - Register a drm device to drm_vrconsole
 * @dev: the drm device to register.
 */
void drm_vrconsole_register(struct drm_device *dev)
{
	struct resource *mem = NULL;
	struct device_node *of_node;
	struct drm_vrconsole *new;
	int ret;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		goto err_warn;

	of_node = of_find_compatible_node(NULL, NULL, "simple-framebuffer");
	if (!of_node) {
		drm_warn(dev, "no simple-fb node\n");
		goto err_free;
	}

	if (!of_device_is_available(of_node)) {
		drm_warn(dev, "no simple-fb enabled node\n");
		goto err_free;
	}

	new->fb_width = drm_vrconsole_get_width_of(of_node);
	if (new->fb_width < 0)
		goto err_node_put;

	new->fb_height = drm_vrconsole_get_height_of(of_node);
	if (new->fb_height < 0)
		goto err_node_put;

	new->fb_stride = drm_vrconsole_get_stride_of(of_node);
	if (new->fb_stride < 0)
		goto err_node_put;

	new->fb_format = drm_vrconsole_get_format_of(dev, of_node);
	if (IS_ERR(new->fb_format))
		goto err_node_put;

	if (new->fb_format->format != DRM_FORMAT_ARGB8888 &&
	    new->fb_format->format != DRM_FORMAT_XRGB8888) {
		drm_warn(dev, "drm_vrconsole: invalid fb-simple format\n");
		goto err_node_put;
	}

	mem = drm_vrconsole_get_memory_of(dev, of_node);
	if (IS_ERR_OR_NULL(mem))
		goto err_node_put;

	new->fb_size = resource_size(mem);
	new->fb_base = devm_memremap(dev->dev, mem->start,
					new->fb_size, MEMREMAP_WC);
	if (IS_ERR(new->fb_base)) {
		new->fb_base = NULL;
		goto err_node_put;
	}

	drm_info(dev, "drm_vrconsole: found %dx%d/%d format %p4cc framebuffer\n",
		 new->fb_width, new->fb_height, new->fb_stride,
		 &new->fb_format->format);

	of_node_put(of_node);

	mutex_init(&new->lock);
	if (drm_client_init(dev, &new->client, "drm_vrconsole", &drm_vrconsole_client_funcs))
		goto err_free;

	/* Register drm client early, so it can show the splash while we register the console */
	drm_client_register(&new->client);

	/* TODO make it a module parameter */
	ret = request_firmware_direct(&new->vrfont_fw, "default.vrfont",
				      dev->dev);
	if (ret) {
		drm_warn(dev, "failed to load vrfont (%d)\n", ret);
		goto err_warn_console;
	}

	/* TODO make it a module parameter */
	ret = request_firmware_direct(&new->vrterminal_fw, "default.vrterminal",
				      dev->dev);
	if (ret) {
		drm_warn(dev, "failed to load vrterminal (%d)\n", ret);
		goto err_warn_console;
	}

	// Init the font and the terminal
	ret = vrfont_init(&new->vrfont, new->vrfont_fw->data,
			  new->vrfont_fw->size );
	if (ret) {
		drm_warn(dev, "Font failed init (%d)\n", ret);
		goto err_warn_console;
	}

	ret = vrterminal_init(&new->vrterminal, new->vrterminal_fw->data,
			       new->vrterminal_fw->size);
	if (ret) {
		drm_warn(dev, "Terminal failed init (%d)\n", ret);
		goto err_warn_console;
	}

	memcpy(&new->terminalBounds, &new->vrterminal.terminalBounds,
	       sizeof(new->terminalBounds));

	// expand terminalBounds by the glyph size
	// this is overly conservative given a centered glyph (/2 would be better)
	// but this accounts for non-center offsets
	// the * 10 accounts for pixel -> subpixel
	vrbounds_pad( &new->terminalBounds,
			( (int) new->vrfont.maxGlyphWidth ) * 10, ( (int) new->vrfont.maxGlyphWidth ) * 10,
			( (int) new->vrfont.maxGlyphHeight ) * 10, ( (int) new->vrfont.maxGlyphHeight ) * 10 );

	INIT_WORK(&new->update_work, vrcon_terminal_update_work);

	memcpy(&new->csw, &vr_con, sizeof(vr_con));

	console_lock();
	ret = do_take_over_console(&new->csw, 0, MAX_NR_CONSOLES - 1, 1);
	console_unlock();
	if (ret)
		goto err_warn_console;

	/* Register input handler as the very end when we are sure the console is registered */
	memcpy(&new->input_handler, &input_switch_handler, sizeof(input_switch_handler));

	ret = input_register_handler(&new->input_handler);
	if (ret)
		drm_warn(dev, "failed to register input handler\n");

	drm_dbg(dev, "Registered with vt\n");
	return;

err_node_put:
	of_node_put(of_node);
err_free:
	kfree(new);
err_warn:
	drm_warn(dev, "Failed to register with drm vrconsole\n");
	return;
err_warn_console:
	drm_warn(dev, "failed to register vt console\n");
	return;
}

MODULE_DESCRIPTION("DRM VR VT console");
MODULE_LICENSE("GPL");
