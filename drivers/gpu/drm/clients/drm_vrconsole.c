// SPDX-License-Identifier: GPL-2.0 or MIT
/*
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
#include <linux/tty.h>

#include <drm/drm_client.h>
#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>

#include "drm_client_internal.h"
#include "drm_internal.h"

#include "vrterminal.h"
#include "vrterminal_drawing.h"

MODULE_AUTHOR("Jocelyn Falempe");
MODULE_DESCRIPTION("DRM VR boot logger");
MODULE_LICENSE("GPL");

/**
 * DOC: overview
 *
 * This is a simple graphic logger, to print the kernel message on screen, until
 * a userspace application is able to take over.
 * It is only for debugging purpose.
 */

struct drm_vrconsole_scanout {
	struct drm_client_buffer *buffer;
	const struct font_desc *font;
	VRFont_t vrfont;
	VRTerminal_t vrterminal;
};

struct drm_vrconsole {
	struct mutex lock;
	struct drm_client_dev client;
	bool registered;
	bool probed;
	u32 n_scanout;
	struct drm_vrconsole_scanout *scanout;
	const struct firmware *vrfont;
	const struct firmware *vrterminal;
	struct tty_driver *tty;
	struct console console;
	struct tty_port port;
};

static struct drm_vrconsole *client_to_drm_vrconsole(struct drm_client_dev *client)
{
	return container_of(client, struct drm_vrconsole, client);
}

static void drm_vrconsole_draw_kmsg_record(struct drm_vrconsole_scanout *scanout,
				     const char *s, unsigned int len)
{
	struct drm_framebuffer *fb = scanout->buffer->fb;
	u32 px_width = fb->format->cpp[0];
	struct iosys_map map;
	VRFont_t * pFont = &scanout->vrfont;
	VRTerminal_t * pTerminal = &scanout->vrterminal;
	struct drm_rect r;
	Pixbuf pb;

	if (drm_client_buffer_vmap_local(scanout->buffer, &map))
		return;

	pb.w = fb->width * px_width;
	pb.h = fb->height;
	pb.stride = fb->pitches[0];
	pb.map = &map;
	
	int nTerminalSizeX = pTerminal->width;
	int nTerminalSizeY = pTerminal->height;

	static TerminalUpdate_t terminalUpdate;
	static VRStereoBounds_t terminalBounds;

	vrterminal_update_reset( &terminalUpdate );

	static int first_draw = 1;
	if ( first_draw )
	{
		first_draw = 0;
		terminalBounds = pTerminal->terminalBounds;

		// expand terminalBounds by the glyph size
		// this is overly conservative given a centered glyph (/2 would be better)
		// but this accounts for non-center offsets
		// the * 10 accounts for pixel -> subpixel
		vrbounds_pad( &terminalBounds,
			( (int) pFont->maxGlyphWidth ) * 10, ( (int) pFont->maxGlyphWidth ) * 10,
			( (int) pFont->maxGlyphHeight ) * 10, ( (int) pFont->maxGlyphHeight ) * 10 );

		// clear the full frame on first draw
		clear_pixbuf( &pb, pb.w, pb.h );
	}

	vrterminal_feed( &terminalUpdate, pTerminal, s, len );

	if ( terminalUpdate.full_update )
	{
		clear_stereo_bounds( &pb, &terminalBounds );
		for ( int row = 0; row < nTerminalSizeY ; ++row )
		{
			for ( int col = 0; col < nTerminalSizeX; ++col )
			{
				const VRCell_t *cell = &pTerminal->vCells[ row * nTerminalSizeX + col ];
				if (cell->ch == 0 || cell->ch == ' ')
					continue;
				VRTerminalPoint_t * pTerminalPoint = vrterminal_get_point( pTerminal, col, row );
				draw_glyph( &pb, pTerminalPoint, pFont, cell->ch );
			}
		}
	}
	else
	{
		// Otherwise add only the new characters
		for ( int nUpdate = 0; nUpdate < (int) terminalUpdate.update_count; ++nUpdate )
		{
			int row = terminalUpdate.vUpdates[nUpdate].y;
			int col = terminalUpdate.vUpdates[nUpdate].x;
			const VRCell_t *cell = &pTerminal->vCells[ row * nTerminalSizeX + col ];
			if (cell->ch == 0 || cell->ch == ' ')
				continue;
			VRTerminalPoint_t * pTerminalPoint = vrterminal_get_point( pTerminal, col, row );
			draw_glyph( &pb, pTerminalPoint, pFont, cell->ch );
		}
	}

	drm_client_buffer_vunmap_local(scanout->buffer);

	// TODO: only damage the actual region changed
	// (though this isnt used right now on steam frame)
	for ( int neye = 0; neye<2; ++neye )
	{
		r = DRM_RECT_INIT(terminalBounds.min_x[neye]/10, terminalBounds.min_y[neye]/10,
				  terminalBounds.max_x[neye]/10 - terminalBounds.min_x[neye]/10,
				  terminalBounds.max_y[neye]/10 - terminalBounds.min_y[neye]/10);
		drm_client_framebuffer_flush(scanout->buffer, &r);
	}
}

static u32 drm_vrconsole_find_usable_format(struct drm_plane *plane)
{
	int i;

	for (i = 0; i < plane->format_count; i++)
		if (plane->format_types[i] == DRM_FORMAT_ARGB8888)
			return plane->format_types[i];
	return DRM_FORMAT_INVALID;
}

static int drm_vrconsole_setup_modeset(struct drm_vrconsole *dvrconsole,
				 struct drm_client_dev *client,
				 struct drm_mode_set *mode_set,
				 struct drm_vrconsole_scanout *scanout)
{
	struct drm_crtc *crtc = mode_set->crtc;
	u32 width = mode_set->mode->hdisplay;
	u32 height = mode_set->mode->vdisplay;
	u32 format;
	int ret;

	format = drm_vrconsole_find_usable_format(crtc->primary);
	if (format == DRM_FORMAT_INVALID)
		return -EINVAL;

	scanout->buffer = drm_client_framebuffer_create(client, width, height, format);
	if (IS_ERR(scanout->buffer)) {
		drm_warn(client->dev, "drm_vrconsole can't create framebuffer %d %d %p4cc\n",
			 width, height, &format);
		return -ENOMEM;
	}
	mode_set->fb = scanout->buffer->fb;

	// Init the font and the terminal
	ret = vrfont_init(&scanout->vrfont, dvrconsole->vrfont->data,
			  dvrconsole->vrfont->size );
	if (ret) {
		drm_warn(client->dev, "Font failed init (%d)\n", ret);
		return -EINVAL;
	}

	ret = vrterminal_init(&scanout->vrterminal, dvrconsole->vrterminal->data,
			       dvrconsole->vrterminal->size);
	if (ret) {
		drm_warn(client->dev, "Terminal failed init (%d)\n", ret);
		return -EINVAL;
	}

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

	dvrconsole->probed = true;

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

	if (drm_client_modeset_commit(client))
		goto err_failed_commit;

	dvrconsole->n_scanout = n_modeset;
	return;

err_failed_commit:
	for (i = 0; i < n_modeset; i++)
		drm_client_framebuffer_delete(dvrconsole->scanout[i].buffer);

err_nomodeset:
	kfree(dvrconsole->scanout);
	dvrconsole->scanout = NULL;
}

static void drm_vrconsole_free_scanout(struct drm_client_dev *client)
{
	struct drm_vrconsole *dvrconsole = client_to_drm_vrconsole(client);
	int i;

	if (dvrconsole->n_scanout) {
		for (i = 0; i < dvrconsole->n_scanout; i++)
			drm_client_framebuffer_delete(dvrconsole->scanout[i].buffer);
		dvrconsole->n_scanout = 0;
		kfree(dvrconsole->scanout);
		dvrconsole->scanout = NULL;
	}
}

static void drm_vrconsole_client_unregister(struct drm_client_dev *client)
{
	struct drm_vrconsole *dvrconsole = client_to_drm_vrconsole(client);
	struct drm_device *dev = client->dev;

	unregister_console(&dvrconsole->console);
	tty_unregister_driver(dvrconsole->tty);
	tty_driver_kref_put(dvrconsole->tty);
	tty_port_destroy(&dvrconsole->port);

	release_firmware(dvrconsole->vrfont);
	release_firmware(dvrconsole->vrterminal);

	mutex_lock(&dvrconsole->lock);
	drm_vrconsole_free_scanout(client);
	drm_client_release(client);
	mutex_unlock(&dvrconsole->lock);
	kfree(dvrconsole);
	drm_dbg(dev, "Unregistered with drm vrconsole\n");
}

static int drm_vrconsole_client_hotplug(struct drm_client_dev *client)
{
	struct drm_vrconsole *dvrconsole = client_to_drm_vrconsole(client);

	mutex_lock(&dvrconsole->lock);
	drm_vrconsole_free_scanout(client);
	dvrconsole->probed = false;
	mutex_unlock(&dvrconsole->lock);
	return 0;
}

static const struct drm_client_funcs drm_vrconsole_client_funcs = {
	.owner		= THIS_MODULE,
	.unregister	= drm_vrconsole_client_unregister,
	.hotplug	= drm_vrconsole_client_hotplug,
};

/*
 * TTY operations open function.
 */
static int vrconsole_open(struct tty_struct *tty, struct file *filp)
{
	struct drm_vrconsole *dvrconsole = tty->driver->driver_state;

	return tty_port_open(&dvrconsole->port, tty, filp);
}

/*
 * TTY operations close function.
 */
static void vrconsole_close(struct tty_struct *tty, struct file *filp)
{
	struct drm_vrconsole *dvrconsole = tty->driver->driver_state;

	tty_port_close(&dvrconsole->port, tty, filp);
}

/*
 * TTY operations write function.
 */
static ssize_t vrconsole_write(struct tty_struct *tty, const u8 *buf, size_t count)
{
	struct drm_vrconsole *dvrconsole = tty->driver->driver_state;
	int i;

	if (!dvrconsole->probed)
		drm_vrconsole_init_client(dvrconsole);

	/* Check that we are still the master before drawing */
	if (drm_master_internal_acquire(dvrconsole->client.dev)) {
		drm_master_internal_release(dvrconsole->client.dev);

		for (i = 0; i < dvrconsole->n_scanout; i++)
			drm_vrconsole_draw_kmsg_record(&dvrconsole->scanout[i], buf, count);
	}

	return count;
}

/*
 * TTY operations write_room function.
 */
static unsigned int vrconsole_write_room(struct tty_struct *tty)
{
	return 4096;
}

/*
 * TTY operations hangup function.
 */
static void vrconsole_hangup(struct tty_struct *tty)
{
	struct drm_vrconsole *dvrconsole = tty->driver->driver_state;

	tty_port_hangup(&dvrconsole->port);
}

/*
 * TTY port operations shutdown function.
 */
static void vrconsole_port_shutdown(struct tty_port *tport)
{
}

static const struct tty_operations vrconsole_ops = {
	.open = vrconsole_open,
	.close = vrconsole_close,
	.write = vrconsole_write,
	.write_room = vrconsole_write_room,
	.hangup = vrconsole_hangup,
};

static const struct tty_port_operations vrconsole_port_ops = {
	.shutdown = vrconsole_port_shutdown,
};

static struct tty_driver *vrconsole_console_device(struct console *c, int *index)
{
	struct drm_vrconsole *dvrconsole = c->data;

	*index = 0;
	return dvrconsole->tty;
}

/**
 * drm_vrconsole_register() - Register a drm device to drm_vrconsole
 * @dev: the drm device to register.
 */
void drm_vrconsole_register(struct drm_device *dev)
{
	struct drm_vrconsole *new;
	int ret;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		goto err_warn;

	ret = request_firmware_direct(&new->vrfont, "RobotoMono-Bold.vrfont",
				      dev->dev);
	if (ret) {
		drm_warn(dev, "failed to load vrfont (%d)\n", ret);
		goto err_free;
	}

	ret = request_firmware_direct(&new->vrterminal, "terminal_80x24.vrterminal",
				      dev->dev);
	if (ret) {
		drm_warn(dev, "failed to load vrterminal (%d)\n", ret);
		goto err_free_font;
	}

	mutex_init(&new->lock);
	if (drm_client_init(dev, &new->client, "drm_vrconsole", &drm_vrconsole_client_funcs))
		goto err_free_terminal;

	drm_client_register(&new->client);

	new->tty = tty_alloc_driver(1,
			TTY_DRIVER_RESET_TERMIOS |
			TTY_DRIVER_REAL_RAW |
			TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(new->tty))
		goto err_free_terminal;

	tty_port_init(&new->port);
	new->port.ops = &vrconsole_port_ops;

	new->tty->driver_name = "vrconsole";
	new->tty->name = "vrconsole";
	new->tty->major = TTYAUX_MAJOR;
	new->tty->minor_start = 3;
	new->tty->type = TTY_DRIVER_TYPE_CONSOLE;
	new->tty->init_termios = tty_std_termios;
	new->tty->init_termios.c_oflag = OPOST | OCRNL | ONOCR | ONLRET;
	new->tty->driver_state = new;
	tty_set_operations(new->tty, &vrconsole_ops);
	tty_port_link_device(&new->port, new->tty, 0);

	ret = tty_register_driver(new->tty);
	if (ret < 0) {
		drm_warn(dev, "failed to register vrtconsole driver (%d)\n", ret);
		goto err_free_tty;
	}

	strscpy(new->console.name, "vrconsole");
	new->console.device = vrconsole_console_device;
	new->console.data = new;
	register_console(&new->console);

	drm_dbg(dev, "Registered with drm vrconsole as %s\n", new->console.name);
	return;

err_free_tty:
	tty_driver_kref_put(new->tty);
	tty_port_destroy(&new->port);
err_free_terminal:
	release_firmware(new->vrterminal);
err_free_font:
	release_firmware(new->vrfont);
err_free:
	kfree(new);
err_warn:
	drm_warn(dev, "Failed to register with drm vrconsole\n");
}
