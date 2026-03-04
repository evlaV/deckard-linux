#ifndef VRTERMINAL_DRAWING
#define VRTERMINAL_DRAWING

#define LINUX_DRM_VRCONSOLE 
#ifdef LINUX_DRM_VRCONSOLE
#include <linux/iosys-map.h>
#endif

/* -------- framebuffer pixbuf -------- */
typedef struct {
#ifdef LINUX_DRM_VRCONSOLE
	struct iosys_map *map;
#else
	uint8_t *px;   // RGBA8888
#endif
	int w, h, stride;
} Pixbuf;

void clear_pixbuf( Pixbuf *pb, int width, int height );
void clear_stereo_bounds( Pixbuf *pb, VRStereoBounds_t *bounds );
void draw_glyph( Pixbuf *pb, VRTerminalPoint_t * pTerminalPoint, VRFont_t * pFont, char codepoint, char attr );

#endif
