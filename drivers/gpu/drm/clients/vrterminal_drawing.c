#include "vrterminal.h"
#include "vrterminal_drawing.h"

#ifdef LINUX_DRM_VRCONSOLE
#include "drm_draw_internal.h"
#else
#include <string.h>
#endif


static inline int clamp_int( int val, int minval, int maxval )
{
    return ( val < minval ) ? minval : ( ( val > maxval ) ? maxval : val );
}

static inline void pset_v( Pixbuf *pb, int x, int y, uint8_t value, int channel )
{
#ifdef VRCONSOLE_GLYPH_BOUNDS_CHECKING	
	if ((unsigned)x >= (unsigned)pb->w || (unsigned)y >= (unsigned)pb->h || x<0 || y < 0 ) return;
#endif
#ifdef LINUX_DRM_VRCONSOLE
	iosys_map_wr(pb->map, (size_t)y * pb->stride + (size_t)x*4 + channel, u8, value);
#else
	uint8_t *p = pb->px + (size_t)y * pb->stride + (size_t)x*4;
	p[channel]=value;
#endif
}


void clear_pixbuf(Pixbuf *pb, int width, int height)
{
#ifdef LINUX_DRM_VRCONSOLE
	iosys_map_memset(pb->map, 0, 0, pb->h * pb->stride);
#else
	for(int y=0; y < height; y++)
	{
		uint8_t *row = pb->px + y * pb->stride;
		memset( row, 0, width * 4 );
	}
#endif
}

void clear_stereo_bounds( Pixbuf *pb, VRStereoBounds_t *bounds )
{
	for ( int neye=0; neye<2; ++neye )
	{
		// Check for valid bounds
		if ( bounds->max_x[neye] <= bounds->min_x[neye] ||
			bounds->max_y[neye] <= bounds->min_y[neye] )
		{
			continue;
		}

		int xmin = clamp_int( bounds->min_x[neye]/10, 0, pb->w );
		int xmax = clamp_int( bounds->max_x[neye]/10 + 1, 0, pb->w );
		int ymin = clamp_int( bounds->min_y[neye]/10, 0, pb->h );
		int ymax = clamp_int( bounds->max_y[neye]/10 + 1, 0, pb->h );
		int xwidth = xmax - xmin;

		for(int y=ymin; y < ymax; y++)
		{
#ifdef LINUX_DRM_VRCONSOLE
			iosys_map_memset(pb->map, 4 * xmin + y * pb->stride, 0, xwidth * 4);
#else
			uint8_t *row = ( pb->px + 4 * xmin + y * pb->stride);
			memset( row, 0, xwidth * 4 );
#endif
		}
	}
}

//#define DEBUG_RENDER

#ifdef DEBUG_RENDER

static const int k_nDotGain = 128;

void draw_glyph_debug( Pixbuf *pb, VRTerminalPoint_t * pTerminalPoint, VRFont_t * pFont, char codepoint, int nFrame )
{
	if ( !pTerminalPoint || !pFont )
		return;

	//bool bDrawDot = false; //  nFrame % 10 < 3;
	bool bDrawChar = true; // nFrame % 10 >= 3;

	for ( int neye = 0; neye < 2; ++neye )
	{
		int nCx = (pTerminalPoint->green_x[neye]+5)/10;
		int nCy = (pTerminalPoint->green_y[neye]+5)/10;

		// cross-hairs
		/*
		for ( int y=0; y<pGlyph->nHeight; ++y )
		{
			pset_v( pb, nCx, nCy+ y - pGlyph->nHeight/2, k_nCrosshairsGain, 1 );
		}
		for ( int x=0; x<pGlyph->nWidth; ++x )
		{
			pset_v( pb, nCx + x - pGlyph->nWidth/2, nCy, k_nCrosshairsGain, 1 );
		}*/

		// center dot
		if ( bDrawDot )
		{
			for ( int dy = -1; dy<=1; ++dy)
			{
				for ( int dx = -1; dx<=1; ++dx)
				{
					pset_v(pb, nCx+dx, nCy+dy, 0, 1 );
				}
			}

			pset_v(pb, nCx, nCy, k_nDotGain, 1 );
		}
	}
}
#endif

static u8 intensity_from_attr(unsigned int i)
{
	bool bold = i & (1 << 4);
	bool half = i & (1 << 5);

	return (bold ? 0xff : (half ? 0x7f : 0xaa));
}

static void rgb_from_attr(unsigned int i, unsigned int *c)
{
	u8 intensity = intensity_from_attr(i);

	c[0] = i&1 ? intensity : 0;
	c[1] = i&2 ? intensity : 0;
	c[2] = i&4 ? intensity : 0;
}

void draw_glyph( Pixbuf *pb, VRTerminalPoint_t * pTerminalPoint, VRFont_t * pFont, char codepoint, char attr )
{
	unsigned int rgb[3];

	if ( !pTerminalPoint || !pFont )
		return;

	rgb_from_attr(attr, rgb);

	for ( int neye = 0; neye < 2; ++neye )
	{
		VRGlyph_t * pGlyph = vrfont_find_vrglyph( pFont, codepoint, neye );
		if ( pGlyph == NULL )
			continue;

		//BGR ordering
		int xPos[3] = { (pTerminalPoint->blue_x[neye]+5)/10, (pTerminalPoint->green_x[neye]+5)/10, (pTerminalPoint->red_x[neye]+5)/10 };
		int yPos[3] = { (pTerminalPoint->blue_y[neye]+5)/10, (pTerminalPoint->green_y[neye]+5)/10, (pTerminalPoint->red_y[neye]+5)/10 };
		
		// TODO: bounds check character
		const unsigned char * pData = vrfont_find_vrglyph_pixeldata( pFont, pGlyph );
		for ( int y=0; y<pGlyph->nHeight; ++y )
		{
			const unsigned char * pGlyphRowPtr = pData + y*pGlyph->nWidth;

			for ( int x=0; x<pGlyph->nWidth; ++x )
			{
				unsigned char nValue = *pGlyphRowPtr;
				static const uint8_t k_nMinValueToBlit = 1;
				if ( nValue > k_nMinValueToBlit )
				{

					for ( int nChannel = 0; nChannel<3; ++nChannel )
					{
						unsigned value = ((unsigned int)nValue * (unsigned int)(rgb[nChannel] + 1)) >> 8;
						pset_v( pb,
							xPos[nChannel] + x - pGlyph->nOriginX,
							yPos[nChannel] + y - pGlyph->nOriginY,
							value, nChannel );
					}
				}
				
				++pGlyphRowPtr;
			}
		}
	}
}
