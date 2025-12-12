//========= Copyright Valve Corporation ============//

#include "vrterminal.h"
#ifdef VRTERMINAL_LINUX
#include <linux/string.h>
#include <linux/kstrtox.h>
#else
#include <string.h>
#endif

#ifdef VRTERMINAL_UBOOT
#include <ansi.h>
#include <exports.h>
#include <linux/ctype.h>
#include <linux/kconfig.h>
#elif defined(VRTERMINAL_LINUX)
#include <linux/ctype.h>
#include <linux/kconfig.h>
#else
#include <stdlib.h>
#include <ctype.h>
#endif

#ifndef INT32_MAX
#define INT32_MAX 0x7FFFFFFF
#endif

#ifndef INT32_MIN
#define INT32_MIN 0x80000000
#endif

static inline int32_t s32_min(int32_t a, int32_t b)
{
	return (a < b) ? a : b;
}

static inline int32_t s32_max(int32_t a, int32_t b)
{
	return (a > b) ? a : b;
}

static inline uint16_t u16_min(uint16_t a, uint16_t b)
{
	return (a < b) ? a : b;
}

static inline uint16_t u16_max(uint16_t a, uint16_t b)
{
	return (a > b) ? a : b;
}

static inline uint8_t u8_min(uint8_t a, uint8_t b)
{
	return (a < b) ? a : b;
}

static inline uint8_t u8_max(uint8_t a, uint8_t b)
{
	return (a > b) ? a : b;
}

static inline int is_visible_ascii( unsigned char c )
{
	return (c >= 0x21 && c <= 0x7E);  // '!' .. '~', excludes space
}

static char *parsenum(char *s, int *num)
{
	char *end;
#if defined(VRTERMINAL_UBOOT) || defined(VRTERMINAL_LINUX)
	*num = simple_strtol(s, &end, 10);
#else
	*num = strtol(s, &end, 10);
#endif
	return end;
}

#pragma pack( push, 1 )

// These structs must match the filesystem on disk representation
// for format version "1"
// Do not change without compat considerations

typedef struct VRFontHeader_t
{
	uint32_t nMagic;
	uint8_t nVersion;
	uint8_t nStartCodePoint;
	uint8_t nNumCodePoints;
	uint8_t reserved;
} VRFontHeader_t;

typedef struct VRTerminalHeader_t
{
	uint32_t nMagic;
	uint8_t nVersion;
	uint8_t reserved[3];
	uint16_t nTerminalSizeX;
	uint16_t nTerminalSizeY;
} VRTerminalHeader_t;

#pragma pack( pop )

#ifndef NULL
#define NULL 0
#endif


void vrbounds_pad( VRStereoBounds_t * pbounds, int nLeft, int nRight, int nTop, int nBottom )
{
	if ( !pbounds )
		return;

	for ( int neye=0; neye<2; ++neye )
	{
		pbounds->min_x[neye] -= nLeft;
		pbounds->max_x[neye] += nRight;
		pbounds->min_y[neye] -= nTop;
		pbounds->max_y[neye] += nBottom;
	}
}


int vrfont_init( VRFont_t * pFont, const uint8_t *pData, uint32_t size )
{
	if ( !pFont )
		return 1;

	// memset the struct to 0, and then set the members
	memset( pFont, 0, sizeof( VRFont_t ) );
	pFont->pData = pData;
	pFont->size = size;

	if ( pFont->size < sizeof( VRFontHeader_t ) )
	{
		return 2;
	}

	const VRFontHeader_t *pHeader = (const  VRFontHeader_t *) pFont->pData;
	if (pHeader->nMagic != 0x56525458 )
	{
		return 3;
	}
	if (pHeader->nVersion != 1 )
	{
		return 4;
	}

	// confirm all w/h are sane.  confirm all origins are contained in size
	int nMinSpace = 2 * pHeader->nNumCodePoints * sizeof( VRGlyph_t ) + sizeof( VRFontHeader_t );
	if ( pFont->size < nMinSpace )
	{
		return 5;
	}

	pFont->maxGlyphWidth = 0;
	pFont->maxGlyphHeight = 0;

	int nglyphs = 0;
	// int nlastoffset = 0;
	int nStartCodePoint = pHeader->nStartCodePoint;
	int nEndCodePoint = pHeader->nStartCodePoint + pHeader->nNumCodePoints;
	for ( int codepoint = nStartCodePoint; codepoint<=nEndCodePoint; ++codepoint )
	{
		for (int eye = 0; eye <2; ++eye )
		{
			VRGlyph_t * pGlyph = vrfont_find_vrglyph( pFont, codepoint, eye );
			if (!pGlyph)
				continue;

			if ( pGlyph->nWidth > 64 || pGlyph->nHeight > 64 || 
				pGlyph->nOriginX >= pGlyph->nWidth || pGlyph->nOriginY >= pGlyph->nHeight )
			{
				return 6;
			}

			if ( pGlyph->nDataOffsetFromFileStart )
			{
				int nPixelDataEnd = pGlyph->nDataOffsetFromFileStart + pGlyph->nWidth * pGlyph->nHeight;
				if ( nPixelDataEnd > pFont->size )
					return 7;
			}
			
			pFont->maxGlyphWidth = u8_max( pFont->maxGlyphWidth, pGlyph->nWidth );
			pFont->maxGlyphHeight = u8_max( pFont->maxGlyphHeight, pGlyph->nHeight );

			nglyphs++;
		}
	}

	return 0;
}

VRGlyph_t * vrfont_find_vrglyph( const VRFont_t * pFont, unsigned char codepoint, int neye )
{
	VRFontHeader_t *pHeader = (VRFontHeader_t *) pFont->pData;

	if ( codepoint < pHeader->nStartCodePoint || codepoint > ( pHeader->nStartCodePoint + pHeader->nNumCodePoints ) )
		return NULL;
	int nHeaderIndex = ( codepoint - pHeader->nStartCodePoint ) * 2 + neye;

	VRGlyph_t * pGlyph = (VRGlyph_t *) ( pFont->pData + sizeof( VRFontHeader_t ) + nHeaderIndex * sizeof( VRGlyph_t ) );
	if ( pGlyph->nDataOffsetFromFileStart == 0 )
	{
		return NULL;
	}

	return pGlyph;
}

const unsigned char * vrfont_find_vrglyph_pixeldata( const VRFont_t * pFont, const VRGlyph_t * pGlyph )
{
	if ( !pFont || !pGlyph || !pGlyph->nDataOffsetFromFileStart )
		return NULL;

	int nPixelDataEnd = pGlyph->nDataOffsetFromFileStart + pGlyph->nWidth * pGlyph->nHeight;
	if ( nPixelDataEnd > pFont->size )
		return NULL;

	return pFont->pData + pGlyph->nDataOffsetFromFileStart;
}

int vrterminal_init( VRTerminal_t * pTerminal, const uint8_t *pData, uint32_t size )
{
	if ( !pTerminal )
		return 1;

	// memset the struct to 0, and then set the members
	memset( pTerminal, 0, sizeof( VRTerminal_t ) );
	pTerminal->pData = pData;
	pTerminal->size = size;

	if ( pTerminal->size < sizeof( VRTerminalHeader_t ) )
	{
		return 2;
	}

	const VRTerminalHeader_t *pHeader = (const VRTerminalHeader_t *) pTerminal->pData;
	if (pHeader->nMagic != 0x56525459 )
	{
		return 3;
	}
	if (pHeader->nVersion != 1 )
	{
		return 4;
	}

	int nCells = pHeader->nTerminalSizeX * pHeader->nTerminalSizeY;
	int nMinSpace = nCells * sizeof( VRTerminalPoint_t ) + sizeof( VRTerminalHeader_t );
	if ( pTerminal->size < nMinSpace )
	{
		return 5;
	}

	pTerminal->width = pHeader->nTerminalSizeX;
	pTerminal->height = pHeader->nTerminalSizeY;
	if ( pTerminal->width * pTerminal->height > TERMINAL_MAX_CELLS )
		return 6;

	if ( pTerminal->width == 0 || pTerminal->height == 0 )
		return 7;

	// Calc the bounds for all points
	for ( int neye=0; neye<2; ++neye)
	{
		pTerminal->terminalBounds.min_x[neye] = INT32_MAX;
		pTerminal->terminalBounds.min_y[neye] = INT32_MAX;
		pTerminal->terminalBounds.max_x[neye] = INT32_MIN;
		pTerminal->terminalBounds.max_y[neye] = INT32_MIN;
	}

	for (int ny=0; ny<pTerminal->height; ++ny )
	{
		for ( int nx=0; nx<pTerminal->width; ++nx )
		{
			VRTerminalPoint_t * pPt = vrterminal_get_point( pTerminal, nx, ny );
			if ( !pPt )
				continue;
			for ( int neye =0; neye < 2; ++neye )
			{
				pTerminal->terminalBounds.min_x[neye] = s32_min( pTerminal->terminalBounds.min_x[neye],
					s32_min( s32_min( pPt->blue_x[neye], pPt->red_x[neye] ), pPt->green_x[neye] ) );
				pTerminal->terminalBounds.min_y[neye] = s32_min( pTerminal->terminalBounds.min_y[neye],
					s32_min( s32_min( pPt->blue_y[neye], pPt->red_y[neye] ), pPt->green_y[neye] ) );

				pTerminal->terminalBounds.max_x[neye] = s32_max( pTerminal->terminalBounds.max_x[neye],
					s32_max( s32_max( pPt->blue_x[neye], pPt->red_x[neye] ), pPt->green_x[neye] ) );
				pTerminal->terminalBounds.max_y[neye] = s32_max( pTerminal->terminalBounds.max_y[neye],
					s32_max( s32_max( pPt->blue_y[neye], pPt->red_y[neye] ), pPt->green_y[neye] ) );
			}
		}
	}

	return 0;
}

VRTerminalPoint_t * vrterminal_get_point( VRTerminal_t * pTerminal, int idx, int idy )
{
	if ( !pTerminal || pTerminal->size < sizeof( VRTerminalHeader_t ) )
		return NULL;
	VRTerminalHeader_t *pHeader = (VRTerminalHeader_t *) pTerminal->pData;
	if ( idx < 0 || idx >= pHeader->nTerminalSizeX || idy < 0 || idy >= pHeader->nTerminalSizeY )
		return NULL;
	const int nTerminalPointIndex = pHeader->nTerminalSizeX * idy + idx;
	VRTerminalPoint_t * pTerminalPoint = ( VRTerminalPoint_t * ) &pTerminal->pData[ sizeof( VRTerminalHeader_t ) + nTerminalPointIndex * sizeof( VRTerminalPoint_t ) ];
	if ( !pTerminalPoint->nValid )
		return NULL;
	return pTerminalPoint;
}

void vrterminal_scroll_up( TerminalUpdate_t * pUpdate, VRTerminal_t *pTerminal )
{
	// cells[0] is top left cell
	// Move all rows up by 1
	memmove( &pTerminal->vCells[0], &pTerminal->vCells[pTerminal->width],
			(pTerminal->height - 1) * pTerminal->width * sizeof(VRCell_t) );

	// clear the bottom row
	VRCell_t * pCellPtr = pTerminal->vCells + (pTerminal->height - 1) * pTerminal->width;
	memset( pCellPtr, 0, sizeof( VRCell_t ) * pTerminal->width );

	if ( pUpdate )
	{
		pUpdate->full_update = 1;
	}
}

static void vrterminal_record_update( TerminalUpdate_t * pUpdate, uint8_t x, uint8_t y )
{
	if ( !pUpdate || pUpdate->full_update )
		return;

	if ( pUpdate->update_count >= ( TERMINAL_UPDATE_COUNT_MAX - 1 ) )
	{
		pUpdate->full_update = 1;
		return;
	}

	pUpdate->vUpdates[pUpdate->update_count].x = x;
	pUpdate->vUpdates[pUpdate->update_count].y = y;
	pUpdate->update_count++;
}

void vrterminal_putc( TerminalUpdate_t * pUpdate, VRTerminal_t * pTerminal, uint8_t x, uint8_t y, unsigned char c, unsigned char attr)
{
	pTerminal->cursor_y = y;
	pTerminal->cursor_x = x;

	int nCellIndex = pTerminal->cursor_y * pTerminal->width + pTerminal->cursor_x;
	if ( pTerminal->vCells[nCellIndex].ch != c )
	{
		// are we overwriting a visible character at the current cursor position?
		if ( is_visible_ascii( pTerminal->vCells[nCellIndex].ch ) )
		{
			// if so flag a full update
			// TODO: have a real record for this position update
			pUpdate->full_update = 1;
		}
		else if ( is_visible_ascii( c ) )
		{
			// otherwise it's a non-visible to visible update
			vrterminal_record_update( pUpdate, pTerminal->cursor_x, pTerminal->cursor_y );
		}

		// Update cell
		pTerminal->vCells[nCellIndex].ch = c;
		pTerminal->vCells[nCellIndex].attr = attr;
	}
}

void vrterminal_update_reset( TerminalUpdate_t * pUpdate )
{
	pUpdate->full_update = 0;
	pUpdate->update_count = 0;
}

#if 0
static void vrterminal_put_char( TerminalUpdate_t * pUpdate, VRTerminal_t * pTerminal, unsigned char c, uint16_t flags, int advance_cursor )
{
	// Wrap before writing
	if ( pTerminal->cursor_x >= pTerminal->width )
	{
		pTerminal->cursor_x = 0;
		pTerminal->cursor_y++;

		if ( pTerminal->cursor_y >= pTerminal->height )
		{
			vrterminal_scroll_up( pUpdate, pTerminal );
			pTerminal->cursor_y = pTerminal->height - 1;
		}
	}

	int nCellIndex = pTerminal->cursor_y * pTerminal->width + pTerminal->cursor_x;
	if ( pTerminal->vCells[nCellIndex].ch != c /*|| pTerminal->vCells[nCellIndex].flags != flags*/ )
	{
		// are we overwriting a visible character at the current cursor position?
		if ( is_visible_ascii( pTerminal->vCells[nCellIndex].ch ) )
		{
			// if so flag a full update
			// TODO: have a real record for this position update
			pUpdate->full_update = 1;
		}
		else if ( is_visible_ascii( c ) )
		{
			// otherwise it's a non-visible to visible update
			vrterminal_record_update( pUpdate, pTerminal->cursor_x, pTerminal->cursor_y );
		}

		// Update cell
		pTerminal->vCells[nCellIndex].ch = c;
		//pTerminal->vCells[nCellIndex].flags = flags;
	}

	// Advance cursor
	if ( advance_cursor )
	{
		pTerminal->cursor_x++;

		if (pTerminal->cursor_x >= pTerminal->width)
		{
			pTerminal->cursor_x = 0;
			pTerminal->cursor_y++;

			if (pTerminal->cursor_y >= pTerminal->height) {

				vrterminal_scroll_up( pUpdate, pTerminal );
				pTerminal->cursor_y = pTerminal->height - 1;
			}
		}
	}
}

static int vrterminal_is_visually_clear( VRTerminal_t * pTerminal )
{
	int nNumCells = pTerminal->width * pTerminal->height;
	for ( int nCellIndex = 0; nCellIndex < nNumCells; ++nCellIndex )
	{
		if ( is_visible_ascii( pTerminal->vCells[nCellIndex].ch ) )
		{
			return 0;
		}
	}

	return 1;
}

static void vrterminal_clear_console( TerminalUpdate_t * pUpdate, VRTerminal_t * pTerminal )
{
	if ( vrterminal_is_visually_clear( pTerminal ) )
		return;

	if ( pUpdate )
	{
		pUpdate->full_update = 1;
	}
	memset( pTerminal->vCells, 0,
	   pTerminal->width * pTerminal->height * sizeof(VRCell_t) );
}

static void handle_escape_char( TerminalUpdate_t * pUpdate, VRTerminal_t * pTerminal, char ch )
{
#ifdef VRTERMINAL_UBOOT
	if (!IS_ENABLED(CONFIG_VIDEO_ANSI))
		goto error;
#endif
	/* Sanity checking for bogus ESC sequences: */
	if (pTerminal->escape_len >= sizeof(pTerminal->escape_buf))
		goto error;

	// Save / restore cursor position
	if (pTerminal->escape_len == 0)
	{
		switch (ch) {
		case '7':
			pTerminal->escape_cursor_x_saved = pTerminal->cursor_x;
			pTerminal->escape_cursor_y_saved = pTerminal->cursor_y;
			pTerminal->in_escape = 0;
			return;
		case '8': {
			pTerminal->cursor_x = pTerminal->escape_cursor_x_saved;
			pTerminal->cursor_y = pTerminal->escape_cursor_y_saved;
			pTerminal->in_escape = 0;
			return;
		}
		case '[':
			break;
		default:
			goto error;
		}
	}

	pTerminal->escape_buf[pTerminal->escape_len++] = ch;

	/*
	 * Escape sequences are terminated by a letter, so keep
	 * accumulating until we get one:
	 */
	if (!isalpha(ch))
		return;

	/*
	 * clear escape mode first, otherwise things will get highly
	 * surprising if you hit any debug prints that come back to
	 * this console.
	 */
	pTerminal->in_escape = 0;

	switch (ch) {
	case 'A':
	case 'B':
	case 'C':
	case 'D':
	case 'E':
	case 'F': {
		//int row, col, num;
		//char *s = pTerminal->escape_buf;
//TODO implement		
#if 0
		/*
		 * Cursor up/down: [%dA, [%dB, [%dE, [%dF
		 * Cursor left/right: [%dD, [%dC
		 */
		s++;    /* [ */
		s = parsenum(s, &num);
		if (num == 0)			/* No digit in sequence ... */
			num = 1;		/* ... means "move by 1". */

		get_cursor_position(priv, &row, &col);
		if (ch == 'A' || ch == 'F')
			row -= num;
		if (ch == 'C')
			col += num;
		if (ch == 'D')
			col -= num;
		if (ch == 'B' || ch == 'E')
			row += num;
		if (ch == 'E' || ch == 'F')
			col = 0;
		if (col < 0)
			col = 0;
		if (row < 0)
			row = 0;
		/* Right and bottom overflows are handled in the callee. */
		set_cursor_position(dev, row, col);
#endif		
		break;
	}
	case 'H':
	case 'f':
	{
		// Set cursor position: [%d;%df or [%d;%dH
		int row, col;
		char *s = pTerminal->escape_buf;
		s++;    /* [ */
		s = parsenum( s, &row );
		s++;    /* ; */
		s = parsenum( s, &col );
		pTerminal->cursor_x = col;
		pTerminal->cursor_y = row;
		break;
	}
	case 'J':
		vrterminal_clear_console( pUpdate, pTerminal ); // TODO: handle clear mode
		break;
	case 'K': {
		int mode = 0;
		parsenum( pTerminal->escape_buf + 1, &mode ); // +1 to skip '['

		/*
		 * Clear (parts of) current line
		 *   [0K       - clear line to end
		 *   [2K       - clear entire line
		 */
		if (mode == 2)
		{
			pTerminal->cursor_x = 0;
		}
		int nCharsToWrite = pTerminal->width - pTerminal->cursor_x;
		for ( int nchar = 0; nchar<nCharsToWrite; ++nchar )
		{
			vrterminal_put_char( pUpdate, pTerminal, ' ', 0, 1 );
		}
		
		break;
	}
	case 'm': {
		char *s = pTerminal->escape_buf;
		char *end = &pTerminal->escape_buf[pTerminal->escape_len];

		/*
		 * Set graphics mode: [%d;...;%dm
		 *
		 * Foreground Colors:
		 *
		 *   30	Black
		 *   31	Red
		 *   32	Green
		 *   33	Yellow
		 *   34	Blue
		 *   35	Magenta
		 *   36	Cyan
		 *   37	White
		 *
		 * 	 90  bright black   (often a gray)
		 *   91  bright red
		 *   92  bright green
		 *   93  bright yellow
		 *   94  bright blue
		 *   95  bright magenta
		 *   96  bright cyan
		 *   97  bright white
		 *
		 * Background Colors:
		 *
		 *   40	Black
		 *   41	Red
		 *   42	Green
		 *   43	Yellow
		 *   44	Blue
		 *   45	Magenta
		 *   46	Cyan
		 *   47	White
		 */

		s++;    /* [ */
		while (s < end) {
			int val;

			s = parsenum(s, &val);
			s++;

			switch (val) {
			case 0: // reset all flags
				// pTerminal->cursor_flags = 0; // reset
				break;
			case 1: // set bold / bright
				break;
			case 2: // set faint
				break;
			case 3: // set italics
				break;
			case 4: // set underline
				break;
			case 7: // set inverse (swap bg/fg)
				break;
			case 22: // clear bold/faint
				break;
			case 23: // clear italics
				break;
			case 24: // clear underline
				break;
			case 27: // clear inverse
				break;
			case 30 ... 37: // set fg color
				break;
			case 39: // clear fg color (reset to default)
				break;
			case 40 ... 47: // set bg color
				break;
			case 49: // clear bg color
				break;
			default:
				/* ignore unsupported SGR parameter */
				break;
			}
		}
		break;
	}
	default:
#ifdef VRTERMINAL_UBOOT
		debug("unrecognized escape sequence: %*s\n",
			  pTerminal->escape_len, pTerminal->escape_buf);
#endif
	}

	// Clear the escape_buf for easier debugging
	memset( pTerminal->escape_buf, 0, sizeof( pTerminal->escape_buf ) );
	return;

error:
	/* something went wrong, just revert to normal mode: */
	pTerminal->in_escape = 0;
}

void vrterminal_feed( TerminalUpdate_t * pUpdate, VRTerminal_t * pTerminal, const char * buf, int nSize )
{
	for ( int i = 0; i < nSize; ++i ) {
		unsigned char c = (unsigned char) buf[i];

		if ( pTerminal->in_escape )
		{
			handle_escape_char( pUpdate, pTerminal, c );
		}
		else
		{
			switch (c)
			{
			case '\x1b': // escape character
				pTerminal->escape_len = 0;
				pTerminal->in_escape = 1;
				break;

			case '\n': // newline
				pTerminal->cursor_x = 0;
				pTerminal->cursor_y++;
				if (pTerminal->cursor_y >= pTerminal->height )
				{
					vrterminal_scroll_up( pUpdate, pTerminal );
					pTerminal->cursor_y = pTerminal->height - 1;
					if ( pUpdate )
					{
						pUpdate->full_update = 1;
					}
				}
				break;

			case '\r': // return to beginning of line
				pTerminal->cursor_x = 0;
				break;

			case '\b': // backspace
				if ( pTerminal->cursor_x > 0 )
				{
					pTerminal->cursor_x--;
					vrterminal_put_char( pUpdate, pTerminal, ' ', 0, 0 );
				}
				break;
			case '\t': // tab
			{
				static const int k_nTabWidth = 4;
				int col  = pTerminal->cursor_x;
				int next_tab = ( ( col / k_nTabWidth ) + 1 ) * k_nTabWidth;
				int spaces = next_tab - col;

				while (spaces-- > 0) {
					vrterminal_put_char( pUpdate, pTerminal, ' ', 0, 1 );
				}
				break;
			}
			default:
				if ( c >= 32 ) // printable
				{
					vrterminal_put_char( pUpdate, pTerminal, c, pTerminal->cursor_flags, 1 );
				}
				break;
			}
		}
	}
}
#endif
