//========= Copyright Valve Corporation ============//

#ifndef VRTERMINAL
#define VRTERMINAL

//#define VRTERMINAL_UBOOT
#ifdef VRTERMINAL_UBOOT
#include <charset.h>
#endif

#define VRTERMINAL_LINUX
#ifdef VRTERMINAL_LINUX
#include <linux/types.h>
#else
#include <stdint.h>
#endif

// FontView -- a view into loaded memory for the font
// Does not imply data ownership

typedef struct VRFont_t
{
	const uint8_t *pData; // ptr to the file data
	uint32_t size;  // size of pData in bytes
	uint8_t maxGlyphWidth; // max width in pixels
	uint8_t maxGlyphHeight; // max height in pixels
} VRFont_t;

int vrfont_init( VRFont_t * pFont, const uint8_t *pData, uint32_t size );


#pragma pack( push, 1 )

// This is in the file format v1.  do not change
typedef struct VRGlyph_t
{
	uint32_t nDataOffsetFromFileStart;
	uint8_t nWidth;
	uint8_t nHeight;
	uint8_t nOriginX;
	uint8_t nOriginY;
} VRGlyph_t;

// This is in the file format v1.  do not change
typedef struct VRTerminalPoint_t
{
	uint16_t red_x[2];  // L,R. in pixel TENTHS
	uint16_t red_y[2];  // L,R. in pixel TENTHS
	uint16_t green_x[2]; // L,R. in pixel TENTHS
	uint16_t green_y[2]; // L,R. in pixel TENTHS
	uint16_t blue_x[2]; // L,R. in pixel TENTHS
	uint16_t blue_y[2]; // L,R. in pixel TENTHS
	uint8_t nValid;
	uint8_t reserved[3];
} VRTerminalPoint_t;

// TerminalView -- a view into loaded memory for the terminal
// Does not imply data ownership
#define TERMINAL_MAX_HEIGHT 64
#define TERMINAL_MAX_WIDTH 128
#define TERMINAL_MAX_CELLS (TERMINAL_MAX_HEIGHT * TERMINAL_MAX_WIDTH)

typedef struct VRCell_t {
	uint8_t ch;  // ascii (not utf8)
	uint8_t attr; // [0..3] color [4] bold [5] half
} VRCell_t;

typedef struct VRStereoBounds_t
{
	int min_x[2];  // L,R. in pixel TENTHS
	int max_x[2];  // L,R. in pixel TENTHS
	int min_y[2];  // L,R. in pixel TENTHS
	int max_y[2];  // L,R. in pixel TENTHS
} VRStereoBounds_t;

void vrbounds_pad( VRStereoBounds_t * pbounds, int nLeft, int nRight, int nTop, int nBottom ); // positive values pad out

typedef struct VRTerminal_t
{
	// loaded .vrterminal file view (no ownership over data )
	const uint8_t *pData;
	uint32_t size;

	// Local state
	int width;
	int height;
    int cursor_x;
    int cursor_y;
	uint16_t cursor_flags;
	VRCell_t vCells[TERMINAL_MAX_CELLS]; // visible cells
	VRStereoBounds_t terminalBounds;

	// Escape sequence handling
	int in_escape;
	int escape_len;
	char escape_buf[32];
    int escape_cursor_x_saved;
    int escape_cursor_y_saved;
} VRTerminal_t;

typedef struct VRCellUpdate_t {
	uint8_t x, y;
} VRCellUpdate_t;


#define TERMINAL_UPDATE_COUNT_MAX TERMINAL_MAX_WIDTH

typedef struct TerminalUpdate_t
{
	uint8_t full_update;
	uint8_t update_count;
	uint8_t unused[2];
	VRCellUpdate_t vUpdates[TERMINAL_UPDATE_COUNT_MAX];
} TerminalUpdate_t;


#pragma pack( pop )

VRGlyph_t * vrfont_find_vrglyph( const VRFont_t * pFont, unsigned char codepoint, int neye );

const unsigned char * vrfont_find_vrglyph_pixeldata( const VRFont_t * pFont, const VRGlyph_t * pGlyph );

VRTerminalPoint_t * vrterminal_get_point( VRTerminal_t * pTerminal, int idx, int idy );

int vrterminal_init( VRTerminal_t * pTerminal, const uint8_t *pData, uint32_t size );

void vrterminal_update_reset( TerminalUpdate_t * pUpdate );

void vrterminal_scroll_up( TerminalUpdate_t * pUpdate, VRTerminal_t *pTerminal );
void vrterminal_putc( TerminalUpdate_t * pUpdate, VRTerminal_t * pTerminal, uint8_t x, uint8_t y, unsigned char c, unsigned char attr );
void vrterminal_clear_console( TerminalUpdate_t * pUpdate, VRTerminal_t * pTerminal );


#endif
