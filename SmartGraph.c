

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: SmartGraph.c												}
{       Copyright(c): Leon de Boer(LdB) 2017, 2018							}
{       Version: 2.12														}
{		https://github.com/LdB-ECM/Raspberry-Pi								}
{		(Extracted from rpi-SmartStart.c)									}
{       RasROS Modifications:                                               }
{			Added text wrap-around in WriteText							    }
{***************[ THIS CODE IS FREEWARE UNDER CC Attribution]***************}
{																            }
{     This sourcecode is released for the purpose to promote programming    }
{  on the Raspberry Pi. You may redistribute it and/or modify with the      }
{  following disclaimer and condition.                                      }
{																            }
{      The SOURCE CODE is distributed "AS IS" WITHOUT WARRANTIES AS TO      }
{   PERFORMANCE OF MERCHANTABILITY WHETHER EXPRESSED OR IMPLIED.            }
{   Redistributions of source code must retain the copyright notices to     }
{   maintain the author credit (attribution) .								}
{																			}
{***************************************************************************}*/


#include <stdbool.h>		// C standard unit needed for bool and true/false
#include <stdint.h>			// C standard unit needed for uint8_t, uint32_t, etc
#include <stdarg.h>			// C standard unit needed for variadic functions
#include <string.h>								// Needed for strlen	
#include "Font8x16.h"							// Provides the 8x16 bitmap font for console 
#include "SmartGraph.h"
#include "MemMapIO.h"

#define STOP while(1){timer_wait(10000000);};

/*--------------------------------------------------------------------------}
{					 INTERNAL DEVICE CONTEXT STRUCTURE						}
{--------------------------------------------------------------------------*/
typedef struct __attribute__((__packed__, aligned(4))) tagINTDC {
	uintptr_t fb;  													// Frame buffer address
	uint32_t wth;  													// Screen width (of frame buffer)
	uint32_t ht;  													// Screen height (of frame buffer)
	uint32_t depth;  													// Colour depth (of frame buffer)
	uint32_t pitch;  													// Pitch (Line to line offset)

	uint32_t Columns;
	uint32_t Rows;
	
	/* Position control */
	POINT curPos;  													// Current position of graphics pointer
	POINT cursor;  													// Current cursor position

	/* Current colour controls in RGBA format */
	RGBA TxtColor;  													// Text colour currently set in RGBA format
	RGBA BkColor;  													// Background colour currently set in RGBA format
	RGBA BrushColor;  												// Brush colour currently set in RGBA format

	/* Current color controls in RGBA565 format */
	RGB565 TxtColor565;  												// Text colour currently set in RGBA565 format
	RGB565 BkColor565;  												// Background colour currently set in RGBA565 format
	RGB565 BrushColor565;  											// Brush colour currently set in RGBA565 format

	struct {
		unsigned BkGndTransparent : 1;  								// Background is transparent
		unsigned _reserved : 15;
	}
	;

	/* Bitmap handle .. if one assigned to DC */
	HBITMAP bmp;  													// The bitmap assigned to DC

	/* Function pointers that are set to graphics primitives depending on colour depth */
	void(*ClearArea)(struct tagINTDC* dc, uint_fast32_t x1, uint_fast32_t y1, uint_fast32_t x2, uint_fast32_t y2);
	void(*VertLine)(struct tagINTDC* dc, uint_fast32_t cy, int_fast8_t dir);
	void(*HorzLine)(struct tagINTDC* dc, uint_fast32_t cx, int_fast8_t dir);
	void(*DiagLine)(struct tagINTDC* dc, uint_fast32_t dx, uint_fast32_t dy, int_fast8_t xdir, int_fast8_t ydir);
	void(*WriteChar)(struct tagINTDC* dc, uint8_t Ch);
	void(*TransparentWriteChar)(struct tagINTDC* dc, uint8_t Ch);
	void(*PutImage)(struct tagINTDC* dc, uint_fast32_t dx, uint_fast32_t dy, uint_fast32_t p2wth, HIMAGE imgSrc, bool BottomUp);
} INTDC;

INTDC __attribute__((aligned(4))) console = { 0 };

/***************************************************************************}
{						  PRIVATE C ROUTINES 			                    }
{***************************************************************************/

/*--------------------------------------------------------------------------}
{					   16 BIT COLOUR GRAPHICS ROUTINES						}
{--------------------------------------------------------------------------*/

/*-[INTERNAL: ClearArea16]--------------------------------------------------}
. 16 Bit colour version of the clear area call which block fills the given
. area (x1,y1) up to but not including (x2,y2) with the current brush colour. 
. As an internal function pairs assumed to be correctly ordered and dc valid.
.--------------------------------------------------------------------------*/
static void ClearArea16(INTDC* dc, uint_fast32_t x1, uint_fast32_t y1, uint_fast32_t x2, uint_fast32_t y2) {
	RGB565* __attribute__((aligned(2))) video_wr_ptr = (RGB565*)(uintptr_t)(dc->fb + (y1 * dc->pitch * 2) + (x1 * 2));    //RasROS was __packed__, before aligned(2)
	for (uint_fast32_t y = 0; y < (y2 - y1); y++) {
		// For each y line
for(uint_fast32_t x = 0 ; x < (x2 - x1) ; x++) {
			// For each x between x1 and x2
video_wr_ptr[x] = dc->BrushColor565;  					// Write the colour
		}
		video_wr_ptr += dc->pitch;  									// Offset to next line
	}
}

/*-[INTERNAL: VertLine16]---------------------------------------------------}
. 16 Bit colour version of the vertical line draw from (x,y), to cy pixels
. in dir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void VertLine16(INTDC* dc, uint_fast32_t cy, int_fast8_t dir) {
	RGB565* __attribute__((aligned(2))) video_wr_ptr = (RGB565*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 2) + (dc->curPos.x * 2));
	for (uint_fast32_t i = 0; i < cy; i++) {
		// For each y line
video_wr_ptr[0] = dc->TxtColor565;  							// Write the current text colour
if(dir == 1) video_wr_ptr += dc->pitch;  					// Positive offset to next line
	else  video_wr_ptr -= dc->pitch;  						// Negative offset to next line
	}
}

/*-[INTERNAL: HorzLine16]---------------------------------------------------}
. 16 Bit colour version of the horizontal line draw from (x,y), to cx pixels 
. in dir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void HorzLine16(INTDC* dc, uint_fast32_t cx, int_fast8_t dir) {
	RGB565* __attribute__((aligned(2))) video_wr_ptr = (RGB565*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 2) + (dc->curPos.x * 2));
	for (uint_fast32_t i = 0; i < cx; i++) {
		// For each x pixel
video_wr_ptr[0] = dc->TxtColor565;  							// Write the current text colour
video_wr_ptr += dir;  										// Positive offset to next pixel
	}
}

/*-[INTERNAL: DiagLine16]---------------------------------------------------}
. 16 Bit colour version of the diagonal line draw from (x,y) to dx pixels 
. in xdir, and dy pixels in ydir in the current text colour. 
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void DiagLine16(INTDC* dc, uint_fast32_t dx, uint_fast32_t dy, int_fast8_t xdir, int_fast8_t ydir) {
	RGB565* __attribute__((aligned(2))) video_wr_ptr = (RGB565*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 2) + (dc->curPos.x * 2));
	uint_fast32_t tx = 0;  											// Zero test x value
	uint_fast32_t ty = 0;  											// Zero test y value
	uint_fast32_t eulerMax = dx;  									// Start with dx value
	if(dy > eulerMax) dy = eulerMax;  								// If dy greater change to that value
	for(uint_fast32_t i = 0 ; i < eulerMax ; i++) {
		// For euler steps
video_wr_ptr[0] = dc->TxtColor565;  							// Write pixel in current text colour
tx += dx;  													// Increment test x value by dx
if(tx >= eulerMax) {
			// If tx >= eulerMax we step
tx -= eulerMax;  											// Subtract eulerMax
video_wr_ptr += xdir;  									// Move pointer left/right 1 pixel
		}
		ty += dy;  													// Increment test y value by dy
		if(ty >= eulerMax) {
			// If ty >= eulerMax we step
ty -= eulerMax;  											// Subtract eulerMax
video_wr_ptr += (ydir*dc->pitch);  						// Move pointer up/down 1 line
		}
	}
}

/*-[INTERNAL: WriteChar16]--------------------------------------------------}
. 16 Bit colour version of the text character draw. The given character is
. drawn at the current position in the current text and background colours.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void WriteChar16(INTDC* dc, uint8_t Ch) {
	RGB565* __attribute__((aligned(2))) video_wr_ptr = (RGB565*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 2) + (dc->curPos.x * 2));
	for (uint_fast32_t y = 0; y < 4; y++) {
		uint32_t b = BitFont[(Ch * 4) + y];  							// Fetch character bits
		for(uint_fast32_t i = 0 ; i < 32 ; i++) {
			// For each bit
RGB565 col = dc->BkColor565;  							// Preset background colour
int xoffs = i % 8;  										// X offset
if((b & 0x80000000) != 0) col = dc->TxtColor565;  		// If bit set take current text colour
video_wr_ptr[xoffs] = col;  								// Write pixel
b <<= 1;  												// Roll font bits left
if(xoffs == 7) video_wr_ptr += dc->pitch;  				// If was bit 7 next line down
		}
	}
}

/*-[INTERNAL: TransparentWriteChar16]---------------------------------------}
. 16 Bit colour version of the text character draw. The given character is
. drawn at the current position in the current text color but with current
. background pixels outside font pixels left untouched.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void TransparentWriteChar16(INTDC* dc, uint8_t Ch) {
	RGB565* __attribute__((aligned(2))) video_wr_ptr = (RGB565*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 2) + (dc->curPos.x * 2));
	for (uint_fast32_t y = 0; y < 4; y++) {
		uint32_t b = BitFont[(Ch * 4) + y];  							// Fetch character bits
		for(uint_fast32_t i = 0 ; i < 32 ; i++) {
			// For each bit
int xoffs = i % 8;  										// X offset
if((b & 0x80000000) != 0) 								// If bit set take text colour
	video_wr_ptr[xoffs] = dc->TxtColor565;  				// Write pixel in current text colour
b <<= 1;  												// Roll font bits left
if(xoffs == 7) video_wr_ptr += dc->pitch;  				// If was bit 7 next line down
		}
	}
}

/*-[INTERNAL: PutImage16]---------------------------------------------------}
. 16 Bit colour version of the put image draw. The image is transferred from
. the source position and drawn to screen. The source and destination format
. need to be the same and should be ensured by preprocess code.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void PutImage16(INTDC* dc, uint_fast32_t dx, uint_fast32_t dy, uint_fast32_t p2wth, HIMAGE ImageSrc, bool BottomUp) {
	HIMAGE video_wr_ptr;
	video_wr_ptr.ptrRGB565 = (RGB565*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 2) + (dc->curPos.x * 2));
	for (uint_fast32_t y = 0; y < dy; y++) {
		// For each line
for(uint_fast32_t x = 0 ; x < dx ; x++) {
			// For each pixel
video_wr_ptr.ptrRGB565[x] = ImageSrc.ptrRGB565[x];  		// Transfer pixel
		}
		if (BottomUp) video_wr_ptr.ptrRGB565 -= dc->pitch;  			// Next line up
			else video_wr_ptr.ptrRGB565 += dc->pitch;  				// Next line down
		ImageSrc.rawImage += p2wth;  									// Adjust image pointer by power 2 width
	}
}

/*--------------------------------------------------------------------------}
{					   24 BIT COLOUR GRAPHICS ROUTINES						}
{--------------------------------------------------------------------------*/

/*-[INTERNAL: ClearArea24]--------------------------------------------------}
. 24 Bit colour version of the clear area call which block fills the given
. area (x1,y1) up to but not including (x2,y2) with the current brush colour.
. As an internal function pairs assumed to be correctly ordered and dc valid.
.--------------------------------------------------------------------------*/
static void ClearArea24(INTDC* dc, uint_fast32_t x1, uint_fast32_t y1, uint_fast32_t x2, uint_fast32_t y2) {
	RGB* __attribute__((aligned(1))) video_wr_ptr = (RGB*)(uintptr_t)(dc->fb + (y1 * dc->pitch * 3) + (x1 * 3));
	for (uint_fast32_t y = 0; y < (y2 - y1); y++) {
		// For each y line
for(uint_fast32_t x = 0 ; x < (x2 - x1) ; x++) {
			// For each x between x1 and x2
video_wr_ptr[x] = dc->BrushColor.rgb;  					// Write the colour
		}
		video_wr_ptr += dc->pitch;  									// Offset to next line
	}
}

/*-[INTERNAL: VertLine24]---------------------------------------------------}
. 24 Bit colour version of the vertical line draw from (x,y), to cy pixels
. in dir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void VertLine24(INTDC* dc, uint_fast32_t cy, int_fast8_t dir) {
	RGB* __attribute__((aligned(1))) video_wr_ptr = (RGB*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 3) + (dc->curPos.x * 3));
	for (uint_fast32_t i = 0; i < cy; i++) {
		// For each y line
video_wr_ptr[0] = dc->TxtColor.rgb;  							// Write the colour
if(dir == 1) video_wr_ptr += dc->pitch;  					// Positive offset to next line
else  video_wr_ptr -= dc->pitch;  						// Negative offset to next line
	}
}

/*-[INTERNAL: HorzLine24]---------------------------------------------------}
. 24 Bit colour version of the horizontal line draw from (x,y), to cx pixels
. in dir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void HorzLine24(INTDC* dc, uint_fast32_t cx, int_fast8_t dir) {
	RGB* __attribute__((aligned(1))) video_wr_ptr = (RGB*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 3) + (dc->curPos.x * 3));
	for (uint_fast32_t i = 0; i < cx; i++) {
		// For each x pixel
video_wr_ptr[0] = dc->TxtColor.rgb;  							// Write the colour
video_wr_ptr += dir;  										// Positive offset to next pixel
	}
}

/*-[INTERNAL: DiagLine24]---------------------------------------------------}
. 24 Bit colour version of the diagonal line draw from (x,y) to dx pixels
. in xdir, and dy pixels in ydir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void DiagLine24(INTDC* dc, uint_fast32_t dx, uint_fast32_t dy, int_fast8_t xdir, int_fast8_t ydir) {
	RGB* __attribute__((aligned(1))) video_wr_ptr = (RGB*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 3) + (dc->curPos.x * 3));
	uint_fast32_t tx = 0;
	uint_fast32_t ty = 0;
	uint_fast32_t eulerMax = dx;  									// Start with dx value
	if(dy > eulerMax) dy = eulerMax;  								// If dy greater change to that value
	for(uint_fast32_t i = 0 ; i < eulerMax ; i++) {
		// For euler steps
video_wr_ptr[0] = dc->TxtColor.rgb;  							// Write pixel
tx += dx;  													// Increment test x value by dx
if(tx >= eulerMax) {
			// If tx >= eulerMax we step
tx -= eulerMax;  											// Subtract eulerMax
video_wr_ptr += xdir;  									// Move pointer left/right 1 pixel
		}
		ty += dy;  													// Increment test y value by dy
		if(ty >= eulerMax) {
			// If ty >= eulerMax we step
ty -= eulerMax;  											// Subtract eulerMax
video_wr_ptr += (ydir*dc->pitch);  						// Move pointer up/down 1 line
		}
	}
}

/*-[INTERNAL: WriteChar24]--------------------------------------------------}
. 24 Bit colour version of the text character draw. The given character is
. drawn at the current position in the current text and background colours.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void WriteChar24(INTDC* dc, uint8_t Ch) {
	RGB* __attribute__((aligned(1))) video_wr_ptr = (RGB*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 3) + (dc->curPos.x * 3));
	for (uint_fast32_t y = 0; y < 4; y++) {
		uint32_t b = BitFont[(Ch * 4) + y];  							// Fetch character bits
		for(uint_fast32_t i = 0 ; i < 32 ; i++) {
			// For each bit
RGB col = dc->BkColor.rgb;  								// Preset background colour
int xoffs = i % 8;  										// X offset
if((b & 0x80000000) != 0) col = dc->TxtColor.rgb;  		// If bit set take text colour
video_wr_ptr[xoffs] = col;  								// Write pixel
b <<= 1;  												// Roll font bits left
if(xoffs == 7) video_wr_ptr += dc->pitch;  				// If was bit 7 next line down
		}
	}
}

/*-[INTERNAL: TransparentWriteChar24]---------------------------------------}
. 24 Bit colour version of the text character draw. The given character is
. drawn at the current position in the current text color but with current
. background pixels outside font pixels left untouched.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void TransparentWriteChar24(INTDC* dc, uint8_t Ch) {
	RGB* __attribute__((aligned(1))) video_wr_ptr = (RGB*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 3) + (dc->curPos.x * 3));
	for (uint_fast32_t y = 0; y < 4; y++) {
		uint32_t b = BitFont[(Ch * 4) + y];  							// Fetch character bits
		for(uint_fast32_t i = 0 ; i < 32 ; i++) {
			// For each bit
int xoffs = i % 8;  										// X offset
if((b & 0x80000000) != 0)								// If bit set take text colour
	video_wr_ptr[xoffs] = dc->TxtColor.rgb;  				// Write pixel
b <<= 1;  												// Roll font bits left
if(xoffs == 7) video_wr_ptr += dc->pitch;  				// If was bit 7 next line down
		}
	}
}

/*-[INTERNAL: PutImage24]---------------------------------------------------}
. 24 Bit colour version of the put image draw. The image is transferred from
. the source position and drawn to screen. The source and destination format
. need to be the same and should be ensured by preprocess code.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void PutImage24(INTDC* dc, uint_fast32_t dx, uint_fast32_t dy, uint_fast32_t p2wth, HIMAGE ImageSrc, bool BottomUp) {
	HIMAGE video_wr_ptr;
	video_wr_ptr.ptrRGB = (RGB*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 3) + (dc->curPos.x * 3));
	for (uint_fast32_t y = 0; y < dy; y++) {
		// For each line
for(uint_fast32_t x = 0 ; x < dx ; x++) {
			// For each pixel
video_wr_ptr.ptrRGB[x] = ImageSrc.ptrRGB[x];  			// Transfer pixel
		}
		if (BottomUp) video_wr_ptr.ptrRGB -= dc->pitch;  				// Next line up
			else video_wr_ptr.ptrRGB += dc->pitch;  					// Next line down
		ImageSrc.rawImage += p2wth;  									// Adjust image pointer by power 2 width
	}
}

/*--------------------------------------------------------------------------}
{					   32 BIT COLOUR GRAPHICS ROUTINES						}
{--------------------------------------------------------------------------*/

/*-[INTERNAL: ClearArea32]--------------------------------------------------}
. 32 Bit colour version of the clear area call which block fills the given
. area (x1,y1) up to but not including (x2,y2) with the current background colour (RasROS was brush color).
. As an internal function pairs assumed to be correctly ordered and dc valid.
.--------------------------------------------------------------------------*/
static void ClearArea32(INTDC* dc, uint_fast32_t x1, uint_fast32_t y1, uint_fast32_t x2, uint_fast32_t y2) {
	RGBA* __attribute__((aligned(4))) video_wr_ptr = (RGBA*)(uintptr_t)(dc->fb + (y1 * dc->pitch * 4) + (y1 * 4));
	for (uint_fast32_t y = 0; y < (y2 - y1); y++) {
		// For each y line
for(uint_fast32_t x = 0 ; x < (x2 - x1) ; x++) {
			// For each x between x1 and x2
video_wr_ptr[x] = dc->BkColor;  						// Write the current back colour
		}
		video_wr_ptr += dc->pitch;  									// Next line down
	}
}

/*-[INTERNAL: VertLine32]---------------------------------------------------}
. 24 Bit colour version of the vertical line draw from (x,y), to cy pixels
. in dir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void VertLine32(INTDC* dc, uint_fast32_t cy, int_fast8_t dir) {
	RGBA* __attribute__((aligned(4))) video_wr_ptr = (RGBA*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 4) + (dc->curPos.x * 4));
	for (uint_fast32_t i = 0; i < cy; i++) {
		// For each y line
video_wr_ptr[0] = dc->TxtColor;  								// Write the colour
if(dir == 1) video_wr_ptr += dc->pitch;  					// Positive offset to next line
else  video_wr_ptr -= dc->pitch;  						// Negative offset to next line
	}
}

/*-[INTERNAL: HorzLine32]---------------------------------------------------}
. 32 Bit colour version of the horizontal line draw from (x,y), to cx pixels
. in dir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void HorzLine32(INTDC* dc, uint_fast32_t cx, int_fast8_t dir) {
	RGBA* __attribute__((aligned(4))) video_wr_ptr = (RGBA*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 4) + (dc->curPos.x * 4));
	for (uint_fast32_t i = 0; i < cx; i++) {
		// For each x pixel
video_wr_ptr[0] = dc->TxtColor;  								// Write the colour
video_wr_ptr += dir;  										// Positive offset to next pixel
	}
}

/*-[INTERNAL: DiagLine32]---------------------------------------------------}
. 32 Bit colour version of the diagonal line draw from (x,y) to dx pixels
. in xdir, and dy pixels in ydir in the current text colour.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void DiagLine32(INTDC* dc, uint_fast32_t dx, uint_fast32_t dy, int_fast8_t xdir, int_fast8_t ydir) {
	RGBA* __attribute__((aligned(4))) video_wr_ptr = (RGBA*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 4) + (dc->curPos.x * 4));
	uint_fast32_t tx = 0;
	uint_fast32_t ty = 0;
	uint_fast32_t eulerMax = dx;  									// Start with dx value
	if(dy > eulerMax) dy = eulerMax;  								// If dy greater change to that value
	for(uint_fast32_t i = 0 ; i < eulerMax ; i++) {
		// For euler steps
video_wr_ptr[0] = dc->TxtColor;  								// Write pixel
tx += dx;  													// Increment test x value by dx
if(tx >= eulerMax) {
			// If tx >= eulerMax we step
tx -= eulerMax;  											// Subtract eulerMax
video_wr_ptr += xdir;  									// Move pointer left/right 1 pixel
		}
		ty += dy;  													// Increment test y value by dy
		if(ty >= eulerMax) {
			// If ty >= eulerMax we step
ty -= eulerMax;  											// Subtract eulerMax
video_wr_ptr += (ydir*dc->pitch);  						// Move pointer up/down 1 line
		}
	}
}

/*-[INTERNAL: WriteChar32]--------------------------------------------------}
. 32 Bit colour version of the text character draw. The given character is
. drawn at the current position in the current text and background colours.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void WriteChar32(INTDC* dc, uint8_t Ch) {
	RGBA* __attribute__((aligned(4))) video_wr_ptr = (RGBA*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 4) + (dc->curPos.x * 4)); //RasROS was __packed__, before aligned(4)
	for (uint_fast32_t y = 0; y < 4; y++) {
		uint32_t b = BitFont[(Ch * 4) + y];  							// Fetch character bits
		for(uint_fast32_t i = 0 ; i < 32 ; i++) {
			// For each bit
RGBA col = dc->BkColor;  									// Preset background colour
uint_fast8_t xoffs = i % 8;  								// X offset
if((b & 0x80000000) != 0) col = dc->TxtColor;  			// If bit set take text colour
video_wr_ptr[xoffs] = col;  								// Write pixel
b <<= 1;  												// Roll font bits left
if(xoffs == 7) video_wr_ptr += dc->pitch;  				// If was bit 7 next line down
		}
	}
}

/*-[INTERNAL: TransparentWriteChar32]---------------------------------------}
. 32 Bit colour version of the text character draw. The given character is
. drawn at the current position in the current text color but with current
. background pixels outside font pixels left untouched.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void TransparentWriteChar32(INTDC* dc, uint8_t Ch) {
	RGBA* __attribute__((aligned(4))) video_wr_ptr = (RGBA*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 4) + (dc->curPos.x * 4)); //RasROS was __packed__, before aligned(4)
	for (uint_fast32_t y = 0; y < 4; y++) {
		uint32_t b = BitFont[(Ch * 4) + y];  							// Fetch character bits
		for(uint_fast32_t i = 0 ; i < 32 ; i++) {
			// For each bit
uint_fast8_t xoffs = i % 8;  								// X offset
if((b & 0x80000000) != 0)								// If bit set take text colour
	video_wr_ptr[xoffs] = dc->TxtColor;  					// Write pixel
b <<= 1;  												// Roll font bits left
if(xoffs == 7) video_wr_ptr += dc->pitch;  				// If was bit 7 next line down
		}
	}
}

/*-[INTERNAL: PutImage32]---------------------------------------------------}
. 32 Bit colour version of the put image draw. The image is transferred from
. the source position and drawn to screen. The source and destination format
. need to be the same and should be ensured by preprocess code.
. As an internal function the dc is assumed to be valid.
.--------------------------------------------------------------------------*/
static void PutImage32(INTDC* dc, uint_fast32_t dx, uint_fast32_t dy, uint_fast32_t p2wth, HIMAGE ImageSrc, bool BottomUp) {
	HIMAGE video_wr_ptr;
	video_wr_ptr.ptrRGBA = (RGBA*)(uintptr_t)(dc->fb + (dc->curPos.y * dc->pitch * 4) + (dc->curPos.x * 4));
	for (uint_fast32_t y = 0; y < dy; y++) {
		// For each line
for(uint_fast32_t x = 0 ; x < dx ; x++) {
			// For each pixel
video_wr_ptr.ptrRGBA[x] = ImageSrc.ptrRGBA[x];  			// Transfer pixel
		}
		if (BottomUp) video_wr_ptr.ptrRGBA -= dc->pitch;  			// Next line up
			else video_wr_ptr.ptrRGBA += dc->pitch;  					// Next line down
		ImageSrc.rawImage += p2wth;  									// Adjust image pointer by power 2 width
	}
}


/*==========================================================================}
{		      SMARTSTART GRAPHICS COLOUR CONTROL ROUTINES					}
{==========================================================================*/

/*-[SetBkMode]--------------------------------------------------------------}
. Matches WIN32 API, Sets the background mix mode of the device context. 
. The background mix mode is used with text, hatched brushes, and pen styles 
. that are not solid lines. Most used to make transparent text.
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
BOOL SetBkMode(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
int mode)											// The new mode
{
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	if((mode == OPAQUE) || (mode == TRANSPARENT))
	{
		intDC->BkGndTransparent = (mode == TRANSPARENT) ? 1 : 0;  	// Set or clear the transparent background flag
		return TRUE; 
	}
	return FALSE;													// Return fail as mode is invalid
}

/*-[SetDCPenColor]----------------------------------------------------------}
. Matches WIN32 API, Sets the current text color to specified color value, 
. or to the nearest physical color if the device cannot represent the color.
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
COLORREF SetDCPenColor(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
COLORREF crColor)							// The new pen color
{
	COLORREF retValue;
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	retValue = intDC->TxtColor.ref;  									// We will return current text color
	intDC->TxtColor.ref = crColor;  									// Update text color on the RGBA format
	intDC->TxtColor565.R = intDC->TxtColor.rgbRed >> 3;  				// Transfer converted red bits onto the RGBA565 format
	intDC->TxtColor565.G = intDC->TxtColor.rgbGreen >> 2;  			// Transfer converted green bits onto the RGBA565 format
	intDC->TxtColor565.B = intDC->TxtColor.rgbBlue >> 3;  			// Transfer converted blue bits onto the RGBA565 format
	return (retValue);  												// Return color reference
}

/*-[SetDCBrushColor]--------------------------------------------------------}
. Matches WIN32 API, Sets the current brush color to specified color value, 
. or to the nearest physical color if the device cannot represent the color.
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
COLORREF SetDCBrushColor(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
COLORREF crColor)							// The new brush color
{
	COLORREF retValue;
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	retValue = intDC->BrushColor.ref;  								// We will return current brush color
	intDC->BrushColor.ref = crColor;  								// Update brush colour
	intDC->BrushColor565.R = intDC->BrushColor.rgbRed >> 3;  			// Transfer converted red bits onto the RGBA565 format
	intDC->BrushColor565.G = intDC->BrushColor.rgbGreen >> 2;  		// Transfer converted green bits onto the RGBA565 format
	intDC->BrushColor565.B = intDC->BrushColor.rgbBlue >> 3;  		// Transfer converted blue bits onto the RGBA565 format
	return (retValue);  												// Return color reference
}

/*-[SetBkColor]-------------------------------------------------------------}
. Matches WIN32 API, Sets the current background color to specified color,  
. or to nearest physical color if the device cannot represent the color. 
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
COLORREF SetBkColor(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
COLORREF crColor)
{
	COLORREF retValue;
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	retValue = intDC->BkColor.ref;  									// We will return current background color
	intDC->BkColor.ref = crColor;  									// Update background colour
	intDC->BkColor565.R = intDC->BkColor.rgbRed >> 3;  				// Transfer converted red bits onto the RGBA565 format
	intDC->BkColor565.G = intDC->BkColor.rgbGreen >> 2;  				// Transfer converted green bits onto the RGBA565 format
	intDC->BkColor565.B = intDC->BkColor.rgbBlue >> 3;  				// Transfer converted blue bits onto the RGBA565 format
	return (retValue);  												// Return color reference
}


/*==========================================================================}
{		       SMARTSTART GRAPHICS PRIMITIVE ROUTINES						}
{==========================================================================*/

/*-[MoveToEx]---------------------------------------------------------------}
. Matches WIN32 API, Updates the current graphics position to the specified
. point and optionally stores the previous position (if valid ptr provided).
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
BOOL MoveToEx(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
int X,
	// New x graphics position 
int Y,
	// New y graphics position
POINT* lpPoint)										// Pointer to return old position in (If set as 0 return ignored) 
{
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	if(intDC->fb) {
		if (lpPoint) (*lpPoint) = intDC->curPos;  					// Return current position
		intDC->curPos.x = X;  										// Update x position
		intDC->curPos.y = Y;  										// Update y position
		return TRUE;  												// Function successfully completed
	}
	return FALSE;													// The DC has no valid framebuffer it is not initialized
}

/*-[LineTo]-----------------------------------------------------------------}
. Matches WIN32 API, Draws a line from the current position up to, but not
. including, specified endpoint coordinate.
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
BOOL LineTo(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
int nXEnd,
	// End at x graphics position
int nYEnd)												// End at y graphics position
{
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	if(intDC->fb) {
		int_fast8_t xdir, ydir;
		uint_fast32_t dx, dy;

		if (nXEnd < intDC->curPos.x) {
			// End x value given is less than x start position
dx = intDC->curPos.x - nXEnd;  							// Unsigned x distance is (current x - end x) 
xdir = -1;  												// Set the x direction as negative
		}
		else {
			dx = nXEnd - intDC->curPos.x;  							// Unsigned x distance is (end x - current x)
			xdir = 1;  												// Set the x direction as positive
		}

		if (nYEnd < intDC->curPos.y) {
			// End y value given is less than y start position
dy = intDC->curPos.y - nYEnd;  							// Unsigned y distance is (current y - end y)
ydir = -1;  												// Set the y direction as negative
		}
		else {
			dy = nYEnd - intDC->curPos.y;  							// Unsigned y distance is (end y - current y)
			ydir = 1;  												// Set the y direction as positive
		}
		if (dx == 0) intDC->VertLine(intDC, dy, ydir);  				// Zero dx means vertical line
			else if(dy == 0) intDC->HorzLine(intDC, dx, xdir);  		// Zero dy means horizontal line
			else intDC->DiagLine(intDC, dx, dy, xdir, ydir);  		// Anything else is a diagonal line
		intDC->curPos.x = nXEnd;  									// Update x position
		intDC->curPos.y = nYEnd;  									// Update y position
		return TRUE;  												// Function successfully completed
	}
	return FALSE;													// The DC has no valid framebuffer it is not initialized
}

/*-[Rectangle]--------------------------------------------------------------}
. Matches WIN32 API, The rectangle defined by the coords is outlined using 
. the current pen and filled by using the current brush. The right and bottom
. co-ordinates are not included in the drawing.
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
BOOL Rectangle(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
int nLeftRect,
	// Left x value of rectangle
int nTopRect,
	// Top y value of rectangle
int nRightRect,
	// Right x value of rectangle (not drawn)
int nBottomRect)									// Bottom y value of rectangle (not drawn)
{
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	if((nRightRect > nLeftRect) && (nBottomRect > nTopRect) && (intDC->fb))// Make sure coords are in ascending order and we have a frame buffer
	{
		if (intDC->TxtColor.ref != intDC->BrushColor.ref)			// The text colour and brush colors differ 
			{
				POINT orgPoint = { 0 };
				MoveToEx(hdc, nLeftRect, nTopRect, &orgPoint);  			// Move to top left coord (hold original coords)
				if(nRightRect - nLeftRect <= 2) {
					// Single or double vertical line
LineTo(hdc, nLeftRect, nBottomRect - 1);  			// Draw left line
if(nRightRect - nLeftRect == 2) {
						// The double line case
MoveToEx(hdc, nRightRect, nBottomRect - 1, 0);  	// Move to right side
LineTo(hdc, nRightRect, nTopRect);  				// Draw the rigth side line
					}
				}
				else if(nBottomRect - nTopRect <= 2) {
					// Single or double horizontal line
LineTo(hdc, nRightRect - 1, nTopRect);  				// Draw top line
if(nBottomRect - nTopRect == 2) {
						// The double line case
MoveToEx(hdc, nLeftRect, nBottomRect - 1, 0);  	// Move to left side bottom
LineTo(hdc, nRightRect - 1, nBottomRect - 1);  	// Draw the bottom line
					}
				}
				else {
					// Normal case where both direction are greater than 2
intDC->ClearArea(intDC,
						nLeftRect + 1,
						nTopRect + 1,
						nRightRect - 1,
						nBottomRect - 1);  				// Call clear area function
		LineTo(hdc, nRightRect - 1, nTopRect);  				// Draw top line
		LineTo(hdc, nRightRect - 1, nBottomRect - 1);  		// Draw right line
		LineTo(hdc, nLeftRect, nBottomRect - 1);  			// Draw bottom line
		LineTo(hdc, nLeftRect, nTopRect);  					// Draw left line
				}
				MoveToEx(hdc, orgPoint.x, orgPoint.y, 0);  				// Restore the graphics coords
			}
		else {
			intDC->ClearArea(intDC,
				nLeftRect,
				nTopRect,
				nRightRect, 
				nBottomRect);  										// Call clear area function
		}
		return TRUE;												// Return success
	}
	return FALSE;													// Return fail as one or both coords pairs were inverted 
}

/*-[TextOut]----------------------------------------------------------------}
. Matches WIN32 API, Writes a character string at the specified location, 
. using the currently selected font, background color, and text color.
. If DC is passed as 0 the screen console is assumed as the target.
.--------------------------------------------------------------------------*/
BOOL TextOut(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
int nXStart,
	// Start text at x graphics position
int nYStart,
	// Start test at y graphics position
LPCSTR lpString,
	// Pointer to character string to print
int cchString)										// Number of characters to print
{
	INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  				// If hdc is zero then we want console otherwise convert handle
	if((cchString) && (lpString) && (intDC->fb)) {
		// Check text data valid and we have a frame buffer
intDC->curPos.x = nXStart;  									// Set x graphics position
intDC->curPos.y = nYStart;  									// Set y graphics position
for(int i = 0 ; i < cchString ; i++) {
			// For each character
if(intDC->BkGndTransparent)
	intDC->TransparentWriteChar(intDC, lpString[i]);  	// Write the character in transparent mode
	else intDC->WriteChar(intDC, lpString[i]);  			// Write the character in fore/back colours
		}
		return TRUE;												// Return success
	}
	return FALSE;													// Return failure as string or count zero or no frame buffer
}

/*==========================================================================}
{				   PI BASIC DISPLAY CONSOLE ROUTINES 						}
{==========================================================================*/

/*-[PiConsole_Init]---------------------------------------------------------}
. Attempts to initialize the Pi graphics screen to the given resolutions.
. The width, Height and Depth can al individually be set to 0 which invokes
. autodetection of that particular settings. So it allows you to set very 
. specific settings or just default for the HDMI monitor detected. This 
. routine needs to be called before any attempt to use any console or 
. graphical routines, although most will just
. return a fail if attempted to use before this is called.
.--------------------------------------------------------------------------*/
//Initialize given the pointer to the framebuffer, and its width/height/color 
//depth and line pitch. Modified for RasROS because original also initialized
//the screen.
//
bool PiConsole_Init(int Width, int Height, int Depth, int Pitch, uint32_t pFramebuffer)
{

	console.fb = pFramebuffer;  						// Transfer the frame buffer
	console.pitch = Pitch;  										// Transfer the line pitch
	console.wth = Width;  											// Hold the successful setting width
	console.ht = Height;  											// Hold the successful setting height
	console.depth = Depth;  											// Hold the successful setting colour depth

	console.Columns = Width / BitFontWth;							//RasROS addition 
	console.Rows = Height / BitFontHt;
	
	
	SetDCPenColor(0, 0xFFFFFFFF);   									// Default text colour is white
	SetBkColor(0, 0x00000000);   										// Default background colour black
	SetDCBrushColor(0, 0xFF00FF00);   									// Default brush colour is green????

	switch(Depth) {
	case 32:														/* 32 bit colour screen mode */
		console.ClearArea = ClearArea32;  							// Set console function ptr to 32bit colour version of clear area
		console.VertLine = VertLine32;  								// Set console function ptr to 32bit colour version of vertical line
		console.HorzLine = HorzLine32;  								// Set console function ptr to 32bit colour version of horizontal line
		console.DiagLine = DiagLine32;  								// Set console function ptr to 32bit colour version of diagonal line
		console.WriteChar = WriteChar32;  							// Set console function ptr to 32bit colour version of write character
		console.TransparentWriteChar = TransparentWriteChar32;  		// Set console function ptr to 32bit colour version of transparent write character
		console.PutImage = PutImage32;  								// Set console function ptr to 32bit colour version of put bitmap image
		console.pitch /= 4;  											// 4 bytes per write
		break;
	case 24:														/* 24 bit colour screen mode */
		console.ClearArea = ClearArea24;  							// Set console function ptr to 24bit colour version of clear area
		console.VertLine = VertLine24;  								// Set console function ptr to 24bit colour version of vertical line
		console.HorzLine = HorzLine24;  								// Set console function ptr to 24bit colour version of horizontal line
		console.DiagLine = DiagLine24;  								// Set console function ptr to 24bit colour version of diagonal line
		console.WriteChar = WriteChar24;  							// Set console function ptr to 24bit colour version of write character
		console.TransparentWriteChar = TransparentWriteChar24;  		// Set console function ptr to 24bit colour version of transparent write character
		console.PutImage = PutImage24;  								// Set console function ptr to 24bit colour version of put bitmap image
		console.pitch /= 3;  											// 3 bytes per write
		break;
	case 16:														/* 16 bit colour screen mode */
		console.ClearArea = ClearArea16;  							// Set console function ptr to 16bit colour version of clear area
		console.VertLine = VertLine16;  								// Set console function ptr to 16bit colour version of vertical line
		console.HorzLine = HorzLine16;  								// Set console function ptr to 16bit colour version of horizontal line
		console.DiagLine = DiagLine16;  								// Set console function ptr to 16bit colour version of diagonal line
		console.WriteChar = WriteChar16;  							// Set console function ptr to 16bit colour version of write character
		console.TransparentWriteChar = TransparentWriteChar16;  		// Set console function ptr to 16bit colour version of transparent write character
		console.PutImage = PutImage16;  								// Set console function ptr to 16bit colour version of put bitmap image
		console.pitch /= 2;  											// 2 bytes per write
		break;
	}

	return true;  													// Return successful
}

/*-[GetConsole_FrameBuffer]-------------------------------------------------}
. Simply returns the console frame buffer. If PiConsole_Init has not yet been
. called it will return 0, which sort of forms an error check value.
.--------------------------------------------------------------------------*/
uint32_t GetConsole_FrameBuffer(void) {
	return (uint32_t)console.fb;									// Return the console frame buffer
}

/*-[GetConsole_Width]-------------------------------------------------------}
. Simply returns the console width. If PiConsole_Init has not yet been called 
. it will return 0, which sort of forms an error check value. If autodetect
. setting was used it is a simple way to get the detected width setup.
.--------------------------------------------------------------------------*/
uint32_t GetConsole_Width(void) {
	return (uint32_t)console.wth;									// Return the console width in pixels
}

/*-[GetConsole_Height]------------------------------------------------------}
. Simply returns the console height. If PiConsole_Init has not been called
. it will return 0, which sort of forms an error check value. If autodetect
. setting was used it is a simple way to get the detected height setup.
.--------------------------------------------------------------------------*/
uint32_t GetConsole_Height(void) {
	return (uint32_t)console.ht;									// Return the console height in pixels
}

/*-[GetConsole_Depth]-------------------------------------------------------}
. Simply returns the console depth. If PiConsole_Init has not yet been called
. it will return 0, which sort of forms an error check value. If autodetect
. setting was used it is a simple way to get the detected colour depth setup.
.--------------------------------------------------------------------------*/
uint32_t GetConsole_Depth(void) {
	return (uint32_t)console.depth;									// Return the colour depth in bits per pixel
}

/*==========================================================================}
{			  PI BASIC DISPLAY CONSOLE CURSOR POSITION ROUTINES 			}
{==========================================================================*/

/*-[WhereXY]----------------------------------------------------------------}
. Simply returns the console cursor x,y position into any valid pointer.
. If a value is not required simply set the pointer to 0 to signal ignore.
.--------------------------------------------------------------------------*/
void WhereXY(uint32_t* x, uint32_t* y)
{
	if (x) (*x) = console.cursor.x;  									// If x pointer is valid write x cursor position to it
	if(y)(*y) = console.cursor.y;  									// If y pointer is valid write y cursor position to it 
}

/*-[GotoXY]-----------------------------------------------------------------}
. Simply set the console cursor to the x,y position. Subsequent text writes
. using console write will then be to that cursor position. This can even be
. set before any screen initialization as it just sets internal variables.
. Position is character column, row! Not pixel..
.--------------------------------------------------------------------------*/
void GotoXY(uint32_t x, uint32_t y)
{
	if (x < console.Columns )
		console.cursor.x = x;  											// Set cursor x position to that requested
	if (y < console .Rows )
		console.cursor.y = y;  											// Set cursor y position to that requested
}

/*-[WriteText]--------------------------------------------------------------}
. Simply writes the given null terminated string out to the the console at
. current cursor x,y position. If PiConsole_Init has not yet been called it
. will simply return, as it does for empty of invalid string pointer.
.--------------------------------------------------------------------------*/
void WriteText(char* lpString) {
	while ((console.fb) && (lpString) && (*lpString != 0))			// While console initialize, string pointer valid and not '\0'
		{
			switch (*lpString) {
			case '\r': {
					// Carriage return character
	console.cursor.x = 0;  								// Cursor back to line start
				}
				break;
			case '\t': {
					// Tab character character
	console.cursor.x += 8;  								// Cursor increment to by 8
	console.cursor.x -= (console.cursor.x % 8);  			// align it to 8
				}
				break;
			case '\n': {
					// New line character
	console.cursor.x = 0;  								// Cursor back to line start
	console.cursor.y++;  									// Increment cursor down a line
				}
				break;
			default: {
					// All other characters
					console.curPos.x = console.cursor.x * BitFontWth;
					console.curPos.y = console.cursor.y * BitFontHt;
					console.WriteChar(&console, *lpString);  				// Write the character to graphics screen
					console.cursor.x++;  									// Cursor.x forward one character
					
				}
				break;
			}
			lpString++;  													// Next character
			
			//RasROS check for wraparound and scroll
			if(console.cursor.x > console.Columns)
			{	
				console.cursor.x = 0;
				console.cursor.y++;
			}
			if (console.cursor.y > console.Rows - 2) 
			{
				console.ClearArea(&console, 0, 0, console.wth, console.ht);
				console.cursor.x = 0;
				console.cursor.y = 0;
				console.curPos.x = 0;
				console.curPos.y = 0;
				
				/*
				//Scroll everything up a line and clear the last line RasROS TODO... (memcpy part buggy)
				for(int ToRow = 0 ; ToRow < console.Rows - 1 ; ToRow++)
				{
					memcpy((void *)(console.fb + ToRow * BitFontHt),
						(void *)(console.fb + (ToRow + 1) * BitFontHt), 
						(console.wth) * BitFontHt * (console.depth >> 3));   //>> = bits to bytes 
					console.ClearArea(&console, 0, (console.Rows - 1) * BitFontHt, console.wth, console.ht);
				}
				console.cursor.y = console.Rows - 1;
				*/
			}
		}
}

/*==========================================================================}
{							GDI OBJECT ROUTINES								}
{==========================================================================*/

/*-[SelectObject]-----------------------------------------------------------}
. Matches WIN32 API, SelectObject function selects an object to the specified 
. device context(DC). The new object replaces the previous object on the DC.
. RETURN: Success the return value is a handle to the object being replaced.
. Failure is indicated by a return of HGDI_ERROR
.--------------------------------------------------------------------------*/
HGDIOBJ SelectObject(HDC hdc,
	// Handle to the DC (0 means use standard console DC)
HGDIOBJ h)									// Handle to GDI object
{
	HGDIOBJ retVal = { HGDI_ERROR };
	if (h.objType) {
		INTDC* intDC = (hdc == 0) ? &console : (INTDC*)hdc;  			// If hdc is zero then we want console otherwise convert handle
		switch(*h.objType)
		{
		case 0:													// Object is a bitmap
			{
				retVal.bitmap = intDC->bmp;  							// Return the old bitmap handle 
				if(intDC->depth == h.bitmap->bmBitsPixel)			// If colour depths match simply put image to DC
				{
					intDC->curPos.x = 0;  							// Zero the x graphics position on the DC
					intDC->curPos.y = 0;  							// zero the Y graphics position on the DC
					intDC->PutImage(intDC,
						h.bitmap->bmWidth,
						h.bitmap->bmHeight,
						h.bitmap->bmWidthBytes,	
						h.bitmap->bmImage,
						h.bitmap->bmBottomUp);  	// Output the bitmap directly to DC
				}
				else {
					// Otherwise we need to run conversion
uint8_t __attribute__((aligned(4))) buffer[4096];
					HIMAGE imgSrc = h.bitmap->bmImage;  				// Transfer pointer
					HIMAGE src;
					src.rawImage = &buffer[0];  						// Pointer src now needs to point to the temp buffer
					intDC->curPos.x = 0;  							// Zero the x graphics position on the DC
					intDC->curPos.y = (h.bitmap->bmBottomUp) ?  
						h.bitmap->bmHeight : 0;  						// Set the Y graphics position on the DC depending on bottom up flag
					for(int y = 0 ; y < h.bitmap->bmHeight ; y++)
					{
						switch (intDC->depth) {
							// For each of the DC colour depths we need to provide a conversion
	case 16 :  									// DC is RGB656
							{
								RGB565 out;  								// So we need RGB565 data
								for(unsigned int i = 0 ; i < h.bitmap->bmWidth ; i++) {
									if (h.bitmap->bmBitsPixel == 24) {
										out.R = imgSrc.ptrRGB[i].rgbRed >> 3;  // Set red bits from RGB
										out.G = imgSrc.ptrRGB[i].rgbGreen >> 2;  // Set green bits from RGB
										out.B = imgSrc.ptrRGB[i].rgbBlue >> 3;  // Set bule bits from RGB
									}
									else {
										out.R = imgSrc.ptrRGBA[i].rgbRed >> 3;  // Set red bits from RGBA
										out.G = imgSrc.ptrRGBA[i].rgbGreen >> 2;  // Set green bits from RGBA
										out.B = imgSrc.ptrRGBA[i].rgbBlue >> 3;  // Set blue bits from RGBA
									}
									src.ptrRGB565[i] = out;  				// Write the converted RGB565 out
								}
								break;
							}
						case 24:									// DC is RGB
							{
								RGB out;  								// So we need RGB data
								for(unsigned int i = 0 ; i < h.bitmap->bmWidth ; i++) {
									if (h.bitmap->bmBitsPixel == 16) {
										out.rgbRed = imgSrc.ptrRGB565[i].R << 3;  // Red value from RGB565 red
										out.rgbGreen = imgSrc.ptrRGB565[i].G << 2;  // Green value from RGB565 green
										out.rgbBlue = imgSrc.ptrRGB565[i].B << 3;  // Blue value from RGB565 blue
									}
									else {
										out.rgbRed = imgSrc.ptrRGBA[i].rgbRed;  // Red value from RGBA red
										out.rgbGreen = imgSrc.ptrRGBA[i].rgbGreen;  // Green value from RGBA green
										out.rgbBlue = imgSrc.ptrRGBA[i].rgbBlue;  // Blue value from RGBA blue
									}
									src.ptrRGB[i] = out;  				// Write the converted RGB out
								}
								break;
							}
						case 32:									// DC is RGBA
							{
								RGBA out;  								// So we need RGBA data
								out.rgbAlpha = 0xFF;  					// Set the alpha
								for(unsigned int i = 0 ; i < h.bitmap->bmWidth ; i++) {
									if (h.bitmap->bmBitsPixel == 16) {
										out.rgbRed = imgSrc.ptrRGB565[i].R << 3;  // Red value from RGB565 red
										out.rgbGreen = imgSrc.ptrRGB565[i].G << 2;  // Green value from RGB565 green
										out.rgbBlue = imgSrc.ptrRGB565[i].B << 3;  // Blue value from RGB565 blue
									}
									else {
										out.rgbRed = imgSrc.ptrRGB[i].rgbRed;  // Red value from RGB red
										out.rgbGreen = imgSrc.ptrRGB[i].rgbGreen;  // Green value from RGB green
										out.rgbBlue = imgSrc.ptrRGB[i].rgbBlue;  // Blue value from RGB blue
									}
									src.ptrRGBA[i] = out;  				// Write the converted RGB out
								}
								break;
							}
						}
						intDC->PutImage(intDC,
							h.bitmap->bmWidth,
							1, 
							h.bitmap->bmWidthBytes,
							src,
							true);  		// Output the single line
						imgSrc.rawImage += h.bitmap->bmWidthBytes;  	// Adjust by power line power of two value
						(h.bitmap->bmBottomUp) ? intDC->curPos.y-- : intDC->curPos.y++;
					}
				}
				intDC->bmp = h.bitmap;  								// Hold the bitmap pointer
			}
		}
	}
	return retVal;
}

