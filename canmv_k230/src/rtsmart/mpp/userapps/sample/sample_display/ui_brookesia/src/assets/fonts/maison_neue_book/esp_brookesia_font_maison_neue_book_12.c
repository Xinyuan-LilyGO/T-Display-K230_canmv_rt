/*******************************************************************************
 * Size: 12 px
 * Bpp: 4
 * Opts: --bpp 4 --size 12 --no-compress --font MaisonNeue-Book.ttf --range 32-127 --format lvgl -o esp_brookesia_font_maison_neue_book_12.c
 ******************************************************************************/

#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

#ifndef ESP_BROOKESIA_FONT_MAISON_NEUE_BOOK_12
#define ESP_BROOKESIA_FONT_MAISON_NEUE_BOOK_12 1
#endif

#if ESP_BROOKESIA_FONT_MAISON_NEUE_BOOK_12

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */

    /* U+0021 "!" */
    0x2d, 0x2, 0xd0, 0x2d, 0x2, 0xd0, 0x2d, 0x2,
    0xd0, 0x4, 0x2, 0x70, 0x6f, 0x10,

    /* U+0022 "\"" */
    0xe5, 0x8e, 0x58, 0xc4, 0x70,

    /* U+0023 "#" */
    0x0, 0x7, 0x70, 0xc2, 0x0, 0x0, 0xa4, 0xe,
    0x0, 0x9, 0xef, 0xee, 0xfe, 0x90, 0x0, 0xe0,
    0x4a, 0x0, 0x0, 0x1d, 0x6, 0x80, 0x0, 0x3,
    0xb0, 0x86, 0x0, 0x2e, 0xff, 0xef, 0xee, 0x10,
    0x8, 0x60, 0xd1, 0x0, 0x0, 0xb3, 0x1d, 0x0,
    0x0,

    /* U+0024 "$" */
    0x0, 0x3, 0x10, 0x0, 0x0, 0x6, 0x30, 0x0,
    0x3, 0xcf, 0xe8, 0x0, 0xe, 0x46, 0x4b, 0x90,
    0x1e, 0x6, 0x31, 0x90, 0xa, 0xca, 0x30, 0x0,
    0x0, 0x4b, 0xfa, 0x20, 0x1, 0x6, 0x36, 0xe0,
    0x5b, 0x6, 0x30, 0xb4, 0xd, 0x76, 0x34, 0xe1,
    0x2, 0xae, 0xfc, 0x40, 0x0, 0x6, 0x30, 0x0,

    /* U+0025 "%" */
    0x0, 0x0, 0x0, 0x0, 0x72, 0x0, 0x7, 0xdd,
    0x30, 0x2, 0xd0, 0x0, 0x1d, 0x3, 0xc0, 0xb,
    0x40, 0x0, 0x3a, 0x0, 0xe0, 0x4b, 0x0, 0x0,
    0x1d, 0x3, 0xc0, 0xd4, 0xbd, 0x50, 0x6, 0xdc,
    0x27, 0x99, 0x51, 0xd1, 0x0, 0x0, 0x1d, 0x1d,
    0x0, 0xa3, 0x0, 0x0, 0x97, 0xd, 0x0, 0xa4,
    0x0, 0x2, 0xd0, 0xa, 0x40, 0xd1, 0x0, 0xb,
    0x40, 0x2, 0xcd, 0x60, 0x0, 0x8, 0x0, 0x0,
    0x0, 0x0,

    /* U+0026 "&" */
    0x0, 0xae, 0xc2, 0x0, 0x7, 0x90, 0x4c, 0x0,
    0x7, 0x80, 0x0, 0x0, 0x0, 0xd4, 0x0, 0x0,
    0x5, 0xcd, 0x63, 0xa0, 0x1d, 0x11, 0xcb, 0xa0,
    0x3b, 0x0, 0xb, 0xc0, 0x1e, 0x30, 0x1b, 0xc8,
    0x4, 0xce, 0xe8, 0x0,

    /* U+0027 "'" */
    0xee, 0xc0,

    /* U+0028 "(" */
    0x0, 0x0, 0x1, 0xc0, 0x9, 0x70, 0x2d, 0x0,
    0x77, 0x0, 0xb4, 0x0, 0xd2, 0x0, 0xe1, 0x0,
    0xc3, 0x0, 0x96, 0x0, 0x5b, 0x0, 0xd, 0x20,
    0x4, 0xc0, 0x0, 0x40,

    /* U+0029 ")" */
    0x0, 0x0, 0x85, 0x0, 0x1d, 0x10, 0x8, 0x80,
    0x1, 0xe0, 0x0, 0xd2, 0x0, 0xc3, 0x0, 0xb4,
    0x0, 0xc3, 0x0, 0xe0, 0x5, 0xb0, 0xc, 0x30,
    0x6a, 0x0, 0x30, 0x0,

    /* U+002A "*" */
    0x0, 0x75, 0x0, 0x1c, 0xa9, 0xc0, 0x3, 0xfe,
    0x20, 0x7, 0x8a, 0x50, 0x5, 0x0, 0x40,

    /* U+002B "+" */
    0x0, 0x4, 0x0, 0x0, 0x0, 0xd0, 0x0, 0x0,
    0xd, 0x0, 0x3, 0xdd, 0xfd, 0xd4, 0x0, 0xd,
    0x0, 0x0, 0x0, 0xd0, 0x0, 0x0, 0xd, 0x0,
    0x0,

    /* U+002C "," */
    0x64, 0xe8, 0x77, 0x50,

    /* U+002D "-" */
    0x5f, 0xfb,

    /* U+002E "." */
    0x73, 0xe8,

    /* U+002F "/" */
    0x0, 0x0, 0x40, 0x0, 0x0, 0xc0, 0x0, 0x6,
    0x60, 0x0, 0xc, 0x0, 0x0, 0x48, 0x0, 0x0,
    0xb1, 0x0, 0x2, 0x90, 0x0, 0xa, 0x20, 0x0,
    0x1b, 0x0, 0x0, 0x84, 0x0, 0x0, 0x40, 0x0,
    0x0,

    /* U+0030 "0" */
    0x1, 0xbf, 0xd5, 0x0, 0xb7, 0x2, 0xd2, 0x2d,
    0x0, 0x6, 0x95, 0x90, 0x0, 0x1c, 0x67, 0x0,
    0x0, 0xe5, 0x90, 0x0, 0x1c, 0x2d, 0x0, 0x6,
    0x90, 0xb7, 0x2, 0xd2, 0x1, 0xbf, 0xd5, 0x0,

    /* U+0031 "1" */
    0x1, 0x91, 0x4d, 0xe2, 0x41, 0xc2, 0x0, 0xc2,
    0x0, 0xc2, 0x0, 0xc2, 0x0, 0xc2, 0x0, 0xc2,
    0x0, 0xc2,

    /* U+0032 "2" */
    0x2, 0xce, 0xc3, 0x0, 0xd4, 0x2, 0xd0, 0x3b,
    0x0, 0xc, 0x20, 0x0, 0x3, 0xe0, 0x0, 0x7,
    0xe4, 0x0, 0x2d, 0x91, 0x0, 0xd, 0x50, 0x0,
    0x3, 0xc0, 0x0, 0x0, 0x3f, 0xff, 0xff, 0x10,

    /* U+0033 "3" */
    0x2, 0xce, 0xc2, 0x0, 0xd4, 0x5, 0xb0, 0x18,
    0x0, 0xe, 0x0, 0x0, 0x6, 0x90, 0x0, 0x3e,
    0xe2, 0x0, 0x0, 0x3, 0xd0, 0x59, 0x0, 0xd,
    0x21, 0xe3, 0x3, 0xe0, 0x4, 0xce, 0xc3, 0x0,

    /* U+0034 "4" */
    0x0, 0x3, 0x80, 0x0, 0x1, 0xce, 0x10, 0x0,
    0x95, 0xd1, 0x0, 0x3b, 0xd, 0x10, 0xc, 0x10,
    0xd1, 0x8, 0x60, 0xd, 0x10, 0xde, 0xee, 0xfe,
    0x10, 0x0, 0xd, 0x10, 0x0, 0x0, 0xd1, 0x0,

    /* U+0035 "5" */
    0x9, 0xff, 0xfa, 0x0, 0xc1, 0x0, 0x0, 0xe,
    0x0, 0x0, 0x2, 0xda, 0xdb, 0x30, 0x2b, 0x30,
    0x3e, 0x10, 0x0, 0x0, 0x95, 0x35, 0x0, 0x9,
    0x52, 0xe2, 0x3, 0xe1, 0x5, 0xde, 0xc3, 0x0,

    /* U+0036 "6" */
    0x0, 0x9e, 0xe6, 0x0, 0x99, 0x1, 0xd4, 0x1e,
    0x0, 0x3, 0x24, 0xa8, 0xdd, 0x60, 0x6e, 0x80,
    0x1c, 0x45, 0xe0, 0x0, 0x59, 0x2e, 0x0, 0x5,
    0x90, 0xb7, 0x1, 0xc4, 0x1, 0xbf, 0xe6, 0x0,

    /* U+0037 "7" */
    0xcf, 0xff, 0xfb, 0x0, 0x0, 0x96, 0x0, 0x1,
    0xe0, 0x0, 0x7, 0x90, 0x0, 0xe, 0x20, 0x0,
    0x5b, 0x0, 0x0, 0xc4, 0x0, 0x3, 0xd0, 0x0,
    0xa, 0x60, 0x0,

    /* U+0038 "8" */
    0x4, 0xcd, 0xa0, 0x1, 0xd1, 0x8, 0x80, 0x4a,
    0x0, 0x3b, 0x1, 0xd2, 0x9, 0x80, 0x9, 0xff,
    0xe1, 0x6, 0xb1, 0x4, 0xd0, 0xa4, 0x0, 0xc,
    0x27, 0xa1, 0x4, 0xe0, 0x8, 0xee, 0xc3, 0x0,

    /* U+0039 "9" */
    0x5, 0xde, 0xb1, 0x3, 0xd1, 0x6, 0xb0, 0x76,
    0x0, 0xe, 0x17, 0x70, 0x0, 0xe3, 0x3d, 0x20,
    0x7f, 0x40, 0x5d, 0xea, 0xc3, 0x3, 0x0, 0xd,
    0x10, 0xe2, 0x6, 0xb0, 0x3, 0xde, 0xc1, 0x0,

    /* U+003A ":" */
    0x21, 0xe8, 0x42, 0x0, 0x0, 0x73, 0xe8,

    /* U+003B ";" */
    0x21, 0xe8, 0x42, 0x0, 0x0, 0x64, 0xe8, 0x77,
    0x50,

    /* U+003C "<" */
    0x0, 0x0, 0x58, 0x0, 0x1b, 0xb2, 0x6, 0xe6,
    0x0, 0x4f, 0x20, 0x0, 0x6, 0xe6, 0x0, 0x0,
    0x1b, 0xb2, 0x0, 0x0, 0x58,

    /* U+003D "=" */
    0x3f, 0xff, 0xfd, 0x0, 0x0, 0x0, 0x3e, 0xee,
    0xed,

    /* U+003E ">" */
    0x1b, 0x20, 0x0, 0x5, 0xe7, 0x0, 0x0, 0x1a,
    0xc3, 0x0, 0x0, 0x8d, 0x0, 0x1a, 0xc3, 0x5,
    0xe7, 0x0, 0x1b, 0x20, 0x0,

    /* U+003F "?" */
    0x2, 0xbf, 0xd4, 0x0, 0xc7, 0x4, 0xe0, 0xc,
    0x0, 0xe, 0x10, 0x0, 0x7, 0xc0, 0x0, 0xb,
    0xa1, 0x0, 0x3, 0xc0, 0x0, 0x0, 0x26, 0x0,
    0x0, 0x3, 0x70, 0x0, 0x0, 0x7f, 0x0, 0x0,

    /* U+0040 "@" */
    0x0, 0x29, 0xcc, 0x81, 0x0, 0x2c, 0x30, 0x5,
    0xc0, 0xb, 0x23, 0xcc, 0x16, 0x71, 0xb0, 0xb2,
    0x68, 0xc, 0x38, 0x0, 0x7e, 0x90, 0xc3, 0x80,
    0xb9, 0x59, 0xb, 0x1b, 0xd, 0x17, 0xa1, 0xb0,
    0xc2, 0x5c, 0x99, 0xc3, 0x3, 0xc2, 0x0, 0x31,
    0x0, 0x2, 0xac, 0xc9, 0x10,

    /* U+0041 "A" */
    0x0, 0x1f, 0x30, 0x0, 0x0, 0x6c, 0x80, 0x0,
    0x0, 0xc3, 0xd0, 0x0, 0x1, 0xc0, 0xb3, 0x0,
    0x6, 0x70, 0x58, 0x0, 0xc, 0x20, 0x1d, 0x0,
    0x1f, 0xee, 0xef, 0x30, 0x78, 0x0, 0x6, 0x80,
    0xc3, 0x0, 0x1, 0xe0,

    /* U+0042 "B" */
    0xff, 0xff, 0xa0, 0xf, 0x0, 0x1a, 0x70, 0xf0,
    0x0, 0x5a, 0xf, 0x0, 0xa, 0x70, 0xff, 0xff,
    0xe2, 0xf, 0x0, 0x6, 0xc0, 0xf0, 0x0, 0xf,
    0xf, 0x0, 0x7, 0xc0, 0xff, 0xff, 0xb2, 0x0,

    /* U+0043 "C" */
    0x0, 0x9e, 0xea, 0x10, 0x9, 0xa2, 0x19, 0xb0,
    0x1e, 0x0, 0x0, 0xe2, 0x5a, 0x0, 0x0, 0x10,
    0x68, 0x0, 0x0, 0x0, 0x5a, 0x0, 0x0, 0x41,
    0x2e, 0x0, 0x0, 0xe2, 0xa, 0xa1, 0x19, 0xb0,
    0x0, 0x9e, 0xea, 0x10,

    /* U+0044 "D" */
    0xff, 0xfd, 0x70, 0xf, 0x0, 0x3d, 0x60, 0xf0,
    0x0, 0x2e, 0xf, 0x0, 0x0, 0xd2, 0xf0, 0x0,
    0xb, 0x3f, 0x0, 0x0, 0xd3, 0xf0, 0x0, 0x2f,
    0xf, 0x0, 0x3c, 0x80, 0xff, 0xfe, 0x70, 0x0,

    /* U+0045 "E" */
    0xff, 0xff, 0xf1, 0xf0, 0x0, 0x0, 0xf0, 0x0,
    0x0, 0xf0, 0x0, 0x0, 0xff, 0xff, 0xb0, 0xf0,
    0x0, 0x0, 0xf0, 0x0, 0x0, 0xf0, 0x0, 0x0,
    0xff, 0xff, 0xf1,

    /* U+0046 "F" */
    0xff, 0xff, 0xf1, 0xf0, 0x0, 0x0, 0xf0, 0x0,
    0x0, 0xf0, 0x0, 0x0, 0xff, 0xff, 0xb0, 0xf0,
    0x0, 0x0, 0xf0, 0x0, 0x0, 0xf0, 0x0, 0x0,
    0xf0, 0x0, 0x0,

    /* U+0047 "G" */
    0x0, 0x9e, 0xea, 0x10, 0x9, 0xa2, 0x7, 0xd0,
    0x2e, 0x0, 0x0, 0x91, 0x5a, 0x0, 0x0, 0x0,
    0x68, 0x0, 0xdf, 0xf4, 0x5a, 0x0, 0x0, 0xb3,
    0x2e, 0x0, 0x0, 0xe1, 0xa, 0xa2, 0x19, 0xa0,
    0x0, 0x9e, 0xfa, 0x0,

    /* U+0048 "H" */
    0xf0, 0x0, 0x3c, 0xf0, 0x0, 0x3c, 0xf0, 0x0,
    0x3c, 0xf0, 0x0, 0x3c, 0xfe, 0xee, 0xec, 0xf0,
    0x0, 0x3c, 0xf0, 0x0, 0x3c, 0xf0, 0x0, 0x3c,
    0xf0, 0x0, 0x3c,

    /* U+0049 "I" */
    0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0,
    0xf0,

    /* U+004A "J" */
    0x0, 0x0, 0xb4, 0x0, 0x0, 0xb4, 0x0, 0x0,
    0xb4, 0x0, 0x0, 0xb4, 0x0, 0x0, 0xb4, 0x0,
    0x0, 0xb4, 0x64, 0x0, 0xc3, 0x6b, 0x13, 0xe0,
    0x9, 0xfd, 0x40,

    /* U+004B "K" */
    0xf0, 0x0, 0xb6, 0xf, 0x0, 0x8a, 0x0, 0xf0,
    0x4d, 0x0, 0xf, 0x2e, 0x20, 0x0, 0xfc, 0xe2,
    0x0, 0xf, 0x86, 0xc0, 0x0, 0xf0, 0xb, 0x70,
    0xf, 0x0, 0x1e, 0x20, 0xf0, 0x0, 0x6c, 0x0,

    /* U+004C "L" */
    0xf0, 0x0, 0xf, 0x0, 0x0, 0xf0, 0x0, 0xf,
    0x0, 0x0, 0xf0, 0x0, 0xf, 0x0, 0x0, 0xf0,
    0x0, 0xf, 0x0, 0x0, 0xff, 0xff, 0xd0,

    /* U+004D "M" */
    0xf8, 0x0, 0x0, 0x5f, 0x2f, 0xe0, 0x0, 0xb,
    0xf2, 0xfa, 0x40, 0x2, 0xcd, 0x2f, 0x4b, 0x0,
    0x86, 0xd2, 0xf0, 0xd1, 0xd, 0x1d, 0x2f, 0x7,
    0x74, 0xa0, 0xd2, 0xf0, 0x1d, 0xa4, 0xd, 0x2f,
    0x0, 0xbd, 0x0, 0xd2, 0xf0, 0x5, 0x70, 0xd,
    0x20,

    /* U+004E "N" */
    0xf7, 0x0, 0xf, 0xf, 0xe0, 0x0, 0xf0, 0xf8,
    0x70, 0xf, 0xf, 0x1e, 0x10, 0xf0, 0xf0, 0x78,
    0xf, 0xf, 0x0, 0xd1, 0xf0, 0xf0, 0x7, 0x9f,
    0xf, 0x0, 0xd, 0xf0, 0xf0, 0x0, 0x6f, 0x0,

    /* U+004F "O" */
    0x0, 0x9e, 0xfa, 0x10, 0x9, 0xa2, 0x19, 0xc0,
    0x1e, 0x0, 0x0, 0xc4, 0x5a, 0x0, 0x0, 0x77,
    0x68, 0x0, 0x0, 0x69, 0x5a, 0x0, 0x0, 0x77,
    0x1e, 0x0, 0x0, 0xc4, 0x9, 0xa2, 0x19, 0xc0,
    0x0, 0x9e, 0xfa, 0x10,

    /* U+0050 "P" */
    0xff, 0xff, 0xb1, 0xf, 0x0, 0x8, 0xb0, 0xf0,
    0x0, 0xf, 0xf, 0x0, 0x0, 0xf0, 0xf0, 0x0,
    0x7b, 0xf, 0xff, 0xfb, 0x20, 0xf0, 0x0, 0x0,
    0xf, 0x0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0x0,

    /* U+0051 "Q" */
    0x0, 0x9e, 0xfa, 0x10, 0x9, 0xa2, 0x19, 0xc0,
    0x1e, 0x0, 0x0, 0xc4, 0x5a, 0x0, 0x0, 0x77,
    0x68, 0x0, 0x0, 0x69, 0x5a, 0x0, 0xb1, 0x77,
    0x1e, 0x0, 0x5c, 0xc4, 0x9, 0xa2, 0x1b, 0xe0,
    0x0, 0x9e, 0xe9, 0x84,

    /* U+0052 "R" */
    0xff, 0xfe, 0xa1, 0xf0, 0x0, 0x89, 0xf0, 0x0,
    0x2d, 0xf0, 0x0, 0x8b, 0xff, 0xff, 0xf3, 0xf0,
    0x1e, 0x10, 0xf0, 0xa, 0x60, 0xf0, 0x3, 0xd0,
    0xf0, 0x0, 0xc6,

    /* U+0053 "S" */
    0x4, 0xdf, 0xd4, 0x0, 0xe4, 0x4, 0xe2, 0x1e,
    0x0, 0x6, 0x40, 0xab, 0x40, 0x0, 0x0, 0x4b,
    0xd6, 0x1, 0x10, 0x1, 0xb7, 0x79, 0x0, 0x3,
    0xb1, 0xe5, 0x1, 0xa9, 0x3, 0xcf, 0xe9, 0x0,

    /* U+0054 "T" */
    0x9f, 0xff, 0xff, 0xa0, 0x0, 0xf0, 0x0, 0x0,
    0xf, 0x0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf,
    0x0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf, 0x0,
    0x0, 0x0, 0xf0, 0x0, 0x0, 0xf, 0x0, 0x0,

    /* U+0055 "U" */
    0xf, 0x0, 0x0, 0xf0, 0xf0, 0x0, 0xf, 0xf,
    0x0, 0x0, 0xf0, 0xf0, 0x0, 0xf, 0xf, 0x0,
    0x0, 0xf0, 0xf0, 0x0, 0xf, 0xf, 0x0, 0x1,
    0xe0, 0xa9, 0x11, 0xa9, 0x1, 0xae, 0xea, 0x10,

    /* U+0056 "V" */
    0xc4, 0x0, 0x1, 0xe0, 0x79, 0x0, 0x5, 0xa0,
    0x1e, 0x0, 0xb, 0x50, 0xc, 0x40, 0xe, 0x0,
    0x6, 0x90, 0x5a, 0x0, 0x1, 0xe0, 0xb5, 0x0,
    0x0, 0xc4, 0xe0, 0x0, 0x0, 0x6d, 0xa0, 0x0,
    0x0, 0x1f, 0x40, 0x0,

    /* U+0057 "W" */
    0xa5, 0x0, 0x8a, 0x0, 0x3d, 0x88, 0x0, 0xde,
    0x0, 0x6a, 0x5a, 0x1, 0xdb, 0x30, 0x87, 0x2d,
    0x6, 0x86, 0x80, 0xb4, 0xf, 0xb, 0x42, 0xd0,
    0xe1, 0xd, 0x3e, 0x0, 0xd2, 0xf0, 0xa, 0x9b,
    0x0, 0x99, 0xc0, 0x7, 0xf6, 0x0, 0x4f, 0x90,
    0x4, 0xf2, 0x0, 0xf, 0x60,

    /* U+0058 "X" */
    0x8a, 0x0, 0xe, 0x20, 0xe2, 0x8, 0x90, 0x6,
    0xb1, 0xe1, 0x0, 0xc, 0xc7, 0x0, 0x0, 0x6f,
    0x10, 0x0, 0xd, 0xb8, 0x0, 0x7, 0xa1, 0xe2,
    0x1, 0xe1, 0x7, 0xa0, 0x98, 0x0, 0xe, 0x30,

    /* U+0059 "Y" */
    0xb6, 0x0, 0x8, 0x92, 0xe0, 0x1, 0xe1, 0x9,
    0x80, 0xa7, 0x0, 0x1e, 0x4d, 0x0, 0x0, 0x7f,
    0x50, 0x0, 0x1, 0xf0, 0x0, 0x0, 0x1f, 0x0,
    0x0, 0x1, 0xf0, 0x0, 0x0, 0x1f, 0x0, 0x0,

    /* U+005A "Z" */
    0x4f, 0xff, 0xff, 0x90, 0x0, 0x1, 0xe3, 0x0,
    0x0, 0xa7, 0x0, 0x0, 0x5c, 0x0, 0x0, 0x1e,
    0x20, 0x0, 0xa, 0x70, 0x0, 0x5, 0xc0, 0x0,
    0x1, 0xe2, 0x0, 0x0, 0x7f, 0xff, 0xff, 0x70,

    /* U+005B "[" */
    0xff, 0x6f, 0x0, 0xf0, 0xf, 0x0, 0xf0, 0xf,
    0x0, 0xf0, 0xf, 0x0, 0xf0, 0xf, 0x0, 0xf0,
    0xf, 0xf6,

    /* U+005C "\\" */
    0x40, 0x0, 0x0, 0x84, 0x0, 0x0, 0x1b, 0x0,
    0x0, 0xa, 0x20, 0x0, 0x3, 0x90, 0x0, 0x0,
    0xb1, 0x0, 0x0, 0x48, 0x0, 0x0, 0xc, 0x0,
    0x0, 0x6, 0x60, 0x0, 0x0, 0xc0, 0x0, 0x0,
    0x40,

    /* U+005D "]" */
    0xbf, 0xb0, 0x4b, 0x4, 0xb0, 0x4b, 0x4, 0xb0,
    0x4b, 0x4, 0xb0, 0x4b, 0x4, 0xb0, 0x4b, 0x4,
    0xbb, 0xfb,

    /* U+005E "^" */
    0x0, 0x4e, 0x0, 0x0, 0xbc, 0x50, 0x1, 0xd3,
    0xc0, 0x8, 0x70, 0xc2, 0xe, 0x10, 0x69,

    /* U+005F "_" */
    0x5e, 0xee, 0xee, 0x80,

    /* U+0060 "`" */
    0x7, 0x0, 0x77,

    /* U+0061 "a" */
    0x6, 0xee, 0x80, 0x3d, 0x10, 0xc3, 0x12, 0x2,
    0xd6, 0x7, 0xdb, 0xb6, 0x4b, 0x10, 0x96, 0x6a,
    0x1, 0xd6, 0xb, 0xec, 0xa6,

    /* U+0062 "b" */
    0x3b, 0x0, 0x0, 0x3, 0xb0, 0x0, 0x0, 0x3c,
    0xae, 0xc3, 0x3, 0xf5, 0x4, 0xe0, 0x3c, 0x0,
    0x9, 0x53, 0xb0, 0x0, 0x77, 0x3c, 0x0, 0x9,
    0x53, 0xf5, 0x4, 0xe0, 0x3c, 0xbe, 0xc3, 0x0,

    /* U+0063 "c" */
    0x2, 0xce, 0xc3, 0x0, 0xd4, 0x4, 0xd0, 0x4a,
    0x0, 0x6, 0x16, 0x80, 0x0, 0x0, 0x4a, 0x0,
    0x6, 0x10, 0xd4, 0x4, 0xe0, 0x2, 0xce, 0xc3,
    0x0,

    /* U+0064 "d" */
    0x0, 0x0, 0xb, 0x30, 0x0, 0x0, 0xb3, 0x2,
    0xce, 0xac, 0x30, 0xd4, 0x5, 0xf3, 0x4a, 0x0,
    0xc, 0x36, 0x80, 0x0, 0xb3, 0x4a, 0x0, 0xc,
    0x30, 0xd4, 0x5, 0xf3, 0x3, 0xce, 0xbb, 0x30,

    /* U+0065 "e" */
    0x2, 0xce, 0xc2, 0x0, 0xd4, 0x5, 0xd0, 0x4a,
    0x0, 0xb, 0x36, 0xec, 0xcc, 0xd4, 0x49, 0x0,
    0x0, 0x0, 0xd3, 0x3, 0xd0, 0x2, 0xce, 0xc3,
    0x0,

    /* U+0066 "f" */
    0x5, 0xea, 0xa, 0x50, 0xaf, 0xec, 0xa, 0x40,
    0xa, 0x40, 0xa, 0x40, 0xa, 0x40, 0xa, 0x40,
    0xa, 0x40,

    /* U+0067 "g" */
    0x2, 0xce, 0xac, 0x30, 0xd4, 0x5, 0xf3, 0x4a,
    0x0, 0xc, 0x36, 0x80, 0x0, 0xb3, 0x4a, 0x0,
    0xc, 0x30, 0xd4, 0x5, 0xf3, 0x2, 0xce, 0xbc,
    0x30, 0x20, 0x0, 0xc2, 0xb, 0x70, 0x4d, 0x0,
    0x1a, 0xec, 0x30,

    /* U+0068 "h" */
    0x3b, 0x0, 0x0, 0x3b, 0x0, 0x0, 0x3c, 0xbe,
    0xc1, 0x3f, 0x40, 0x7a, 0x3c, 0x0, 0xe, 0x3b,
    0x0, 0xe, 0x3b, 0x0, 0xe, 0x3b, 0x0, 0xe,
    0x3b, 0x0, 0xe,

    /* U+0069 "i" */
    0x4c, 0x1, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b,
    0x3b,

    /* U+006A "j" */
    0x2, 0xe0, 0x1, 0x1, 0xd0, 0x1d, 0x1, 0xd0,
    0x1d, 0x1, 0xd0, 0x1d, 0x1, 0xd0, 0x2d, 0x1e,
    0x80,

    /* U+006B "k" */
    0x3b, 0x0, 0x0, 0x3b, 0x0, 0x0, 0x3b, 0x1,
    0xd4, 0x3b, 0xb, 0x60, 0x3b, 0xa9, 0x0, 0x3f,
    0xbc, 0x0, 0x3d, 0xa, 0x60, 0x3b, 0x2, 0xe1,
    0x3b, 0x0, 0x89,

    /* U+006C "l" */
    0x3b, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b,
    0x3b,

    /* U+006D "m" */
    0x3d, 0xce, 0x88, 0xed, 0x33, 0xf2, 0x1d, 0xa0,
    0x4c, 0x3c, 0x0, 0x95, 0x0, 0xe3, 0xb0, 0x9,
    0x50, 0xe, 0x3b, 0x0, 0x95, 0x0, 0xe3, 0xb0,
    0x9, 0x50, 0xe, 0x3b, 0x0, 0x95, 0x0, 0xe0,

    /* U+006E "n" */
    0x3c, 0xbe, 0xc1, 0x3f, 0x40, 0x7a, 0x3c, 0x0,
    0xe, 0x3b, 0x0, 0xe, 0x3b, 0x0, 0xe, 0x3b,
    0x0, 0xe, 0x3b, 0x0, 0xe,

    /* U+006F "o" */
    0x2, 0xce, 0xc3, 0x0, 0xd4, 0x3, 0xe1, 0x4a,
    0x0, 0x9, 0x66, 0x80, 0x0, 0x68, 0x4a, 0x0,
    0x9, 0x60, 0xd4, 0x3, 0xe1, 0x2, 0xce, 0xc3,
    0x0,

    /* U+0070 "p" */
    0x3c, 0xae, 0xc3, 0x3, 0xf5, 0x4, 0xe0, 0x3c,
    0x0, 0xa, 0x53, 0xb0, 0x0, 0x77, 0x3c, 0x0,
    0x9, 0x53, 0xf5, 0x4, 0xe0, 0x3c, 0xae, 0xc3,
    0x3, 0xb0, 0x0, 0x0, 0x3b, 0x0, 0x0, 0x2,
    0x80, 0x0, 0x0,

    /* U+0071 "q" */
    0x2, 0xce, 0xac, 0x30, 0xd4, 0x5, 0xf3, 0x4a,
    0x0, 0xc, 0x36, 0x80, 0x0, 0xb3, 0x4a, 0x0,
    0xc, 0x30, 0xd4, 0x5, 0xf3, 0x3, 0xce, 0xac,
    0x30, 0x0, 0x0, 0xb3, 0x0, 0x0, 0xb, 0x30,
    0x0, 0x0, 0x72,

    /* U+0072 "r" */
    0x0, 0x0, 0x3d, 0xd8, 0x3e, 0x10, 0x3c, 0x0,
    0x3b, 0x0, 0x3b, 0x0, 0x3b, 0x0, 0x3b, 0x0,

    /* U+0073 "s" */
    0x7, 0xee, 0x70, 0x2d, 0x1, 0xd3, 0x1e, 0x50,
    0x10, 0x3, 0xad, 0x70, 0x44, 0x0, 0xa7, 0x4c,
    0x10, 0x98, 0x7, 0xee, 0xb0,

    /* U+0074 "t" */
    0x6, 0x10, 0xc, 0x20, 0xaf, 0xea, 0xc, 0x20,
    0xc, 0x20, 0xc, 0x20, 0xc, 0x20, 0xc, 0x30,
    0x7, 0xf5,

    /* U+0075 "u" */
    0x4a, 0x0, 0x2c, 0x4a, 0x0, 0x2c, 0x4a, 0x0,
    0x2c, 0x4a, 0x0, 0x2c, 0x3b, 0x0, 0x3c, 0x1e,
    0x20, 0x9c, 0x6, 0xee, 0x9c,

    /* U+0076 "v" */
    0xa4, 0x0, 0xd, 0x24, 0xa0, 0x4, 0xb0, 0xe,
    0x10, 0xa5, 0x0, 0x86, 0xe, 0x0, 0x2, 0xc5,
    0x90, 0x0, 0xc, 0xd3, 0x0, 0x0, 0x6d, 0x0,
    0x0,

    /* U+0077 "w" */
    0x95, 0x1, 0xf1, 0x4, 0xa6, 0x80, 0x5d, 0x60,
    0x76, 0x2b, 0xa, 0x3b, 0xb, 0x30, 0xe0, 0xc0,
    0xb1, 0xd0, 0xb, 0x78, 0x7, 0x7c, 0x0, 0x8e,
    0x30, 0x2e, 0x80, 0x4, 0xe0, 0x0, 0xd5, 0x0,

    /* U+0078 "x" */
    0x98, 0x0, 0xd3, 0xd, 0x28, 0x90, 0x5, 0xcd,
    0x0, 0x0, 0xe8, 0x0, 0x6, 0xbe, 0x10, 0x1e,
    0x16, 0xa0, 0xa7, 0x0, 0xd4,

    /* U+0079 "y" */
    0xa5, 0x0, 0x2d, 0x4, 0xb0, 0x9, 0x60, 0xd,
    0x10, 0xe0, 0x0, 0x77, 0x59, 0x0, 0x1, 0xdb,
    0x30, 0x0, 0xa, 0xc0, 0x0, 0x0, 0x96, 0x0,
    0x0, 0xe, 0x0, 0x0, 0x6, 0x90, 0x0, 0x0,
    0x52, 0x0, 0x0,

    /* U+007A "z" */
    0x7e, 0xee, 0xf8, 0x0, 0x2, 0xd2, 0x0, 0xc,
    0x40, 0x0, 0x98, 0x0, 0x6, 0xb0, 0x0, 0x3d,
    0x10, 0x0, 0xaf, 0xee, 0xe9,

    /* U+007B "{" */
    0x0, 0xcf, 0x20, 0x4c, 0x0, 0x4, 0xa0, 0x0,
    0x4a, 0x0, 0x6, 0x90, 0x4, 0xf4, 0x0, 0x8,
    0x90, 0x0, 0x4a, 0x0, 0x4, 0xa0, 0x0, 0x4a,
    0x0, 0x3, 0xc0, 0x0, 0xc, 0xf2,

    /* U+007C "|" */
    0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb,

    /* U+007D "}" */
    0xce, 0x40, 0x6, 0xa0, 0x4, 0xb0, 0x4, 0xb0,
    0x3, 0xc0, 0x0, 0xda, 0x3, 0xd2, 0x4, 0xb0,
    0x4, 0xb0, 0x4, 0xb0, 0x6, 0xa0, 0xce, 0x40,

    /* U+007E "~" */
    0x9, 0xfa, 0x23, 0xa1, 0xb1, 0x6d, 0xe3
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 52, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 75, .box_w = 3, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 14, .adv_w = 74, .box_w = 3, .box_h = 3, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 19, .adv_w = 151, .box_w = 9, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 60, .adv_w = 126, .box_w = 8, .box_h = 12, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 108, .adv_w = 193, .box_w = 12, .box_h = 11, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 174, .adv_w = 129, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 210, .adv_w = 49, .box_w = 1, .box_h = 3, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 212, .adv_w = 70, .box_w = 4, .box_h = 14, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 240, .adv_w = 70, .box_w = 4, .box_h = 14, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 268, .adv_w = 94, .box_w = 6, .box_h = 5, .ofs_x = 0, .ofs_y = 4},
    {.bitmap_index = 283, .adv_w = 113, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 308, .adv_w = 58, .box_w = 2, .box_h = 4, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 312, .adv_w = 70, .box_w = 4, .box_h = 1, .ofs_x = 0, .ofs_y = 3},
    {.bitmap_index = 314, .adv_w = 58, .box_w = 2, .box_h = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 316, .adv_w = 85, .box_w = 6, .box_h = 11, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 349, .adv_w = 119, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 381, .adv_w = 70, .box_w = 4, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 399, .adv_w = 110, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 431, .adv_w = 109, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 463, .adv_w = 104, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 495, .adv_w = 109, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 527, .adv_w = 114, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 559, .adv_w = 97, .box_w = 6, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 586, .adv_w = 104, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 618, .adv_w = 109, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 650, .adv_w = 58, .box_w = 2, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 657, .adv_w = 58, .box_w = 2, .box_h = 9, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 666, .adv_w = 106, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 687, .adv_w = 106, .box_w = 6, .box_h = 3, .ofs_x = 0, .ofs_y = 3},
    {.bitmap_index = 696, .adv_w = 106, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 717, .adv_w = 114, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 749, .adv_w = 155, .box_w = 9, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 794, .adv_w = 114, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 830, .adv_w = 121, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 862, .adv_w = 125, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 898, .adv_w = 125, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 930, .adv_w = 108, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 957, .adv_w = 106, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 984, .adv_w = 125, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1020, .adv_w = 125, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1047, .adv_w = 49, .box_w = 2, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1056, .adv_w = 100, .box_w = 6, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1083, .adv_w = 115, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1115, .adv_w = 100, .box_w = 5, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1138, .adv_w = 163, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1179, .adv_w = 129, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1211, .adv_w = 131, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1247, .adv_w = 118, .box_w = 7, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1279, .adv_w = 131, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1315, .adv_w = 117, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1342, .adv_w = 116, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1374, .adv_w = 113, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1406, .adv_w = 127, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1438, .adv_w = 116, .box_w = 8, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1474, .adv_w = 162, .box_w = 10, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1519, .adv_w = 106, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1551, .adv_w = 110, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1583, .adv_w = 114, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1615, .adv_w = 59, .box_w = 3, .box_h = 12, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 1633, .adv_w = 85, .box_w = 6, .box_h = 11, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 1666, .adv_w = 59, .box_w = 3, .box_h = 12, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 1684, .adv_w = 106, .box_w = 6, .box_h = 5, .ofs_x = 0, .ofs_y = 4},
    {.bitmap_index = 1699, .adv_w = 116, .box_w = 7, .box_h = 1, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1703, .adv_w = 56, .box_w = 3, .box_h = 2, .ofs_x = 0, .ofs_y = 8},
    {.bitmap_index = 1706, .adv_w = 98, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1727, .adv_w = 113, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1759, .adv_w = 108, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1784, .adv_w = 113, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1816, .adv_w = 110, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1841, .adv_w = 65, .box_w = 4, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1859, .adv_w = 112, .box_w = 7, .box_h = 10, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 1894, .adv_w = 107, .box_w = 6, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1921, .adv_w = 40, .box_w = 2, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1930, .adv_w = 42, .box_w = 3, .box_h = 11, .ofs_x = -1, .ofs_y = -2},
    {.bitmap_index = 1947, .adv_w = 98, .box_w = 6, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1974, .adv_w = 40, .box_w = 2, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1983, .adv_w = 155, .box_w = 9, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2015, .adv_w = 107, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2036, .adv_w = 113, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2061, .adv_w = 113, .box_w = 7, .box_h = 10, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 2096, .adv_w = 113, .box_w = 7, .box_h = 10, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 2131, .adv_w = 62, .box_w = 4, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2147, .adv_w = 96, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2168, .adv_w = 65, .box_w = 4, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2186, .adv_w = 105, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2207, .adv_w = 103, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2232, .adv_w = 145, .box_w = 9, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2264, .adv_w = 90, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2285, .adv_w = 98, .box_w = 7, .box_h = 10, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 2320, .adv_w = 96, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 2341, .adv_w = 70, .box_w = 5, .box_h = 12, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 2371, .adv_w = 50, .box_w = 1, .box_h = 14, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 2378, .adv_w = 70, .box_w = 4, .box_h = 12, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 2402, .adv_w = 122, .box_w = 7, .box_h = 2, .ofs_x = 0, .ofs_y = 3}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Map glyph_ids to kern left classes*/
static const uint8_t kern_left_class_mapping[] =
{
    0, 1, 2, 3, 0, 0, 0, 0,
    3, 4, 4, 5, 0, 6, 7, 6,
    8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 9, 0, 0, 0, 0, 0,
    2, 0, 18, 19, 20, 21, 22, 23,
    21, 24, 24, 25, 26, 27, 24, 24,
    21, 28, 29, 30, 31, 32, 25, 33,
    34, 26, 35, 36, 37, 38, 37, 0,
    6, 0, 39, 40, 40, 41, 40, 42,
    43, 39, 44, 44, 45, 41, 39, 39,
    40, 40, 0, 46, 47, 48, 49, 50,
    51, 45, 52, 53, 37, 0, 37, 0
};

/*Map glyph_ids to kern right classes*/
static const uint8_t kern_right_class_mapping[] =
{
    0, 0, 0, 1, 0, 0, 0, 0,
    1, 2, 2, 3, 0, 4, 5, 4,
    6, 7, 8, 9, 10, 11, 12, 7,
    13, 14, 15, 16, 16, 0, 0, 0,
    17, 0, 18, 19, 20, 19, 19, 19,
    20, 19, 19, 21, 19, 19, 19, 19,
    20, 19, 20, 19, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 30, 0,
    4, 0, 32, 33, 34, 34, 34, 35,
    34, 33, 36, 37, 33, 33, 38, 38,
    34, 38, 34, 38, 39, 40, 41, 42,
    43, 44, 45, 46, 30, 0, 30, 0
};

/*Kern values between classes*/
static const int8_t kern_class_values[] =
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -4, -8, 0, -15, 0,
    -6, -4, -2, -8, 0, 0, 0, -6,
    0, -6, -4, 0, 0, 0, -4, -4,
    0, -8, -4, -6, -8, 0, -10, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -23,
    0, 0, -4, 0, 0, 0, -13, -4,
    2, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, -2, -8, 0,
    2, 0, 2, 2, 2, 2, 0, 0,
    0, -6, 4, -6, 0, 6, 6, 0,
    0, 0, 0, -4, -2, -4, -4, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -6, 2, 0, 0, 0, -12, 2,
    0, 0, 0, 0, 0, 0, -27, -10,
    0, 0, 0, 0, -8, -19, 0, 0,
    2, -4, -13, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -10, 0, 0,
    0, 0, 4, 0, 0, 0, 8, 6,
    -13, 6, 2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -29,
    0, 0, 0, -4, 0, 0, 0, 0,
    0, 0, 0, -13, 2, -6, -23, -6,
    6, 6, 6, 4, 6, 6, 2, 0,
    0, -15, 4, -21, 2, 6, 6, -8,
    -15, 0, 0, 0, 0, -10, 0, -6,
    -4, 0, 0, -8, 4, 0, 0, -2,
    -6, -4, -2, -2, -8, -2, -2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -8, -6, 0, 0, -3, -4, -4,
    0, 0, -2, 0, 0, -6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -6, -4, -3, -4, -6, -4,
    -8, -2, -2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -4, 0, 0,
    0, -4, -4, 0, 0, 0, -6, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -6, 0, 0, 0, 0, 0, -2, -4,
    0, 0, 0, 0, -6, 0, -6, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -6, 0,
    0, 0, 0, 0, 0, -8, -4, -2,
    0, 0, -6, 0, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, -2, -4, -2, 0, 0, 0,
    -6, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, -35, -17, 0,
    -12, -4, -10, -6, -14, -6, 0, -8,
    -2, 0, 0, -13, 0, -4, -19, -2,
    0, 0, 0, 2, 0, 0, -6, 0,
    0, 0, 0, -11, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -4, 0, 0, 0, 4, 0, -2, -4,
    -2, 0, 0, 0, -8, 2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -19, 0,
    0, 4, -4, 6, 0, 0, 0, -6,
    0, 0, -4, 0, 0, 0, -13, 4,
    0, -4, 0, -6, -19, -8, -13, -8,
    2, -17, 0, -6, 0, 0, 0, -4,
    -8, 0, 0, 0, -4, -6, 0, -10,
    -8, 2, -13, 0, -6, 0, 0, -4,
    0, -4, 0, 0, 0, -4, 0, 0,
    -2, 0, 0, 0, 0, -4, 0, 0,
    0, 0, -6, 0, -7, -2, -6, -8,
    -2, -4, 0, -6, 0, -2, 0, 0,
    0, -2, 0, 0, 0, -2, 0, 0,
    0, 0, -4, -2, 0, -8, 4, -6,
    0, 0, 0, 0, 0, 0, -2, 0,
    0, 0, 2, -4, 0, 0, 0, 0,
    -8, 0, -4, 0, -4, -4, -6, -4,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 2, 0, 4, 4, 2, 4, 0,
    -8, -2, 0, -8, 4, -6, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    -4, -4, 0, 0, 0, 0, -8, 0,
    -8, 0, -4, -9, -6, -4, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 2,
    0, 0, 0, -4, -2, 0, -2, 0,
    0, 0, 0, 6, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, 0,
    0, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -4, 0, -4,
    0, 0, 0, 0, -6, -2, 0, -4,
    0, 0, 0, 0, 0, 0, 0, -21,
    0, -17, 0, 0, 0, 0, 0, 0,
    4, 0, 0, 0, 0, -10, 0, -4,
    -19, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -6, 0, -6, -6, 0,
    -2, 0, -8, -8, -4, -8, -8, -8,
    -8, -10, -4, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -8, 0, -6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -8, 0, 0, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 2,
    0, 0, -15, 6, 0, 0, 0, -6,
    0, 0, 0, 0, 0, 0, -6, 2,
    0, -6, -6, 0, 0, 0, 0, 0,
    2, 0, 0, 2, 4, -4, 0, -10,
    -8, 0, 0, 0, -6, -6, -4, -11,
    -6, 0, -11, 0, -25, 0, 0, 6,
    -23, 6, 0, 0, 0, 0, 0, 0,
    -6, 0, 0, 0, -10, 4, 0, -6,
    0, 0, -17, -6, -25, -10, 2, -27,
    0, -4, 0, 0, 0, -2, -6, 0,
    0, 0, 0, -6, 0, -17, -10, 2,
    -15, 0, 0, 0, 0, -25, 0, -17,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, 2, -12, 0, 0, -27, 0,
    0, 0, 0, 0, -6, -6, -6, -4,
    0, 0, 0, -6, 0, 0, 0, 0,
    -2, 4, 0, 0, 0, 0, 0, 0,
    -8, -2, 0, -8, 4, -6, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    -4, -2, 0, 0, 0, 0, -8, 0,
    -8, 0, -4, -9, -4, -4, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 2,
    0, 0, 0, -4, -2, 0, 0, 0,
    0, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -8, 0, -4, 0, -2, 0,
    0, -6, -2, 0, 0, 0, 0, -4,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 0, 0, -4,
    0, 0, 0, 0, 0, 0, 2, 0,
    -6, 0, 0, 0, 0, -4, 0, 0,
    0, 0, -8, 0, -6, 0, 0, -6,
    0, -4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -4, 0, 0, 0, 0,
    0, 0, 0, 2, 0, -29, -25, -23,
    0, 0, 0, -10, -13, 0, 4, 0,
    0, -19, -2, -19, 0, -8, -29, 0,
    -4, 0, 0, 0, 0, 0, 0, 2,
    4, -31, -4, -25, -9, -4, -4, -27,
    -29, -9, -23, -21, -27, -25, -25, -27,
    0, 2, 0, -25, -10, -15, 0, 0,
    0, 0, -10, 0, 2, 0, 0, 0,
    0, -13, 0, -6, -25, -4, 0, 0,
    4, 0, 0, 4, 0, 2, 0, -12,
    0, -9, 0, 0, 0, -4, -12, -2,
    -8, -2, -4, 0, -2, -6, 0, 2,
    0, -19, -2, -10, 0, 0, 0, 0,
    -6, 0, 2, 0, 0, 0, 0, -8,
    0, 0, -13, 0, 0, 0, 0, 0,
    0, 0, 0, 2, 0, -2, 0, -6,
    0, 0, 0, -2, -6, 0, 0, 0,
    0, 0, -2, 0, 0, 2, 0, -33,
    -19, -25, 0, 0, 0, 0, -17, 0,
    2, 0, 0, 0, 0, -17, 0, -8,
    -27, -4, 0, 0, 4, 0, 0, 0,
    0, 2, 0, -17, 0, -17, -6, 0,
    0, -11, -17, -6, -13, -8, -8, -8,
    -8, -13, 0, 0, 0, 0, -12, 2,
    0, 0, 0, 0, -10, 0, 0, 0,
    0, 0, -4, 0, 0, -12, -2, 0,
    0, 0, 0, 2, 0, 2, 0, 0,
    0, 0, 0, -8, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -6, 0, -4, -11, -4, 2, 0,
    2, 2, 2, 2, 0, 0, 0, -2,
    0, -6, 0, 0, 6, 0, -6, 0,
    0, -6, -2, -4, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 4,
    0, 0, 0, 0, -19, 0, 0, 0,
    2, 0, 0, 0, -29, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 4, 0, 0, -15, -4, -6, 0,
    0, 4, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -4, 0, 0, 0, 0, -4, 0,
    0, 0, 0, -2, 0, -6, -4, -4,
    -2, 0, -10, -6, -6, -6, 2, -4,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -10, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -6,
    -19, -2, 0, -2, -4, 0, 0, 0,
    -2, -2, 0, -6, -4, -6, -4, -4,
    0, 2, 0, 0, 0, 6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 4,
    2, -21, 0, -6, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -6, 0, -4,
    2, 0, 0, 0, -2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 6, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 6, 0, 0, 0, 6,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -4, 0, 0, -13, 6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    2, 2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -4, -10, -4,
    0, -6, 0, 0, 0, 0, -8, 2,
    0, 0, 0, 2, 0, 0, 0, 0,
    0, -19, -12, -15, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -4, 0, -8, 0, -6,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, -12, 0, -12, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -6, 0, 0, 0, 0, -4, 0,
    0, 0, -2, -4, 0, -6, -4, -8,
    -6, 0, -2, 0, 0, 0, -13, 6,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 6, 2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -6,
    0, -6, 0, -6, -2, 0, 0, 0,
    -4, -2, 0, 0, 0, 0, 0, 0,
    -6, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -8, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -4,
    0, -23, -4, -10, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -6, 0, -4, 0, -7,
    0, 0, 0, 0, -6, 2, 0, 0,
    0, 0, 0, 0, 0, -2, 0, -12,
    -2, -8, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -2, 0, -4, 0, -4, 0, 0,
    0, 0, -4, 0, 0, 0, 0, 0,
    0, 0, 0, -4, 0, -23, -6, -10,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -4,
    0, -6, 0, -6, 2, 0, 0, 0,
    -6, 2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    0, -4, 0, 0, 0, 0, -4, 0,
    0, 0, 0, 0, 0, 0
};


/*Collect the kern class' data in one place*/
static const lv_font_fmt_txt_kern_classes_t kern_classes =
{
    .class_pair_values   = kern_class_values,
    .left_class_mapping  = kern_left_class_mapping,
    .right_class_mapping = kern_right_class_mapping,
    .left_class_cnt      = 53,
    .right_class_cnt     = 46,
};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = &kern_classes,
    .kern_scale = 16,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 1,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t esp_brookesia_font_maison_neue_book_12 = {
#else
lv_font_t esp_brookesia_font_maison_neue_book_12 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 15,          /*The maximum line height required by the font*/
    .base_line = 4,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if ESP_BROOKESIA_FONT_MAISON_NEUE_BOOK_12*/
