/**
 * @file lv_conf.h
 * LVGL 8.3.x configuration for CrowPanel 3.5" ESP32 companion display.
 * 480x320 ILI9488, ESP32-WROVER-B.
 */

#if 1 /* Set to "1" to enable content */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
 * COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH 16
/* Swap the 2 bytes of RGB565 color — needed for most SPI TFT displays */
#define LV_COLOR_16_SWAP 1
#define LV_COLOR_SCREEN_TRANSP 0
#define LV_COLOR_MIX_ROUND_OFS 128
#define LV_COLOR_CHROMA_KEY lv_color_hex(0x00ff00)

/*========================
 * MEMORY SETTINGS
 *========================*/
#define LV_MEM_CUSTOM 0
#if LV_MEM_CUSTOM == 0
  /* Internal LVGL heap — 48KB is comfortable for our widget count */
  #define LV_MEM_SIZE (48 * 1024U)
  #define LV_MEM_ADR 0
  #define LV_MEM_AUTO_OPT 0
#else
  #define LV_MEM_CUSTOM_INCLUDE <stdlib.h>
  #define LV_MEM_CUSTOM_ALLOC   malloc
  #define LV_MEM_CUSTOM_FREE    free
  #define LV_MEM_CUSTOM_REALLOC realloc
#endif

#define LV_MEM_BUF_MAX_NUM 16
#define LV_MEMCPY_MEMSET_STD 0

/*====================
 * HAL SETTINGS
 *====================*/
/* Default display refresh period (ms) — ~30fps */
#define LV_DISP_DEF_REFR_PERIOD 33
/* Input device read period (ms) */
#define LV_INDEV_DEF_READ_PERIOD 33
/* Do NOT use a custom tick source — we call lv_tick_inc() manually */
#define LV_TICK_CUSTOM 0
/* Dots-per-inch for the 3.5" 480x320 panel (~160 DPI) */
#define LV_DPI_DEF 160

/*=======================
 * DRAWING FEATURES
 *=======================*/
#define LV_DRAW_COMPLEX 1
#if LV_DRAW_COMPLEX
  #define LV_SHADOW_CACHE_SIZE 0
  #define LV_CIRCLE_CACHE_SIZE 4
#endif

/*===========
 * LOGGING
 *===========*/
/* Disable LVGL logging to save RAM and flash */
#define LV_USE_LOG 0

/*================
 * ASSERT
 *================*/
#define LV_USE_ASSERT_NULL          1
#define LV_USE_ASSERT_MALLOC        1
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0
#define LV_ASSERT_HANDLER_INCLUDE   <stdint.h>
#define LV_ASSERT_HANDLER while(1);

/*================
 * MISC
 *================*/
#define LV_USE_USER_DATA      1
#define LV_USE_PERF_MONITOR   0
#define LV_USE_MEM_MONITOR    0
#define LV_USE_REFR_DEBUG     0
#define LV_SPRINTF_CUSTOM     0
#define LV_USE_BUILTIN_SNPRINTF 1
#define LV_SPRINTF_BUF_SIZE   (sizeof("%.20f") + 20)
#define LV_USE_LARGE_COORD    0

/*==================
 * FONT USAGE
 * (Montserrat fonts, stored in flash)
 *==================*/
#define LV_FONT_MONTSERRAT_8   0
#define LV_FONT_MONTSERRAT_10  0
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_18  0
#define LV_FONT_MONTSERRAT_20  1
#define LV_FONT_MONTSERRAT_22  0
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_26  0
#define LV_FONT_MONTSERRAT_28  0
#define LV_FONT_MONTSERRAT_30  0
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_34  0
#define LV_FONT_MONTSERRAT_36  0
#define LV_FONT_MONTSERRAT_38  0
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_MONTSERRAT_42  0
#define LV_FONT_MONTSERRAT_44  0
#define LV_FONT_MONTSERRAT_46  0
#define LV_FONT_MONTSERRAT_48  0

#define LV_FONT_MONTSERRAT_12_SUBPX      0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_16_CJK            0
#define LV_FONT_UNSCII_8                 0
#define LV_FONT_UNSCII_16                0

#define LV_FONT_CUSTOM_DECLARE  /* nothing */

/* Default font for labels that don't specify one */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

#define LV_FONT_FMT_TXT_LARGE  0
#define LV_USE_FONT_SUBPX      0
#define LV_FONT_SUBPX_BGR      0

/*==================
 * TEXT SETTINGS
 *==================*/
#define LV_TXT_ENC                       LV_TXT_ENC_UTF8
#define LV_TXT_BREAK_CHARS               " ,.;:-_"
#define LV_TXT_LINE_BREAK_LONG_LEN       0
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN  3
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3
#define LV_TXT_COLOR_CMD                 "#"
#define LV_USE_BIDI                      0
#define LV_USE_ARABIC_PERSIAN_CHARS      0

/*==================
 * WIDGET USAGE
 *==================*/
#define LV_USE_ARC       1
#define LV_USE_BAR       1
#define LV_USE_BTN       1
#define LV_USE_BTNMATRIX 0
#define LV_USE_CANVAS    0
#define LV_USE_CHECKBOX  1
#define LV_USE_DROPDOWN  0
#define LV_USE_IMG       0
#define LV_USE_LABEL     1
#if LV_USE_LABEL
  #define LV_LABEL_TEXT_SELECTION 0
  #define LV_LABEL_LONG_TXT_HINT  0
#endif
#define LV_USE_LINE      0
#define LV_USE_ROLLER    0
#define LV_USE_SLIDER    1
#define LV_USE_SWITCH    0
#define LV_USE_TEXTAREA  0
#define LV_USE_TABLE     0

/*==================
 * EXTRA COMPONENTS
 *==================*/
#define LV_USE_ANIMIMG    0
#define LV_USE_CALENDAR   0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN     0
#define LV_USE_KEYBOARD   0
#define LV_USE_LED        0
#define LV_USE_LIST       0
#define LV_USE_MENU       0
#define LV_USE_METER      0
#define LV_USE_MSGBOX     0
#define LV_USE_SPAN       0
#define LV_USE_SPINBOX    0
#define LV_USE_SPINNER    0
#define LV_USE_TABVIEW    0
#define LV_USE_TILEVIEW   0
#define LV_USE_WIN        0

/*==================
 * THEMES
 *==================*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
  #define LV_THEME_DEFAULT_DARK            1
  #define LV_THEME_DEFAULT_GROW            0
  #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif
#define LV_USE_THEME_SIMPLE 0
#define LV_USE_THEME_MONO   0

/*==================
 * LAYOUTS
 *==================*/
#define LV_USE_FLEX 1
#define LV_USE_GRID 0

/*========================
 * 3RD PARTY LIBRARIES
 *========================*/
#define LV_USE_FS_STDIO   0
#define LV_USE_FS_POSIX   0
#define LV_USE_FS_WIN32   0
#define LV_USE_FS_FATFS   0

#define LV_USE_PNG   0
#define LV_USE_BMP   0
#define LV_USE_SJPG  0
#define LV_USE_GIF   0
#define LV_USE_QRCODE 0

/*==================
 * GPU DRIVERS
 *==================*/
#define LV_USE_GPU_STM32_DMA2D  0
#define LV_USE_GPU_SWM341_DMAS  0
#define LV_USE_GPU_NXP_PXP      0
#define LV_USE_GPU_NXP_VG_LITE  0
#define LV_USE_GPU_SDL          0

/*=====================
 * COMPILER SETTINGS
 *=====================*/
#define LV_BIG_ENDIAN_SYSTEM         0
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE  1
#define LV_ATTRIBUTE_MEM_ALIGN       /* nothing */
#define LV_ATTRIBUTE_LARGE_CONST     /* nothing */
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY /* nothing */
#define LV_ATTRIBUTE_FAST_MEM        /* nothing */
#define LV_ATTRIBUTE_DMA             /* nothing */
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning
#define LV_USE_LARGE_COORD 0

#endif /* LV_CONF_H */

#endif /* End of "Content enable" */
