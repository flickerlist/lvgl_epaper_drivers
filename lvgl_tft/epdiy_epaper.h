/**
 * Display class for generic e-Paper driven by EPDiy class
*/
#ifndef EPDIY_H
#define EPDIY_H

#define EPDIY_COLUMNS (LV_HOR_RES_MAX / 8)

#ifdef __cplusplus
extern "C" {
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
  #include "lvgl.h"
#else
  #include "lvgl/lvgl.h"
#endif
#include "sdkconfig.h"

#ifndef EPDIY_ENABLE_16_GRAYSCALE
  #ifdef CONFIG_LV_EPAPER_EPDIY_16_GRAYSCALE
    #define EPDIY_ENABLE_16_GRAYSCALE CONFIG_LV_EPAPER_EPDIY_16_GRAYSCALE
  #else
    #define EPDIY_ENABLE_16_GRAYSCALE 0
  #endif
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
  #include <epdiy.h>
#else
  #include "epd_driver.h"
#endif

/* Configure your display */
void epdiy_init(void);

/* LVGL callbacks */
void epdiy_flush(lv_disp_drv_t*   drv,
                 const lv_area_t* area,
                 lv_color_t*      color_map);

/* Sets a pixel in *buf temporary buffer that comes afterwards in flush as *image_map */
void epdiy_set_px_cb(lv_disp_drv_t* disp_drv,
                     uint8_t*       buf,
                     lv_coord_t     buf_w,
                     lv_coord_t     x,
                     lv_coord_t     y,
                     lv_color_t     color,
                     lv_opa_t       opa);

/**
 * @brief To determine the type for this paint
 */
typedef int lvgl_epdiy_flush_type_t;
#define EPDIY_PARTIAL_PAINT 0
#define EPDIY_REPAINT_ALL 1
#define EPDIY_NO_PAINT 2
#define EPDIY_REPAINT_ALL_AFTER 3
typedef lvgl_epdiy_flush_type_t (*epdiy_flush_type_cb_t)(EpdRect* area,
                                                         int      flush_count);
void set_epdiy_flush_type_cb(epdiy_flush_type_cb_t cb);

/* Global grayscale switch. Disabled keeps the fast monochrome path. */
void epdiy_set_16_grayscale_enabled(bool enabled);
bool epdiy_is_16_grayscale_enabled();

/* refresh all screen */
void epdiy_repaint_all();

/**
 * @brief Check if epdiy paint thread can pause, call this in main loop
 *
 * @return true  : The task has been paused, can stop check
 * @return false : The task has not been paused, need to continue check
 */
bool epdiy_check_pause();

/**
 * @brief lock on poweron, for continue painting
 */
bool epdiy_auto_poweron();
void epdiy_lock_poweron();
void epdiy_unlock_poweron();
bool epdiy_is_locking_poweron();

/* set area to white */
void epdiy_clear_to_white(EpdRect area, int clear_count, int clear_cycle_time);

/* refresh area */
void epdiy_repaint(EpdRect area);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* EPDIY_H */
