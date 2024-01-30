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
#  include "lvgl.h"
#else
#  include "lvgl/lvgl.h"
#endif
#include "sdkconfig.h"

#include <epdiy.h>

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
typedef lvgl_epdiy_flush_type_t (*epdiy_flush_type_cb_t)(EpdRect* area,
                                                         int      flush_count);
void set_epdiy_flush_type_cb(epdiy_flush_type_cb_t cb);

/* refresh all screen */
void epdiy_repaint_all();

/**
 * @brief Check if epdiy paint thread can pause, call this in main loop
 *
 * @return true  : The task has been paused, can stop check
 * @return false : The task has not been paused, need to continue check
 */
bool epdiy_check_pause();

/* refresh area */
void epdiy_repaint(EpdRect area);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* EPDIY_H */
