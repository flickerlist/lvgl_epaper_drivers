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

#include "epd_driver.h"

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
 * @brief To determine if the change need to flush this time
 *
 */
typedef bool (*need_flush_cb)(EpdRect* area, int flush_count);
void epdiy_set_need_flush_cb(need_flush_cb cb);

/**
 * @brief To determine if the paint need repaint this time
 */
typedef bool (*need_repaint_cb)(EpdRect* area, int flush_count);
void epdiy_set_need_repaint_cb(need_repaint_cb cb);

/* refresh all screen */
void epdiy_repaint_all();

/* refresh area */
void epdiy_repaint(EpdRect area);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* EPDIY_H */
