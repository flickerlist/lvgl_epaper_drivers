#include "epdiy_epaper.h"
#include "epd_driver.h"
#include "epd_highlevel.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>
#include <vector>

using namespace std;

EpdiyHighlevelState hl;
uint16_t            flushcalls = 0;
uint8_t*            framebuffer;
uint8_t             temperature       = 25;
const int           _clear_cycle_time = 12;
// MODE_DU: Fast monochrome | MODE_GC16 slow with 16 grayscales
enum EpdDrawMode updateMode = MODE_DU;

epdiy_flush_type_cb_t _epdiy_flush_type_cb;
TaskHandle_t          _paint_task_handle;
void buf_copy_to_framebuffer(EpdRect image_area, const uint8_t* image_data);
void paint_task_cb(void* arg);

typedef struct _paint_t {
  lvgl_epdiy_flush_type_t paint_type;
  EpdRect                 area;
  lv_color_t*             color_map;
  lv_disp_drv_t*          drv;
} paint_t;

#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t epdiy_pm_lock;
#endif  // CONFIG_PM_ENABLE

vector<paint_t> paint_queue;
bool            whole_repainting = false;  // Whole repaint task

/* Display initialization routine */
void epdiy_init(void) {
  epd_init(EPD_OPTIONS_DEFAULT);
  hl          = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  framebuffer = epd_hl_get_framebuffer(&hl);

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "epdiy_pm_lock",
                                     &epdiy_pm_lock));

  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE

  //   Clear all always in init:
  epd_poweron();
  epd_clear_area_cycles(epd_full_screen(), 2, _clear_cycle_time);
  epd_poweroff();

  xTaskCreatePinnedToCore(&paint_task_cb, "paint_cb", 1024 * 4, NULL, 5,
                          &_paint_task_handle, 1);

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE
}

/* Suggested by @kisvegabor https://forum.lvgl.io/t/lvgl-port-to-be-used-with-epaper-displays/5630/26 */
void buf_area_to_framebuffer(const lv_area_t* area, const uint8_t* image_data) {
  assert(framebuffer != NULL);
  uint8_t*   fb_ptr = &framebuffer[area->y1 * EPD_WIDTH / 2 + area->x1 / 2];
  lv_coord_t img_w  = lv_area_get_width(area);
  for (uint32_t y = area->y1; y < area->y2; y++) {
    memcpy(fb_ptr, image_data, img_w / 2);
    fb_ptr += EPD_WIDTH / 2;
    image_data += img_w / 2;
  }
}

/* A copy from epd_copy_to_framebuffer with temporary lenght prediction */
void buf_copy_to_framebuffer(EpdRect image_area, const uint8_t* image_data) {
  assert(framebuffer != NULL);

  for (uint32_t i = 0; i < image_area.width * image_area.height; i++) {
    uint8_t val = image_data[i] ? 0xff : 0x00;

    int xx = image_area.x + i % image_area.width;
    if (xx < 0 || xx >= EPD_WIDTH) {
      continue;
    }
    int yy = image_area.y + i / image_area.width;
    if (yy < 0 || yy >= EPD_HEIGHT) {
      continue;
    }
    uint8_t* buf_ptr = &framebuffer[yy * EPD_WIDTH / 2 + xx / 2];
    if (xx % 2) {
      *buf_ptr = (*buf_ptr & 0x0F) | (val << 4);
    } else {
      *buf_ptr = (*buf_ptr & 0xF0) | val;
    }
  }
}

/* Required by LVGL. Sends the color_map to the screen with a partial update  */
void epdiy_flush(lv_disp_drv_t*   drv,
                 const lv_area_t* area,
                 lv_color_t*      color_map) {
  ++flushcalls;
  uint16_t w = lv_area_get_width(area);
  uint16_t h = lv_area_get_height(area);

  EpdRect update_area = {
    .x = (uint16_t)area->x1, .y = (uint16_t)area->y1, .width = w, .height = h};

  lvgl_epdiy_flush_type_t _paint_type =
    _epdiy_flush_type_cb ? _epdiy_flush_type_cb(&update_area, flushcalls) :
                           EPDIY_PARTIAL_PAINT;
  if (_paint_type == EPDIY_NO_PAINT) {
    lv_disp_flush_ready(drv);
    return;
  }

  paint_t ptr;
  ptr.paint_type = _paint_type;
  ptr.area       = update_area;
  ptr.color_map  = color_map;
  ptr.drv        = drv;
  paint_queue.push_back(ptr);
  vTaskResume(_paint_task_handle);
  // Wait for a while to let `vTaskResume` run after `vTaskSuspend`
  vTaskDelay(pdMS_TO_TICKS(2));
  vTaskResume(_paint_task_handle);
}

/*
 * Called for each pixel. Designed with the idea to fill the buffer directly, not to set each pixel, see LVGL Forum (buf_area_to_framebuffer)
 *
 * !!! Useless for monochrome
*/
void epdiy_set_px_cb(lv_disp_drv_t* disp_drv,
                     uint8_t*       buf,
                     lv_coord_t     buf_w,
                     lv_coord_t     x,
                     lv_coord_t     y,
                     lv_color_t     color,
                     lv_opa_t       opa) {
  uint8_t epd_color = color.full;
  if (epd_color < 250 && updateMode == MODE_DU) {
    epd_color = 0;
  }
  //Instead of using epd_draw_pixel: Set pixel directly in *buf that comes afterwards in flush as *color_map
  uint32_t idx = y * buf_w / 2 + x / 2;
  if (x % 2) {
    buf[idx] = (buf[idx] & 0x0F) | (epd_color & 0xF0);
  } else {
    buf[idx] = (buf[idx] & 0xF0) | (epd_color >> 4);
  }
}

void set_epdiy_flush_type_cb(epdiy_flush_type_cb_t cb) {
  _epdiy_flush_type_cb = cb;
}

int _paint_empty_record = 0;
// This will be faster than create a task for each paint
void paint_task_cb(void* arg) {
  while (true) {
    if (paint_queue.size()) {
      _paint_empty_record = 0;
      auto first          = paint_queue.begin();
      /**
       * buf_copy_to_framebuffer must be called in same thread with `epd_hl_update_area`, or will cause paint buffer wrong data
       */
      uint8_t* buf = (uint8_t*)first->color_map;
      buf_copy_to_framebuffer(first->area, buf);
      /**
     * This seems will destroy `color_map`, so call after used `color_map`
     * epdiy_flush will only be called after lv_disp_flush_ready, so the `paint_queue` will no larger than 1
     */
      lv_disp_flush_ready(first->drv);

#if CONFIG_PM_ENABLE
      ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE

      if (first->paint_type == EPDIY_REPAINT_ALL) {
        epdiy_repaint(first->area);
      } else {
        epd_poweron();
        epd_hl_update_area(&hl, updateMode, temperature, first->area);
        epd_poweroff();
      }

#if CONFIG_PM_ENABLE
      ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE

      // Must after used, or will change `first` to the second item
      paint_queue.erase(paint_queue.begin());
    } else if (whole_repainting) {
      _paint_empty_record = 0;
      epdiy_repaint(epd_full_screen());
      whole_repainting = false;
    } else {
      _paint_empty_record++;
      if (_paint_empty_record >= 10) {
        // Check again, avoid parellel `vTaskResume` called before `vTaskSuspend`
        if (!paint_queue.size() && !whole_repainting) {
          vTaskSuspend(_paint_task_handle);
        }
      } else {
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }
  }
  vTaskDelete(_paint_task_handle);
}

/* refresh all screen */
void epdiy_repaint_all() {
  whole_repainting = true;
  vTaskResume(_paint_task_handle);
  // Wait for a while to let `vTaskResume` run after `vTaskSuspend`
  vTaskDelay(pdMS_TO_TICKS(2));
  vTaskResume(_paint_task_handle);
}

/* refresh area */
void epdiy_repaint(EpdRect area) {
  epd_poweron();
  epd_clear_area_cycles(area, 1, _clear_cycle_time);
  epd_hl_update_area_directly(&hl, updateMode, temperature, area);
  epd_poweroff();
}
