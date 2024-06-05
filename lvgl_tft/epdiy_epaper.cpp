#include "epdiy_epaper.h"
#include "epd_highlevel.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <epdiy.h>
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

vector<paint_t>  paint_queue;
xSemaphoreHandle paint_queue_xMutex;  // lock for paint_queue
bool             whole_repainting = false;  // Whole repaint task

#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t epdiy_pm_lock;
#endif  // CONFIG_PM_ENABLE

/* Display initialization routine */
void epdiy_init(void) {
  paint_queue_xMutex = xSemaphoreCreateMutex();

  epd_init(&epd_board_v7, &ED047TC2, EPD_LUT_64K);
  epd_set_vcom(1560);

  hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  epd_set_rotation(EPD_ROT_LANDSCAPE);
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

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE

  xTaskCreatePinnedToCore(&paint_task_cb, "paint_cb", 1024 * 4, NULL, 5,
                          &_paint_task_handle, 1);
}

/* A copy from epd_copy_to_framebuffer with temporary lenght prediction */
void buf_copy_to_framebuffer(EpdRect image_area, const uint8_t* image_data) {
  assert(framebuffer != NULL);

  auto display_width  = epd_rotated_display_width();
  auto display_height = epd_rotated_display_height();
  for (uint32_t i = 0; i < image_area.width * image_area.height; i++) {
    uint8_t val = image_data[i] > 0x99 ? 0xff : 0x00;

    int xx = image_area.x + i % image_area.width;
    if (xx < 0 || xx >= display_width) {
      continue;
    }
    int yy = image_area.y + i / image_area.width;
    if (yy < 0 || yy >= display_height) {
      continue;
    }
    uint8_t* buf_ptr = &framebuffer[yy * display_width / 2 + xx / 2];
    if (xx % 2) {
      *buf_ptr = (*buf_ptr & 0x0F) | (val << 4);
    } else {
      *buf_ptr = (*buf_ptr & 0xF0) | (val >> 4);
    }
  }
}

/* Required by LVGL. Sends the color_map to the screen with a partial update  */
void epdiy_flush(lv_disp_drv_t*   drv,
                 const lv_area_t* area,
                 lv_color_t*      color_map) {
  ++flushcalls;
  static int x1 = 65535, y1 = 65535, x2 = -1, y2 = -1;
  ESP_LOGW("****", "epdiy_flush start x:%d y:%d width:%d height:%d; time: %ld",
           area->x1, area->y1, area->x2 - area->x1, area->y2 - area->y1,
           clock());
  uint16_t w = lv_area_get_width(area);
  uint16_t h = lv_area_get_height(area);

  EpdRect update_area = {
    .x = (uint16_t)area->x1, .y = (uint16_t)area->y1, .width = w, .height = h};

  // capture the upper left and lower right corners
  if (area->x1 < x1)
    x1 = area->x1;
  if (area->y1 < y1)
    y1 = area->y1;
  if (area->x2 > x2)
    x2 = area->x2;
  if (area->y2 > y2)
    y2 = area->y2;

  lvgl_epdiy_flush_type_t _paint_type =
    _epdiy_flush_type_cb ? _epdiy_flush_type_cb(&update_area, flushcalls) :
                           EPDIY_PARTIAL_PAINT;
  if (_paint_type == EPDIY_NO_PAINT) {
    lv_disp_flush_ready(drv);
    return;
  }

  uint8_t* buf = (uint8_t*)color_map;
  ESP_LOGW(
    "####",
    "epdiy_flush area x:%d y:%d width:%d height:%d; buf_copy_to_framebuffer "
    "before: %ld",
    update_area.x, update_area.y, update_area.width, update_area.height,
    clock());
  buf_copy_to_framebuffer(update_area, buf);
  ESP_LOGW(
    "####",
    "epdiy_flush area x:%d y:%d width:%d height:%d; buf_copy_to_framebuffer "
    "after: %ld",
    update_area.x, update_area.y, update_area.width, update_area.height,
    clock());
  /**
     * This seems will destroy `color_map`, so call after used `color_map`
     * epdiy_flush will only be called after lv_disp_flush_ready, so the `paint_queue` will no larger than 1
     */

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE
  if (lv_disp_flush_is_last(drv)) {
    lv_disp_flush_ready(drv);

    // if (_paint_type == EPDIY_REPAINT_ALL) {
    //   epdiy_repaint(update_area);
    // } else {
    update_area.x      = x1;
    update_area.y      = y1;
    update_area.width  = (x2 - x1) + 1;
    update_area.height = (y2 - y1) + 1;
    epd_poweron();
    epd_hl_update_area(&hl, updateMode, temperature, update_area);
    epd_poweroff();

#if CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE

    ESP_LOGW("----",
             "epdiy_flush paint area x:%d y:%d width:%d height:%d; time:%ld",
             update_area.x, update_area.y, update_area.width,
             update_area.height, clock());
    x1 = y1 = 65535;
    x2 = y2 = -1;  // reset update boundary
  } else {
    lv_disp_flush_ready(drv);
  }
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

// -1 means is suspending, 0 means has task running
int _paint_empty_run_count = 0;
// This will be faster than create a task for each paint
void paint_task_cb(void* arg) {
  while (true) {
    if (paint_queue.size()) {
      _paint_empty_run_count = 0;
      auto first             = paint_queue.begin();
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
      if (xSemaphoreTake(paint_queue_xMutex, pdMS_TO_TICKS(30))) {
        paint_queue.erase(paint_queue.begin());
        xSemaphoreGive(paint_queue_xMutex);
      }
    } else if (whole_repainting) {
      _paint_empty_run_count = 0;

#if CONFIG_PM_ENABLE
      ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE

      epdiy_repaint(epd_full_screen());

#if CONFIG_PM_ENABLE
      ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif  // CONFIG_PM_ENABLE

      whole_repainting = false;
    } else {
      _paint_empty_run_count++;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  vTaskDelete(_paint_task_handle);
}

/* Check if epdiy paint thread can pause */
bool epdiy_check_pause() {
  if (paint_queue.size() || whole_repainting) {
    return false;
  }
  if (_paint_empty_run_count >= 3) {
    _paint_empty_run_count = -1;
    vTaskSuspend(_paint_task_handle);
    return true;
  } else if (_paint_empty_run_count == -1) {
    return true;
  }
  return false;
}

/* refresh all screen */
void epdiy_repaint_all() {
  whole_repainting = true;
  vTaskResume(_paint_task_handle);
}

void epdiy_set_area_to_white(EpdRect& area) {
  for (int l = area.y; l < area.y + area.height; l++) {
    uint8_t* lfb = hl.front_fb + epd_width() / 2 * l;
    uint8_t* lbb = hl.back_fb + epd_width() / 2 * l;

    for (int x = area.x; x < area.x + area.width; x++) {
      if (x % 2) {
        *(lbb + x / 2) = 0xF0 | (*(lbb + x / 2) & 0x0F);
      } else {
        *(lbb + x / 2) = 0x0F | (*(lbb + x / 2) & 0xF0);
      }
    }
  }
}

/* refresh area */
void epdiy_repaint(EpdRect area) {
  epd_poweron();

  // copy from epd_clear_area_cycles
  const short white_time = _clear_cycle_time * 2;
  const short dark_time  = _clear_cycle_time * 5;

  for (int c = 0; c < 1; c++) {
    for (int i = 0; i < 10; i++) {
      epd_push_pixels(area, dark_time, 0);
    }
    for (int i = 0; i < 10; i++) {
      epd_push_pixels(area, white_time, 1);
    }
  }
  epdiy_set_area_to_white(area);

  epd_clear_area_cycles(area, 1, _clear_cycle_time);
  epd_hl_update_area_directly(&hl, updateMode, temperature, area);

  //   epd_hl_update_area(&hl, updateMode, temperature, area);

  epd_poweroff();
}
