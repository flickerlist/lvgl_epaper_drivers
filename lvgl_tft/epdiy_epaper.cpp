#include "epdiy_epaper.h"
#include "epd_highlevel.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <time.h>
#include <vector>

using namespace std;

#ifdef CONFIG_IDF_TARGET_ESP32
  #define USE_PARALLEL_PAINT 1
#endif

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
void epdiy_repaint_full_screen();

typedef struct _paint_t {
  lvgl_epdiy_flush_type_t paint_type;
  EpdRect                 area;
  lv_color_t*             color_map;
  lv_disp_drv_t*          drv;
  bool                    is_last;
} paint_t;

vector<paint_t>  paint_queue;
xSemaphoreHandle paint_queue_xMutex;  // lock for paint_queue
bool             whole_repainting = false;  // Whole repaint task

#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t epdiy_pm_lock;
#endif

/* Display initialization routine */
void epdiy_init(void) {
  paint_queue_xMutex = xSemaphoreCreateMutex();

  hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  epd_set_rotation(EPD_ROT_LANDSCAPE);
  framebuffer = epd_hl_get_framebuffer(&hl);

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "epdiy_pm_lock",
                                     &epdiy_pm_lock));
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif

  //   Clear all always in init:
  epd_poweron();
  epd_clear_area_cycles(epd_full_screen(), 2, _clear_cycle_time);
  epd_poweroff();

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif

#ifdef USE_PARALLEL_PAINT
  xTaskCreatePinnedToCore(&paint_task_cb, "paint_cb", 1024 * 4, NULL, 5,
                          &_paint_task_handle, 1);
#endif
}

/* A copy from epd_copy_to_framebuffer with temporary lenght prediction */
void buf_copy_to_framebuffer(EpdRect image_area, const uint8_t* image_data) {
  assert(framebuffer != NULL);

  auto display_width  = epd_rotated_display_width();
  auto display_height = epd_rotated_display_height();
  for (uint32_t i = 0; i < image_area.width * image_area.height; i++) {
    uint8_t val = image_data[i] ? 0xff : 0x00;

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

#ifdef USE_PARALLEL_PAINT

  // belowing code has same paint time
  paint_t ptr;
  ptr.paint_type = _paint_type;
  ptr.area       = update_area;
  ptr.color_map  = color_map;
  ptr.drv        = drv;
  ptr.is_last    = lv_disp_flush_is_last(drv);
  if (xSemaphoreTake(paint_queue_xMutex, pdMS_TO_TICKS(30))) {
    paint_queue.push_back(ptr);
    xSemaphoreGive(paint_queue_xMutex);
  }
  vTaskResume(_paint_task_handle);

#else

  uint8_t* buf = (uint8_t*)color_map;
  buf_copy_to_framebuffer(update_area, buf);

  static int x1 = 65535, y1 = 65535, x2 = -1, y2 = -1;
  // capture the upper left and lower right corners
  if (area->x1 < x1)
    x1 = area->x1;
  if (area->y1 < y1)
    y1 = area->y1;
  if (area->x2 > x2)
    x2 = area->x2;
  if (area->y2 > y2)
    y2 = area->y2;

  if (lv_disp_flush_is_last(drv)) {
    lv_disp_flush_ready(drv);

    // reset area
    update_area.x      = x1;
    update_area.y      = y1;
    update_area.width  = (x2 - x1) + 1;
    update_area.height = (y2 - y1) + 1;

  #if CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
  #endif

    if (_paint_type == EPDIY_REPAINT_ALL) {
      epdiy_repaint(update_area);
    } else {
      epd_poweron();
      epd_hl_update_area(&hl, updateMode, temperature, update_area);
      epd_poweroff();
    }

  #if CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
  #endif
    // reset update boundary
    x1 = y1 = 65535;
    x2 = y2 = -1;
  } else {
    lv_disp_flush_ready(drv);
  }
#endif
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

      static int x1 = 65535, y1 = 65535, x2 = -1, y2 = -1;
      auto       area = first->area;

      uint8_t* buf = (uint8_t*)first->color_map;
      buf_copy_to_framebuffer(area, buf);

      /**
     * This seems will destroy `color_map`, so call after used `color_map`
     * epdiy_flush will only be called after lv_disp_flush_ready, so the `paint_queue` will no larger than 1
     */
      lv_disp_flush_ready(first->drv);

      // capture the upper left and lower right corners
      if (area.x < x1)
        x1 = area.x;
      if (area.y < y1)
        y1 = area.y;
      if (area.x + area.width > x2)
        x2 = area.x + area.width;
      if (area.y + area.height > y2)
        y2 = area.y + area.height;

      if (first->is_last) {
        // reset area
        area.x      = x1;
        area.y      = y1;
        area.width  = (x2 - x1) + 1;
        area.height = (y2 - y1) + 1;

#if CONFIG_PM_ENABLE
        ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif
        if (first->paint_type == EPDIY_REPAINT_ALL) {
          epdiy_repaint(area);
        } else {
          epd_poweron();
          epd_hl_update_area(&hl, updateMode, temperature, area);
          epd_poweroff();
        }
#if CONFIG_PM_ENABLE
        ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif

        // reset update boundary
        x1 = y1 = 65535;
        x2 = y2 = -1;
      }

      // Must after used, or will change `first` to the second item
      if (xSemaphoreTake(paint_queue_xMutex, pdMS_TO_TICKS(30))) {
        paint_queue.erase(paint_queue.begin());
        xSemaphoreGive(paint_queue_xMutex);
      }
    } else if (whole_repainting) {
      _paint_empty_run_count = 0;

#if CONFIG_PM_ENABLE
      ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif

      epdiy_repaint_full_screen();

#if CONFIG_PM_ENABLE
      ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif

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
#ifndef USE_PARALLEL_PAINT
  return true;
#endif
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
#ifdef USE_PARALLEL_PAINT
  whole_repainting = true;
  vTaskResume(_paint_task_handle);
#else
  #if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
  #endif

  epdiy_repaint_full_screen();

  #if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
  #endif
#endif
}

void epdiy_repaint_full_screen() {
#ifdef CONFIG_IDF_TARGET_ESP32S3

  memset(hl.back_fb, 0xFF, epd_width() / 2 * epd_height());

  epd_poweron();
  auto area = epd_full_screen();
  epd_clear_area_cycles(area, 1, _clear_cycle_time);
  epd_hl_update_area(&hl, updateMode, temperature, area);
  epd_poweroff();

#elif

  epdiy_repaint(epd_full_screen());

#endif
}

/* refresh area */
void epdiy_repaint(EpdRect area) {
  epd_poweron();
#ifdef CONFIG_IDF_TARGET_ESP32S3
  epd_hl_update_area(&hl, updateMode, temperature, area);
#else
  epd_clear_area_cycles(area, 1, _clear_cycle_time);
  epd_hl_update_area_directly(&hl, updateMode, temperature, area);
#endif
  epd_poweroff();
}
