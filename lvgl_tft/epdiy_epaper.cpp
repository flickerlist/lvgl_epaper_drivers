#include "epdiy_epaper.h"
#include "epd_highlevel.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <time.h>
#include <vector>

using namespace std;

#define TAG "epdiy_epaper"

#ifdef CONFIG_IDF_TARGET_ESP32
  #define USE_PARALLEL_PAINT 1
#endif

EpdiyHighlevelState hl;
uint16_t            flushcalls = 0;
uint8_t*            framebuffer;
uint8_t             temperature       = 25;
const int           _clear_cycle_time = 12;
static int          s_lcd_pclk_mhz    = 20;
static const int    EPDIY_LCD_PCLK_MIN_MHZ = 10;
static const int    EPDIY_LCD_PCLK_STEP_MHZ = 2;
static bool         s_16_grayscale_enabled = EPDIY_ENABLE_16_GRAYSCALE;

epdiy_flush_type_cb_t _epdiy_flush_type_cb;
TaskHandle_t          _paint_task_handle;
void buf_copy_to_framebuffer(EpdRect image_area, const lv_color_t* image_data);
void paint_task_cb(void* arg);
enum EpdDrawError epdiy_repaint_full_screen(bool need_power = true);
static bool epdiy_handle_draw_error(enum EpdDrawError err, const char* stage);
static void epdiy_force_full_repaint_after_draw_error(enum EpdDrawError err,
                                                      const char* stage);
static enum EpdDrawMode epdiy_current_update_mode();
static uint8_t epdiy_color_to_gray4(lv_color_t color);
static void epdiy_mark_pending_update(EpdRect area,
                                      const char* stage,
                                      int clear_count);
static void epdiy_pending_update_retry_task(void* arg);
static void epdiy_schedule_pending_update_retry();

typedef struct _paint_t {
  lvgl_epdiy_flush_type_t paint_type;
  EpdRect                 area;
  lv_color_t*             color_map;
  lv_disp_drv_t*          drv;
  bool                    is_last;
} paint_t;

vector<paint_t>          paint_queue;
static SemaphoreHandle_t paint_queue_xMutex = NULL;  // lock for paint_queue
static SemaphoreHandle_t epdiy_update_xMutex = NULL;
bool                     whole_repainting   = false;  // Whole repaint task

static const int EPDIY_PENDING_RETRY_DELAY_MS = 500;
static bool      s_pending_update_valid        = false;
static EpdRect   s_pending_update_area         = {0, 0, 0, 0};
static bool      s_pending_update_use_gc16     = false;
static int       s_pending_update_clear_count  = 0;
static bool      s_pending_retry_task_running  = false;
static const enum EpdDrawError EPDIY_DRAW_POWER_VERIFY_FAILED =
  (enum EpdDrawError)0x800;

#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t epdiy_pm_lock;
#endif

/* Display initialization routine */
void epdiy_init(void) {
  epdiy_update_xMutex = xSemaphoreCreateRecursiveMutex();

#ifdef USE_PARALLEL_PAINT
  paint_queue_xMutex = xSemaphoreCreateMutex();
#endif

  hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  epd_set_rotation(EPD_ROT_LANDSCAPE);
  framebuffer = epd_hl_get_framebuffer(&hl);
  s_lcd_pclk_mhz = epd_get_display()->bus_speed;
  epdiy_set_16_grayscale_enabled(false);

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "epdiy_pm_lock",
                                     &epdiy_pm_lock));
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif

  //   Clear all always in init:
  if (epdiy_auto_poweron()) {
    epd_clear_area_cycles(epd_full_screen(), 2, _clear_cycle_time);
  } else {
    epdiy_mark_pending_update(epd_full_screen(), "init clear", 2);
  }
  if (!epdiy_is_locking_poweron()) {
    epd_poweroff();
  }

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif

#ifdef USE_PARALLEL_PAINT
  xTaskCreatePinnedToCore(&paint_task_cb, "paint_cb", 1024 * 4, NULL, 5,
                          &_paint_task_handle, 1);
#endif
}

static bool epdiy_reduce_lcd_pclk_after_underrun(const char* stage) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  int next_pclk = s_lcd_pclk_mhz > EPDIY_LCD_PCLK_MIN_MHZ ?
                    s_lcd_pclk_mhz - EPDIY_LCD_PCLK_STEP_MHZ :
                    EPDIY_LCD_PCLK_MIN_MHZ;
  if (next_pclk != s_lcd_pclk_mhz) {
    s_lcd_pclk_mhz = next_pclk;
    ESP_LOGW(TAG, "%s underrun, reduce lcd pixel clock to %d MHz",
             stage ? stage : "draw", s_lcd_pclk_mhz);
    epd_set_lcd_pixel_clock_MHz(s_lcd_pclk_mhz);
    return true;
  }
  ESP_LOGW(TAG, "%s underrun, lcd pixel clock already at minimum %d MHz",
           stage ? stage : "draw", s_lcd_pclk_mhz);
#endif
  return false;
}

static bool epdiy_handle_draw_error(enum EpdDrawError err, const char* stage) {
  if (err == EPD_DRAW_SUCCESS) {
    return false;
  }
  ESP_LOGW(TAG, "%s failed, draw error=0x%x", stage ? stage : "draw",
           (unsigned)err);
  if (err & EPD_DRAW_EMPTY_LINE_QUEUE) {
    epdiy_reduce_lcd_pclk_after_underrun(stage);
    return true;
  }
  return false;
}

static void epdiy_force_full_repaint_after_draw_error(enum EpdDrawError err,
                                                      const char* stage) {
  if (!epdiy_handle_draw_error(err, stage)) {
    return;
  }

  ESP_LOGW(TAG, "%s underrun, force full screen repaint",
           stage ? stage : "draw");
  epdiy_repaint_full_screen(false);
}

static enum EpdDrawMode epdiy_current_update_mode() {
  // MODE_DU: fast monochrome; MODE_GL16: slower non-flashing 16 grayscale.
  return epdiy_is_16_grayscale_enabled() ? MODE_GL16 : MODE_DU;
}

static uint8_t epdiy_color_to_gray4(lv_color_t color) {
  if (!epdiy_is_16_grayscale_enabled()) {
    return lv_color_to1(color) ? 0x0F : 0x00;
  }

  uint8_t brightness = lv_color_brightness(color);
  return (brightness + 8) / 17;
}

static bool epdiy_take_update_lock(TickType_t timeout_ticks) {
  if (!epdiy_update_xMutex) {
    return true;
  }
  return xSemaphoreTakeRecursive(epdiy_update_xMutex, timeout_ticks) == pdTRUE;
}

static void epdiy_give_update_lock() {
  if (epdiy_update_xMutex) {
    xSemaphoreGiveRecursive(epdiy_update_xMutex);
  }
}

static bool epdiy_area_is_valid(EpdRect area) {
  return area.width > 0 && area.height > 0;
}

static EpdRect epdiy_merge_area(EpdRect first, EpdRect second) {
  int x1 = first.x < second.x ? first.x : second.x;
  int y1 = first.y < second.y ? first.y : second.y;
  int x2_first = first.x + first.width;
  int y2_first = first.y + first.height;
  int x2_second = second.x + second.width;
  int y2_second = second.y + second.height;
  int x2 = x2_first > x2_second ? x2_first : x2_second;
  int y2 = y2_first > y2_second ? y2_first : y2_second;

  EpdRect merged = {
    .x = x1, .y = y1, .width = x2 - x1, .height = y2 - y1};
  return merged;
}

static void epdiy_log_area(const char* prefix, const char* stage, EpdRect area) {
  ESP_LOGW(TAG, "%s %s area x=%d y=%d w=%d h=%d", stage ? stage : "update",
           prefix, (int)area.x, (int)area.y, (int)area.width,
           (int)area.height);
}

static EpdRect epdiy_area_with_pending(EpdRect area, const char* stage) {
  if (!s_pending_update_valid) {
    return area;
  }

  EpdRect merged = epdiy_merge_area(area, s_pending_update_area);
  epdiy_log_area("merge pending", stage, merged);
  return merged;
}

static int epdiy_pending_clear_count_with(int requested_clear_count) {
  return requested_clear_count > s_pending_update_clear_count ?
           requested_clear_count :
           s_pending_update_clear_count;
}

static void epdiy_mark_pending_update(EpdRect area,
                                      const char* stage,
                                      int clear_count) {
  if (!epdiy_area_is_valid(area)) {
    return;
  }

  bool use_gc16 = epdiy_is_16_grayscale_enabled();
  clear_count = epdiy_pending_clear_count_with(clear_count);
  if (s_pending_update_valid) {
    area = epdiy_merge_area(area, s_pending_update_area);
    use_gc16 = use_gc16 || s_pending_update_use_gc16;
  }

  s_pending_update_area     = area;
  s_pending_update_use_gc16 = use_gc16;
  s_pending_update_clear_count = clear_count;
  s_pending_update_valid    = true;
  if (clear_count > 0) {
    epdiy_log_area(use_gc16 ? "pending clear GC16" : "pending clear DU", stage,
                   area);
  } else {
    epdiy_log_area(use_gc16 ? "pending GC16" : "pending DU", stage, area);
  }
  epdiy_schedule_pending_update_retry();
}

static void epdiy_clear_pending_update() {
  s_pending_update_valid    = false;
  s_pending_update_use_gc16 = false;
  s_pending_update_clear_count = 0;
}

static bool epdiy_prepare_update_area(EpdRect requested_area,
                                      EpdRect* update_area,
                                      int* clear_count,
                                      const char* stage,
                                      int requested_clear_count) {
  EpdRect merged_area = epdiy_area_with_pending(requested_area, stage);
  int effective_clear_count =
    epdiy_pending_clear_count_with(requested_clear_count);
  if (update_area) {
    *update_area = merged_area;
  }
  if (clear_count) {
    *clear_count = effective_clear_count;
  }

  if (epdiy_auto_poweron()) {
    return true;
  }

  epdiy_mark_pending_update(merged_area, stage, effective_clear_count);
  return false;
}

static bool epdiy_verify_power_after_update(const char* stage) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (epd_poweron()) {
    return true;
  }
  ESP_LOGW(TAG, "%s power check failed after draw; keep update pending",
           stage ? stage : "draw");
  return false;
#else
  return true;
#endif
}

static enum EpdDrawError epdiy_update_prepared_area(EpdRect area,
                                                    const char* stage,
                                                    int clear_count) {
  enum EpdDrawMode mode =
    (s_pending_update_valid && s_pending_update_use_gc16) ?
      MODE_GL16 :
      epdiy_current_update_mode();
  if (clear_count > 0) {
    epdiy_log_area("clear before update", stage, area);
    epdiy_clear_to_white(area, clear_count, _clear_cycle_time);
  }
  enum EpdDrawError err = epd_hl_update_area(&hl, mode, temperature, area);
  if (err == EPD_DRAW_SUCCESS) {
    if (!epdiy_verify_power_after_update(stage)) {
      epdiy_mark_pending_update(area, stage, 1);
      return EPDIY_DRAW_POWER_VERIFY_FAILED;
    }
    epdiy_clear_pending_update();
  } else {
    epdiy_mark_pending_update(area, stage, clear_count > 0 ? clear_count : 1);
  }
  return err;
}

static void epdiy_schedule_pending_update_retry() {
  if (s_pending_retry_task_running || !s_pending_update_valid) {
    return;
  }

  s_pending_retry_task_running = true;
  BaseType_t ret =
    xTaskCreate(&epdiy_pending_update_retry_task, "epdiy_pending_retry",
                1024 * 4, NULL, 4, NULL);
  if (ret != pdPASS) {
    s_pending_retry_task_running = false;
    ESP_LOGW(TAG, "create pending update retry task failed");
  }
}

static void epdiy_pending_update_retry_task(void* arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(EPDIY_PENDING_RETRY_DELAY_MS));

  if (!epdiy_take_update_lock(pdMS_TO_TICKS(5000))) {
    s_pending_retry_task_running = false;
    epdiy_schedule_pending_update_retry();
    vTaskDelete(NULL);
    return;
  }

  s_pending_retry_task_running = false;
  if (!s_pending_update_valid) {
    epdiy_give_update_lock();
    vTaskDelete(NULL);
    return;
  }

  EpdRect area = s_pending_update_area;

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif

  EpdRect update_area;
  int     clear_count = 0;
  bool    did_poweron =
    epdiy_prepare_update_area(area, &update_area, &clear_count,
                              "pending retry", 0);
  if (did_poweron) {
    enum EpdDrawError err =
      epdiy_update_prepared_area(update_area, "pending retry", clear_count);
    if (err != EPD_DRAW_SUCCESS) {
      epdiy_force_full_repaint_after_draw_error(err, "pending retry");
    }
  }

  if (!epdiy_is_locking_poweron()) {
    epd_poweroff();
  }

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif

  bool needs_retry = s_pending_update_valid;
  epdiy_give_update_lock();
  if (needs_retry) {
    epdiy_schedule_pending_update_retry();
  }

  vTaskDelete(NULL);
}

/* A copy from epd_copy_to_framebuffer with temporary lenght prediction */
void buf_copy_to_framebuffer(EpdRect image_area, const lv_color_t* image_data) {
  assert(framebuffer != NULL);

  auto display_width  = epd_rotated_display_width();
  auto display_height = epd_rotated_display_height();
  for (uint32_t i = 0; i < image_area.width * image_area.height; i++) {
    uint8_t val = epdiy_color_to_gray4(image_data[i]);

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

#ifdef USE_PARALLEL_PAINT

  // belowing code has same paint time
  paint_t ptr;
  ptr.paint_type = _paint_type;
  ptr.area       = update_area;
  ptr.color_map  = color_map;
  ptr.drv        = drv;
  #ifdef CONFIG_IDF_TARGET_ESP32S3
  ptr.is_last = lv_disp_flush_is_last(drv);
  #else
  // v5 board 批量渲染效果不好，会整屏闪烁一次
  ptr.is_last = true;
  #endif
  if (xSemaphoreTake(paint_queue_xMutex, pdMS_TO_TICKS(30))) {
    paint_queue.push_back(ptr);
    vTaskResume(_paint_task_handle);
    xSemaphoreGive(paint_queue_xMutex);
  } else {
    vTaskResume(_paint_task_handle);
  }

#else

  epdiy_take_update_lock(portMAX_DELAY);
  buf_copy_to_framebuffer(update_area, color_map);

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
      int  clear_count = 0;
      bool update_ok   = false;
      bool did_poweron =
        epdiy_prepare_update_area(update_area, &update_area, &clear_count,
                                  "partial update", 0);
      if (did_poweron) {
        auto err =
          epdiy_update_prepared_area(update_area, "partial update",
                                     clear_count);
        update_ok = err == EPD_DRAW_SUCCESS;
        if (err != EPD_DRAW_SUCCESS) {
          epdiy_force_full_repaint_after_draw_error(err, "partial update");
        }
      }

      if (update_ok && _paint_type == EPDIY_REPAINT_ALL_AFTER) {
        epdiy_repaint_full_screen(false);
      }

      if (!epdiy_is_locking_poweron()) {
        epd_poweroff();
      }
    }

  #if CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
  #endif
    // reset update boundary
    x1 = y1 = 65535;
    x2 = y2 = -1;
    epdiy_give_update_lock();
  } else {
    lv_disp_flush_ready(drv);
    epdiy_give_update_lock();
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
  uint8_t epd_color = epdiy_color_to_gray4(color);
  //Instead of using epd_draw_pixel: Set pixel directly in *buf that comes afterwards in flush as *color_map
  uint32_t idx = y * buf_w / 2 + x / 2;
  if (x % 2) {
    buf[idx] = (buf[idx] & 0x0F) | (epd_color << 4);
  } else {
    buf[idx] = (buf[idx] & 0xF0) | epd_color;
  }
}

void set_epdiy_flush_type_cb(epdiy_flush_type_cb_t cb) {
  _epdiy_flush_type_cb = cb;
}

void epdiy_set_16_grayscale_enabled(bool enabled) {
  s_16_grayscale_enabled = enabled;
  ESP_LOGW(TAG, "16 grayscale %s", enabled ? "enabled" : "disabled");
}

bool epdiy_is_16_grayscale_enabled() {
  return s_16_grayscale_enabled;
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

      static int  x1 = 65535, y1 = 65535, x2 = -1, y2 = -1;
      static bool has_paint_all = false;
      auto        area          = first->area;

      epdiy_take_update_lock(portMAX_DELAY);
      buf_copy_to_framebuffer(area, first->color_map);

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

      auto _paint_type = first->paint_type;
      if (_paint_type == EPDIY_REPAINT_ALL) {
        has_paint_all = true;
      }

      if (first->is_last) {
        // reset area
        area.x      = x1;
        area.y      = y1;
        area.width  = (x2 - x1);
        area.height = (y2 - y1);

#if CONFIG_PM_ENABLE
        ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif
        if (has_paint_all) {
          epdiy_repaint(area);
        } else {
          int  clear_count = 0;
          bool update_ok   = false;
          bool did_poweron =
            epdiy_prepare_update_area(area, &area, &clear_count,
                                      "partial update", 0);
          if (did_poweron) {
            auto err =
              epdiy_update_prepared_area(area, "partial update", clear_count);
            update_ok = err == EPD_DRAW_SUCCESS;
            if (err != EPD_DRAW_SUCCESS) {
              epdiy_force_full_repaint_after_draw_error(err, "partial update");
            }
          }

          if (update_ok && _paint_type == EPDIY_REPAINT_ALL_AFTER) {
            epdiy_repaint_full_screen(false);
          }

          if (!epdiy_is_locking_poweron()) {
            epd_poweroff();
          }
        }
#if CONFIG_PM_ENABLE
        ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif

        // reset update boundary
        x1 = y1 = 65535;
        x2 = y2       = -1;
        has_paint_all = false;
      }

      // Must after used, or will change `first` to the second item
      if (xSemaphoreTake(paint_queue_xMutex, pdMS_TO_TICKS(30))) {
        paint_queue.erase(paint_queue.begin());
        xSemaphoreGive(paint_queue_xMutex);
      }
      epdiy_give_update_lock();
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

/**
 * @brief lock on poweron, for continue painting
 */
bool _is_locking_poweron = false;
bool epdiy_auto_poweron() {
  if (epdiy_is_locking_poweron()) {
    return true;
  }
#ifdef CONFIG_IDF_TARGET_ESP32S3
  return epd_poweron();
#else
  epd_poweron();
  return true;
#endif
}
void epdiy_lock_poweron() {
  int64_t start = esp_timer_get_time();
#ifdef CONFIG_IDF_TARGET_ESP32S3
  while (!epd_poweron()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
#else
  epd_poweron();
#endif
  ESP_LOGW(TAG, "epdiy_lock_poweron cost %lld ms",
           (esp_timer_get_time() - start) / 1000);
  _is_locking_poweron = true;
}
void epdiy_unlock_poweron() {
  if (_is_locking_poweron) {
    epd_poweroff();
    _is_locking_poweron = false;
    ESP_LOGW(TAG, "epdiy_unlock_poweron");
  }
}
bool epdiy_is_locking_poweron() {
  return _is_locking_poweron;
}

/* Check if epdiy paint thread can pause */
bool epdiy_check_pause() {
  if (s_pending_update_valid || s_pending_retry_task_running) {
    return false;
  }
#ifndef USE_PARALLEL_PAINT
  return true;
#endif
  if (paint_queue.size() || whole_repainting) {
    return false;
  }
  if (_paint_empty_run_count >= 3) {
    _paint_empty_run_count = -1;
    if (xSemaphoreTake(paint_queue_xMutex, pdMS_TO_TICKS(30))) {
      // recheck in xSemaphoreTake for safe `vTaskSuspend`
      if (paint_queue.size() || whole_repainting) {
        xSemaphoreGive(paint_queue_xMutex);
        return false;
      } else {
        vTaskSuspend(_paint_task_handle);
        xSemaphoreGive(paint_queue_xMutex);
        return true;
      }
    } else {
      return false;
    }
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

void epdiy_set_white(EpdRect area) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  int width = epd_width();
#else
  int width = epd_rotated_display_width();
#endif

  auto x1         = area.x;
  auto x2         = area.x + area.width;
  auto first_byte = x1 % 2 == 1 ? x1 / 2 + 1 : x1 / 2;  // 5 -> 3
  auto last_byte  = x2 / 2;  // 9 -> 4
  for (int y = area.y; y < area.y + area.height; y++) {
    uint8_t* line = hl.back_fb + width / 2 * y;

    memset(line + first_byte, 0xFF, last_byte - first_byte);
    if (x1 % 2 == 1) {
      // Odd x is stored in the high nibble of byte x / 2.
      *(line + x1 / 2) |= 0xF0;
    }
    if (x2 % 2 == 1) {
      // x2 is exclusive, so the last pixel in-range is the low nibble.
      *(line + x2 / 2) |= 0x0F;
    }
  }
}

/* set area to white */
void epdiy_clear_to_white(EpdRect area, int clear_count, int clear_cycle_time) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  epdiy_set_white(area);
  epd_clear_area_cycles(area, clear_count, clear_cycle_time);
#endif
}

void epdiy_set_framebuffer_gray4_pixel(int x, int y, uint8_t gray) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (!framebuffer) {
    return;
  }
  int display_width  = epd_rotated_display_width();
  int display_height = epd_rotated_display_height();
  if (x < 0 || y < 0 || x >= display_width || y >= display_height) {
    return;
  }

  gray &= 0x0F;
  uint8_t* buf_ptr = &framebuffer[y * display_width / 2 + x / 2];
  if (x % 2) {
    *buf_ptr = (*buf_ptr & 0x0F) | (gray << 4);
  } else {
    *buf_ptr = (*buf_ptr & 0xF0) | gray;
  }
#endif
}

int epdiy_update_framebuffer_area(EpdRect area) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  enum EpdDrawError err = EPD_DRAW_SUCCESS;

  epdiy_take_update_lock(portMAX_DELAY);

  #if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
  #endif

  EpdRect update_area;
  int     clear_count = 0;
  bool    did_poweron =
    epdiy_prepare_update_area(area, &update_area, &clear_count,
                              "framebuffer update", 0);
  if (did_poweron) {
    err = epdiy_update_prepared_area(update_area, "framebuffer update",
                                     clear_count);
    if (err != EPD_DRAW_SUCCESS) {
      epdiy_force_full_repaint_after_draw_error(err, "framebuffer update");
    }
  }
  if (!epdiy_is_locking_poweron()) {
    epd_poweroff();
  }

  #if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
  #endif

  epdiy_give_update_lock();

  return (int)err;
#else
  epdiy_repaint(area);
  return 0;
#endif
}

enum EpdDrawError epdiy_repaint_full_screen(bool need_power) {
#ifdef CONFIG_IDF_TARGET_ESP32S3

  enum EpdDrawError err = EPD_DRAW_SUCCESS;
  EpdRect           area = epd_full_screen();

  epdiy_take_update_lock(portMAX_DELAY);

  bool can_update  = true;
  int  clear_count = 1;
  if (need_power) {
    can_update =
      epdiy_prepare_update_area(area, &area, &clear_count, "full repaint", 1);
  } else {
    area = epdiy_area_with_pending(area, "full repaint");
    clear_count = epdiy_pending_clear_count_with(1);
  }

  if (can_update) {
    for (int attempt = 0; attempt < 4; attempt++) {
      err = epdiy_update_prepared_area(area, "full repaint", clear_count);
      if (err == EPD_DRAW_SUCCESS) {
        break;
      }
      bool can_retry = epdiy_handle_draw_error(err, "full repaint");
      if (!can_retry || s_lcd_pclk_mhz <= EPDIY_LCD_PCLK_MIN_MHZ) {
        break;
      }
      ESP_LOGW(TAG, "retry full repaint after reducing lcd pixel clock");
    }
  }
  if (need_power && !epdiy_is_locking_poweron()) {
    epd_poweroff();
  }
  epdiy_give_update_lock();
  return err;

#else

  epdiy_repaint(epd_full_screen());
  return EPD_DRAW_SUCCESS;

#endif
}

/* refresh area */
void epdiy_repaint(EpdRect area) {
  epdiy_take_update_lock(portMAX_DELAY);

#ifdef CONFIG_IDF_TARGET_ESP32S3
  EpdRect update_area;
  int     clear_count = 1;
  if (epdiy_prepare_update_area(area, &update_area, &clear_count,
                                "repaint area", 1)) {
    auto err = epdiy_update_prepared_area(update_area, "repaint area",
                                          clear_count);
    if (err != EPD_DRAW_SUCCESS) {
      epdiy_force_full_repaint_after_draw_error(err, "repaint area");
    }
  }
#else
  if (epdiy_auto_poweron()) {
    epd_clear_area_cycles(area, 1, _clear_cycle_time);
    epd_hl_update_area_directly(&hl, epdiy_current_update_mode(), temperature,
                                area);
  }
#endif
  if (!epdiy_is_locking_poweron()) {
    epd_poweroff();
  }
  epdiy_give_update_lock();
}
