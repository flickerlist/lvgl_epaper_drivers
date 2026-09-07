#include "epdiy_epaper.h"
#include "epdiy_async_flush_state.h"
#include "epdiy_framebuffer_copy.h"
#include "epd_highlevel.h"
#include "epdiy_refresh_policy.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>
#include <cstring>
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
uint8_t             temperature             = 25;
const int           _clear_cycle_time       = 12;
static int          s_lcd_pclk_mhz          = 20;
static int          s_lcd_pclk_default_mhz  = 20;
static const int    EPDIY_LCD_PCLK_MIN_MHZ  = EPDIY_GC16_STABLE_LCD_PCLK_MHZ;
static const int    EPDIY_LCD_PCLK_STEP_MHZ = 2;
static std::atomic<bool> s_16_grayscale_enabled{EPDIY_ENABLE_16_GRAYSCALE};
// 预先映射全部 8 位颜色，复制像素时直接查表，避免每帧重复计算黑白或灰阶值。
static uint8_t      s_mono_gray4_lut[256];
static uint8_t      s_gc16_gray4_lut[256];
static bool         s_gray4_luts_initialized = false;

epdiy_flush_type_cb_t _epdiy_flush_type_cb;
TaskHandle_t          _paint_task_handle;
void buf_copy_to_framebuffer(EpdRect image_area, const lv_color_t* image_data);
void paint_task_cb(void* arg);
enum EpdDrawError epdiy_repaint_full_screen(bool need_power = true);
static bool epdiy_handle_draw_error(enum EpdDrawError err, const char* stage);
static void epdiy_force_full_repaint_after_draw_error(enum EpdDrawError err,
                                                      const char*       stage);
static enum EpdDrawMode epdiy_current_update_mode();
static uint8_t          epdiy_color_to_gray4(lv_color_t color);
static void             epdiy_initialize_gray4_luts();
static void
epdiy_mark_pending_update(EpdRect area, const char* stage, int clear_count);
static void epdiy_pending_update_retry_task(void* arg);
static void epdiy_schedule_pending_update_retry();
static bool epdiy_take_update_lock(TickType_t timeout_ticks);
static void epdiy_give_update_lock();
static void epdiy_clear_to_white_locked(EpdRect area, int clear_count, int clear_cycle_time);

// 所有共享显示状态的访问都走同一入口，递归调用也不能反向等待帧缓冲。
class epdiy_update_guard {
 public:
  epdiy_update_guard() : locked_(epdiy_take_update_lock(portMAX_DELAY)) {
    assert(locked_);
  }
  ~epdiy_update_guard() {
    if (locked_) epdiy_give_update_lock();
  }
  epdiy_update_guard(const epdiy_update_guard&) = delete;
  epdiy_update_guard& operator=(const epdiy_update_guard&) = delete;

 private:
  bool locked_;
};

typedef struct _paint_t {
  lvgl_epdiy_flush_type_t paint_type;
  EpdRect                 area;
  lv_color_t*             color_map;
  lv_disp_drv_t*          drv;
  bool                    is_last;
} paint_t;

vector<paint_t>          paint_queue;
static SemaphoreHandle_t paint_queue_xMutex  = NULL;  // lock for paint_queue
static SemaphoreHandle_t epdiy_update_xMutex = NULL;
bool                     whole_repainting    = false;  // Whole repaint task

static const int               EPDIY_PENDING_RETRY_DELAY_MS = 500;
static const int               EPDIY_PENDING_RETRY_MAX       = 3;
static bool                    s_pending_update_valid       = false;
static EpdRect                 s_pending_update_area        = {0, 0, 0, 0};
static bool                    s_pending_update_use_gc16    = false;
static int                     s_pending_update_clear_count = 0;
static int                     s_pending_retry_count        = 0;
static bool                    s_pending_retry_task_running = false;
static const enum EpdDrawError EPDIY_DRAW_POWER_ON_FAILED =
  (enum EpdDrawError)0x800;

#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
typedef struct {
  uint64_t             job_id;
  EpdRect              area;
  bool                 repaint_all;
  bool                 repaint_all_after;
  epdiy_async_flush_completion_t completion;
} epdiy_async_flush_job_t;

static QueueHandle_t s_async_job_queue                = NULL;
static QueueHandle_t s_async_completion_queue         = NULL;
static SemaphoreHandle_t s_async_framebuffer_available = NULL;
static TaskHandle_t s_async_epd_task                  = NULL;
static bool s_async_resources_ready                   = false;
static bool s_async_frame_owns_framebuffer            = false;
static bool s_async_menu_feedback                    = false;
static uint64_t s_async_monitor_job_id                = 0;
static uint64_t s_async_next_job_id                   = 1;
static portMUX_TYPE s_async_state_lock = portMUX_INITIALIZER_UNLOCKED;
static epdiy_async_flush_state s_async_flush_state;
// owner 仅表示 GUI 分块复制期间的持有者；入队后清空，但信号量仍由物理任务持有。
static TaskHandle_t s_async_framebuffer_owner = NULL;
// 下面两项只在更新互斥锁内访问，用于最外层调用释放其自行取得的帧缓冲。
static unsigned s_update_lock_depth = 0;
static bool s_update_lock_owns_framebuffer = false;

static void epdiy_set_async_framebuffer_owner(TaskHandle_t owner) {
  portENTER_CRITICAL(&s_async_state_lock);
  s_async_framebuffer_owner = owner;
  portEXIT_CRITICAL(&s_async_state_lock);
}

static void epdiy_async_worker_task(void* arg);
static bool epdiy_init_async_worker();
#endif

#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t epdiy_pm_lock;
#endif

// 异步启用后所有帧均交给工作任务；此接口只标记下一帧为菜单反馈，避免其放行页面内容。
bool epdiy_request_next_flush_async(void) {
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  if (!s_async_resources_ready) {
    return false;
  }
  bool requested = false;
  portENTER_CRITICAL(&s_async_state_lock);
  requested = s_async_flush_state.request_next_frame();
  if (requested) {
    s_async_menu_feedback = true;
  }
  portEXIT_CRITICAL(&s_async_state_lock);
  if (!requested) {
    ESP_LOGW(TAG, "event=async_flush_request_rejected");
  }
  return requested;
#else
  return false;
#endif
}

bool epdiy_take_async_monitor_job(uint64_t* job_id) {
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  if (!job_id) {
    return false;
  }
  bool found = false;
  portENTER_CRITICAL(&s_async_state_lock);
  if (s_async_monitor_job_id != 0) {
    *job_id = s_async_monitor_job_id;
    s_async_monitor_job_id = 0;
    found = true;
  }
  portEXIT_CRITICAL(&s_async_state_lock);
  return found;
#else
  (void)job_id;
  return false;
#endif
}

bool epdiy_take_async_flush_completion(
  epdiy_async_flush_completion_t* completion) {
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  if (!completion || !s_async_completion_queue) {
    return false;
  }
  return xQueueReceive(s_async_completion_queue, completion, 0) == pdTRUE;
#else
  (void)completion;
  return false;
#endif
}

/* Display initialization routine */
void epdiy_init(void) {
  epdiy_update_xMutex = xSemaphoreCreateRecursiveMutex();

#ifdef USE_PARALLEL_PAINT
  paint_queue_xMutex = xSemaphoreCreateMutex();
#endif

  hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  epd_set_rotation(EPD_ROT_LANDSCAPE);
  framebuffer            = epd_hl_get_framebuffer(&hl);
  s_lcd_pclk_default_mhz =
    epdiy_lcd_pclk_for_stable_mono(epd_get_display()->bus_speed);
  s_lcd_pclk_mhz         = s_lcd_pclk_default_mhz;
  epd_set_lcd_pixel_clock_MHz(s_lcd_pclk_mhz);
  epdiy_set_16_grayscale_enabled(false);
  epdiy_initialize_gray4_luts();

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

#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  s_async_resources_ready = epdiy_init_async_worker();
  ESP_LOGI(TAG, "EPD async flush %s",
           s_async_resources_ready ? "enabled" : "disabled");
#endif
}

#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
static bool epdiy_init_async_worker() {
  // 只有一份帧缓冲，因此同时只允许一个物理刷新任务；信号量保护像素所有权。
  s_async_job_queue = xQueueCreate(1, sizeof(epdiy_async_flush_job_t));
  s_async_completion_queue =
    xQueueCreate(4, sizeof(epdiy_async_flush_completion_t));
  s_async_framebuffer_available = xSemaphoreCreateBinary();
  if (!s_async_job_queue || !s_async_completion_queue ||
      !s_async_framebuffer_available) {
    ESP_LOGE(TAG, "failed to allocate async flush resources");
    if (s_async_job_queue) {
      vQueueDelete(s_async_job_queue);
      s_async_job_queue = NULL;
    }
    if (s_async_completion_queue) {
      vQueueDelete(s_async_completion_queue);
      s_async_completion_queue = NULL;
    }
    if (s_async_framebuffer_available) {
      vSemaphoreDelete(s_async_framebuffer_available);
      s_async_framebuffer_available = NULL;
    }
    return false;
  }

  xSemaphoreGive(s_async_framebuffer_available);
  BaseType_t created = xTaskCreatePinnedToCore(
    &epdiy_async_worker_task, "epd_async", 1024 * 6, NULL, 5,
    &s_async_epd_task, 1);
  if (created != pdPASS) {
    ESP_LOGE(TAG, "failed to create async flush worker");
    vQueueDelete(s_async_job_queue);
    vQueueDelete(s_async_completion_queue);
    vSemaphoreDelete(s_async_framebuffer_available);
    s_async_job_queue = NULL;
    s_async_completion_queue = NULL;
    s_async_framebuffer_available = NULL;
    s_async_epd_task = NULL;
    return false;
  }
  return true;
}
#endif

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

static void epdiy_set_lcd_pclk_if_changed(int target_mhz, const char* reason) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (target_mhz <= 0 || target_mhz == s_lcd_pclk_mhz) {
    return;
  }

  s_lcd_pclk_mhz = target_mhz;
  epd_set_lcd_pixel_clock_MHz(s_lcd_pclk_mhz);
#else
  (void)target_mhz;
  (void)reason;
#endif
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
                                                      const char*       stage) {
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

static void epdiy_initialize_gray4_luts() {
#if LV_COLOR_DEPTH == 8
  if (s_gray4_luts_initialized) {
    return;
  }
  static_assert(sizeof(lv_color_t) == 1,
                "LVGL 8-bit color must occupy exactly one byte");
  for (int i = 0; i < 256; ++i) {
    lv_color_t color = {};
    color.full = (uint8_t)i;
    s_mono_gray4_lut[i] = lv_color_to1(color) ? 0x0F : 0x00;
    // 与逐像素路径使用相同的取整规则，保持查表优化前后的灰阶输出一致。
    s_gc16_gray4_lut[i] = (lv_color_brightness(color) + 8) / 17;
  }
  s_gray4_luts_initialized = true;
#endif
}

static uint8_t epdiy_gray4_to_mono(uint8_t gray) {
  return (gray & 0x0F) >= 8 ? 0x0F : 0x00;
}

static void epdiy_normalize_framebuffer_to_mono() {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (!framebuffer) {
    return;
  }

  int    width      = epd_rotated_display_width();
  int    height     = epd_rotated_display_height();
  size_t line_bytes = width / 2;
  size_t size       = line_bytes * height;

  for (size_t i = 0; i < size; i++) {
    uint8_t value  = framebuffer[i];
    framebuffer[i] = epdiy_gray4_to_mono(value & 0x0F) |
                     (epdiy_gray4_to_mono(value >> 4) << 4);
  }
#endif
}

static bool epdiy_take_update_lock(TickType_t timeout_ticks) {
  if (!epdiy_update_xMutex) {
    return true;
  }
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  TaskHandle_t current = xTaskGetCurrentTaskHandle();
  bool recursive = xSemaphoreGetMutexHolder(epdiy_update_xMutex) == current;
  bool acquired_framebuffer = false;
  if (!recursive && s_async_resources_ready && current != s_async_epd_task) {
    portENTER_CRITICAL(&s_async_state_lock);
    bool owns_frame = s_async_framebuffer_owner == current;
    portEXIT_CRITICAL(&s_async_state_lock);
    if (!owns_frame) {
      // 必须先等帧缓冲、再拿更新锁。否则外部调用会占锁等待工作任务，造成互相等待。
      // 已入队但尚未开始的帧同样持有信号量，外部操作不能提前改模式、像素或电源。
      TickType_t started = xTaskGetTickCount();
      if (xSemaphoreTake(s_async_framebuffer_available, timeout_ticks) != pdTRUE) {
        return false;
      }
      acquired_framebuffer = true;
      if (timeout_ticks != portMAX_DELAY) {
        TickType_t elapsed = xTaskGetTickCount() - started;
        timeout_ticks = elapsed < timeout_ticks ? timeout_ticks - elapsed : 0;
      }
    }
  }
#endif
  if (xSemaphoreTakeRecursive(epdiy_update_xMutex, timeout_ticks) != pdTRUE) {
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
    if (acquired_framebuffer) xSemaphoreGive(s_async_framebuffer_available);
#endif
    return false;
  }
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  if (!recursive) s_update_lock_owns_framebuffer = acquired_framebuffer;
  ++s_update_lock_depth;
#endif
  return true;
}

static void epdiy_give_update_lock() {
  if (!epdiy_update_xMutex) return;
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  assert(s_update_lock_depth > 0);
  bool release_framebuffer = --s_update_lock_depth == 0 && s_update_lock_owns_framebuffer;
  if (release_framebuffer) s_update_lock_owns_framebuffer = false;
#endif
  xSemaphoreGiveRecursive(epdiy_update_xMutex);
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  if (release_framebuffer) xSemaphoreGive(s_async_framebuffer_available);
#endif
}

static bool epdiy_area_is_valid(EpdRect area) {
  return area.width > 0 && area.height > 0;
}

static EpdRect epdiy_merge_area(EpdRect first, EpdRect second) {
  int x1        = first.x < second.x ? first.x : second.x;
  int y1        = first.y < second.y ? first.y : second.y;
  int x2_first  = first.x + first.width;
  int y2_first  = first.y + first.height;
  int x2_second = second.x + second.width;
  int y2_second = second.y + second.height;
  int x2        = x2_first > x2_second ? x2_first : x2_second;
  int y2        = y2_first > y2_second ? y2_first : y2_second;

  EpdRect merged = {.x = x1, .y = y1, .width = x2 - x1, .height = y2 - y1};
  return merged;
}

static void
epdiy_log_area(const char* prefix, const char* stage, EpdRect area) {
  ESP_LOGW(TAG, "%s %s area x=%d y=%d w=%d h=%d", stage ? stage : "update",
           prefix, (int)area.x, (int)area.y, (int)area.width, (int)area.height);
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

static void
epdiy_mark_pending_update(EpdRect area, const char* stage, int clear_count) {
  if (!epdiy_area_is_valid(area)) {
    return;
  }

  bool was_pending = s_pending_update_valid;
  bool use_gc16    = epdiy_is_16_grayscale_enabled();
  clear_count      = epdiy_pending_clear_count_with(clear_count);
  if (s_pending_update_valid) {
    area     = epdiy_merge_area(area, s_pending_update_area);
    use_gc16 = use_gc16 || s_pending_update_use_gc16;
  }

  s_pending_update_area        = area;
  s_pending_update_use_gc16    = use_gc16;
  s_pending_update_clear_count = clear_count;
  s_pending_update_valid       = true;
  if (!was_pending) {
    s_pending_retry_count = 0;
  }
  if (clear_count > 0) {
    epdiy_log_area(use_gc16 ? "pending clear GC16" : "pending clear DU", stage,
                   area);
  } else {
    epdiy_log_area(use_gc16 ? "pending GC16" : "pending DU", stage, area);
  }
  epdiy_schedule_pending_update_retry();
}

static void epdiy_clear_pending_update() {
  s_pending_update_valid       = false;
  s_pending_update_use_gc16    = false;
  s_pending_update_clear_count = 0;
  s_pending_retry_count        = 0;
}

static bool epdiy_prepare_update_area(EpdRect     requested_area,
                                      EpdRect*    update_area,
                                      int*        clear_count,
                                      const char* stage,
                                      int         requested_clear_count) {
  EpdRect merged_area = epdiy_area_with_pending(requested_area, stage);
  int     effective_clear_count =
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

static enum EpdDrawError
epdiy_update_prepared_area(EpdRect area, const char* stage, int clear_count) {
  enum EpdDrawMode mode =
    (s_pending_update_valid && s_pending_update_use_gc16) ?
      MODE_GL16 :
      epdiy_current_update_mode();
  if (clear_count > 0) {
    epdiy_clear_to_white_locked(area, clear_count, _clear_cycle_time);
  }
  enum EpdDrawError err = epd_hl_update_area(&hl, mode, temperature, area);
  if (err == EPD_DRAW_SUCCESS) {
    epdiy_clear_pending_update();
  } else {
    epdiy_mark_pending_update(area, stage, clear_count > 0 ? clear_count : 1);
  }
  return err;
}

static void epdiy_schedule_pending_update_retry() {
  if (s_pending_retry_task_running || !s_pending_update_valid ||
      s_pending_retry_count >= EPDIY_PENDING_RETRY_MAX) {
    return;
  }

  s_pending_retry_task_running = true;
  BaseType_t ret = xTaskCreate(&epdiy_pending_update_retry_task,
                               "epdiy_pending_retry", 1024 * 4, NULL, 4, NULL);
  if (ret != pdPASS) {
    s_pending_retry_task_running = false;
    ESP_LOGW(TAG, "create pending update retry task failed");
  }
}

static void epdiy_pending_update_retry_task(void* arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(EPDIY_PENDING_RETRY_DELAY_MS));

  // 重试任务保持登记状态直到拿到锁，不能在等待超时后无锁改写共享重试标志。
  epdiy_take_update_lock(portMAX_DELAY);

  s_pending_retry_task_running = false;
  if (!s_pending_update_valid) {
    epdiy_give_update_lock();
    vTaskDelete(NULL);
    return;
  }

  EpdRect area = s_pending_update_area;
  s_pending_retry_count++;
  ESP_LOGW(TAG, "pending update retry %d/%d", s_pending_retry_count,
           EPDIY_PENDING_RETRY_MAX);

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif

  EpdRect update_area;
  int     clear_count = 0;
  bool did_poweron = epdiy_prepare_update_area(area, &update_area, &clear_count,
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

  // 是否继续重试及任务登记必须与更新共享状态处于同一个锁内。
  if (s_pending_update_valid) {
    epdiy_schedule_pending_update_retry();
  }
  epdiy_give_update_lock();

  vTaskDelete(NULL);
}

/* A copy from epd_copy_to_framebuffer with temporary lenght prediction */
void buf_copy_to_framebuffer(EpdRect image_area, const lv_color_t* image_data) {
  assert(framebuffer != NULL);

  auto display_width  = epd_rotated_display_width();
  auto display_height = epd_rotated_display_height();
#if LV_COLOR_DEPTH == 8
  epdiy_initialize_gray4_luts();
  const uint8_t* gray4_lut = epdiy_is_16_grayscale_enabled() ?
                               s_gc16_gray4_lut :
                               s_mono_gray4_lut;
  EpdiyCopyArea copy_area = {
    .x = image_area.x,
    .y = image_area.y,
    .width = image_area.width,
    .height = image_area.height,
  };
  epdiy_copy_lvgl8_to_gray4(framebuffer, display_width, display_height,
                            copy_area,
                            reinterpret_cast<const uint8_t*>(image_data),
                            gray4_lut);
#else
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
#endif
}

// 同步回退和异步任务共用物理刷新流程，统一保留电源管理与绘制失败恢复。
static int32_t epdiy_execute_physical_update(
  EpdRect update_area,
  bool repaint_all,
  bool repaint_all_after) {
  int32_t draw_error = EPD_DRAW_SUCCESS;
#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_acquire(epdiy_pm_lock));
#endif

  if (repaint_all) {
    epdiy_repaint(update_area);
  } else {
    int clear_count = 0;
    bool update_ok = false;
    bool did_poweron = epdiy_prepare_update_area(
      update_area, &update_area, &clear_count, "partial update", 0);
    if (did_poweron) {
      auto err = epdiy_update_prepared_area(update_area, "partial update",
                                            clear_count);
      draw_error = (int32_t)err;
      update_ok = err == EPD_DRAW_SUCCESS;
      if (!update_ok) {
        epdiy_force_full_repaint_after_draw_error(err, "partial update");
      }
    } else {
      draw_error = (int32_t)EPDIY_DRAW_POWER_ON_FAILED;
    }

    if (update_ok && repaint_all_after) {
      auto err = epdiy_repaint_full_screen(false);
      if (err != EPD_DRAW_SUCCESS) {
        draw_error = (int32_t)err;
      }
    }

    if (!epdiy_is_locking_poweron()) {
      epd_poweroff();
    }
  }

#if CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_release(epdiy_pm_lock));
#endif
  return draw_error;
}

#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
static void epdiy_async_worker_task(void* arg) {
  (void)arg;
  epdiy_async_flush_job_t job = {};
  while (true) {
    if (xQueueReceive(s_async_job_queue, &job, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    epdiy_take_update_lock(portMAX_DELAY);
    job.completion.draw_error = epdiy_execute_physical_update(
      job.area, job.repaint_all, job.repaint_all_after);
    epdiy_give_update_lock();
    // 记录物理完成时间供骨架帧排序使用，不采集阶段耗时。
    job.completion.completed_at_us = esp_timer_get_time();

    // 先发布完成事件，再释放帧缓冲；GUI 关联下一帧时才能先消费上一帧的完成通知。
    if (xQueueSend(s_async_completion_queue, &job.completion,
                   pdMS_TO_TICKS(1000)) != pdTRUE) {
      ESP_LOGE(TAG, "async completion queue full, job=%llu",
               (unsigned long long)job.job_id);
    }

    portENTER_CRITICAL(&s_async_state_lock);
    bool completed = s_async_flush_state.complete_job(job.job_id);
    portEXIT_CRITICAL(&s_async_state_lock);
    if (!completed) {
      ESP_LOGE(TAG, "async state completion mismatch, job=%llu",
               (unsigned long long)job.job_id);
    }
    xSemaphoreGive(s_async_framebuffer_available);
  }
}
#endif

/* Required by LVGL. Sends the color_map to the screen with a partial update  */
void epdiy_flush(lv_disp_drv_t*   drv,
                 const lv_area_t* area,
                 lv_color_t*      color_map) {
  bool    is_last          = lv_disp_flush_is_last(drv);
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  bool frame_async = false;
  portENTER_CRITICAL(&s_async_state_lock);
  frame_async =
    s_async_resources_ready && s_async_flush_state.begin_flush(is_last);
  portEXIT_CRITICAL(&s_async_state_lock);
#endif
  ++flushcalls;
  uint16_t w = lv_area_get_width(area);
  uint16_t h = lv_area_get_height(area);

  EpdRect update_area = {
    .x = (uint16_t)area->x1, .y = (uint16_t)area->y1, .width = w, .height = h};

  lvgl_epdiy_flush_type_t _paint_type =
    _epdiy_flush_type_cb ? _epdiy_flush_type_cb(&update_area, flushcalls) :
                           EPDIY_PARTIAL_PAINT;
  if (_paint_type == EPDIY_NO_PAINT) {
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
    if (frame_async && is_last) {
      portENTER_CRITICAL(&s_async_state_lock);
      s_async_flush_state.cancel_submission();
      s_async_menu_feedback = false;
      portEXIT_CRITICAL(&s_async_state_lock);
      if (s_async_frame_owns_framebuffer) {
        s_async_frame_owns_framebuffer = false;
        epdiy_set_async_framebuffer_owner(NULL);
        xSemaphoreGive(s_async_framebuffer_available);
      }
    }
#endif
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

  #ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  bool owns_framebuffer_slot = false;
  if (s_async_resources_ready) {
    if (!frame_async || !s_async_frame_owns_framebuffer) {
      // 等上一帧物理刷新结束后才能覆盖像素；同一帧的多个分块持续持有该信号量。
      xSemaphoreTake(s_async_framebuffer_available, portMAX_DELAY);
      epdiy_set_async_framebuffer_owner(xTaskGetCurrentTaskHandle());
      if (frame_async) {
        s_async_frame_owns_framebuffer = true;
      }
    }
    owns_framebuffer_slot = true;
  }
  #endif

  epdiy_take_update_lock(portMAX_DELAY);

  buf_copy_to_framebuffer(update_area, color_map);

  // 合并本帧各个 LVGL 分块的区域和刷新类型，仅在最后一个分块提交物理任务。
  static int  x1 = 65535, y1 = 65535, x2 = -1, y2 = -1;
  static bool has_paint_all       = false;
  static bool has_paint_all_after = false;
  // capture the upper left and lower right corners
  if (area->x1 < x1)
    x1 = area->x1;
  if (area->y1 < y1)
    y1 = area->y1;
  if (area->x2 > x2)
    x2 = area->x2;
  if (area->y2 > y2)
    y2 = area->y2;

  if (_paint_type == EPDIY_REPAINT_ALL) {
    has_paint_all = true;
  } else if (_paint_type == EPDIY_REPAINT_ALL_AFTER) {
    has_paint_all_after = true;
  }

  if (is_last) {
    // reset area
    update_area.x      = x1;
    update_area.y      = y1;
    update_area.width  = (x2 - x1) + 1;
    update_area.height = (y2 - y1) + 1;

  #ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
    if (frame_async && s_async_resources_ready) {
      epdiy_async_flush_job_t job = {};
      job.job_id                  = s_async_next_job_id++;
      if (job.job_id == 0) {
        job.job_id = s_async_next_job_id++;
      }
      job.area              = update_area;
      job.repaint_all       = has_paint_all;
      job.repaint_all_after = has_paint_all_after;
      job.completion.job_id = job.job_id;
      portENTER_CRITICAL(&s_async_state_lock);
      job.completion.menu_feedback = s_async_menu_feedback;
      s_async_menu_feedback        = false;
      portEXIT_CRITICAL(&s_async_state_lock);

      if (xQueueSend(s_async_job_queue, &job, 0) == pdTRUE) {
        portENTER_CRITICAL(&s_async_state_lock);
        bool submitted         = s_async_flush_state.submit_job(job.job_id);
        s_async_monitor_job_id = job.job_id;
        portEXIT_CRITICAL(&s_async_state_lock);

        // 入队成功后帧缓冲已交给工作任务；即使状态登记异常，也不能再同步刷同一帧，
        // 否则两个执行路径会并发读取和更新同一份帧缓冲。
        if (!submitted) {
          ESP_LOGE(TAG, "async state submit failed, job=%llu",
                   (unsigned long long)job.job_id);
        }
        s_async_frame_owns_framebuffer = false;
        epdiy_set_async_framebuffer_owner(NULL);

        x1 = y1 = 65535;
        x2 = y2             = -1;
        has_paint_all       = false;
        has_paint_all_after = false;
        epdiy_give_update_lock();
        lv_disp_flush_ready(drv);

        return;
      } else {
        portENTER_CRITICAL(&s_async_state_lock);
        s_async_flush_state.cancel_submission();
        s_async_menu_feedback = false;
        portEXIT_CRITICAL(&s_async_state_lock);
        // 入队失败时帧缓冲仍由当前调用持有，随后沿同步路径完成本帧。
        ESP_LOGE(TAG, "event=async_flush_fallback reason=queue_full");
      }
    }
  #endif

    lv_disp_flush_ready(drv);
    epdiy_execute_physical_update(update_area, has_paint_all,
                                  has_paint_all_after);

    // reset update boundary
    x1 = y1 = 65535;
    x2 = y2             = -1;
    has_paint_all       = false;
    has_paint_all_after = false;
    epdiy_give_update_lock();
  #ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
    if (owns_framebuffer_slot) {
      s_async_frame_owns_framebuffer = false;
      epdiy_set_async_framebuffer_owner(NULL);
      xSemaphoreGive(s_async_framebuffer_available);
    }
  #endif
  } else {
    lv_disp_flush_ready(drv);
    epdiy_give_update_lock();
  #ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
    if (owns_framebuffer_slot && !frame_async) {
      epdiy_set_async_framebuffer_owner(NULL);
      xSemaphoreGive(s_async_framebuffer_available);
    }
  #endif
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
  epdiy_update_guard guard;
  bool was_enabled       = s_16_grayscale_enabled;
  int target_pclk =
    epdiy_lcd_pclk_for_refresh_mode(s_lcd_pclk_default_mhz, enabled);
  epdiy_set_lcd_pclk_if_changed(target_pclk,
                                enabled ? "enable GC16" : "disable GC16");
  if (was_enabled && !enabled) {
    epdiy_normalize_framebuffer_to_mono();
  }
  // 改完时钟和像素后才发布新模式；只读查询无需等待工作任务，保持菜单构建的并行性。
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
      static bool has_paint_all_after = false;
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
      } else if (_paint_type == EPDIY_REPAINT_ALL_AFTER) {
        has_paint_all_after = true;
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
          bool did_poweron = epdiy_prepare_update_area(
            area, &area, &clear_count, "partial update", 0);
          if (did_poweron) {
            auto err =
              epdiy_update_prepared_area(area, "partial update", clear_count);
            update_ok = err == EPD_DRAW_SUCCESS;
            if (err != EPD_DRAW_SUCCESS) {
              epdiy_force_full_repaint_after_draw_error(err, "partial update");
            }
          }

          if (update_ok && has_paint_all_after) {
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
        has_paint_all_after = false;
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
static std::atomic<bool> _is_locking_poweron{false};
bool epdiy_auto_poweron() {
  epdiy_update_guard guard;
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
  epdiy_update_guard guard;
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
  epdiy_update_guard guard;
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
  // 非阻塞检查覆盖已入队、分块复制和正在刷新的帧，不能只看旧的刷屏任务。
  if (!epdiy_take_update_lock(0)) return false;
  bool can_pause = !s_pending_retry_task_running;
#ifdef CONFIG_FL_V4_MENU_FEEDBACK_ASYNC_FLUSH
  portENTER_CRITICAL(&s_async_state_lock);
  can_pause = can_pause && !s_async_flush_state.job_in_flight() &&
              s_async_monitor_job_id == 0;
  portEXIT_CRITICAL(&s_async_state_lock);
  // 物理刷新完成后 GUI 还要消费事件，才能放行骨架后的内容创建。
  can_pause = can_pause && (!s_async_completion_queue ||
                            uxQueueMessagesWaiting(s_async_completion_queue) == 0);
#endif
  epdiy_give_update_lock();
  if (!can_pause) return false;
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
  epdiy_update_guard guard;
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

// 内部调用已持有更新锁且已上电，不能在刷新流程中间提前断电。
static void epdiy_clear_to_white_locked(EpdRect area, int clear_count, int clear_cycle_time) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  epdiy_set_white(area);
  epd_clear_area_cycles(area, clear_count, clear_cycle_time);
#endif
}

// 对外清屏将上电、清屏、断电作为一个受保护的操作，调用方无需直接操作电源。
void epdiy_clear_to_white(EpdRect area, int clear_count, int clear_cycle_time) {
  epdiy_update_guard guard;
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (epdiy_auto_poweron()) {
    epdiy_clear_to_white_locked(area, clear_count, clear_cycle_time);
  }
  if (!epdiy_is_locking_poweron()) epd_poweroff();
#endif
}

void epdiy_set_framebuffer_gray4_pixel(int x, int y, uint8_t gray) {
  epdiy_update_guard guard;
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

static bool epdiy_framebuffer_area_is_valid(EpdRect area) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  int display_width  = epd_rotated_display_width();
  int display_height = epd_rotated_display_height();
  // 4-bit framebuffer packs two pixels per byte. Snapshot/restore only handles
  // byte-aligned rectangles so copying can stay row-based and cheap.
  return framebuffer && area.width > 0 && area.height > 0 && area.x >= 0 &&
         area.y >= 0 && area.x + area.width <= display_width &&
         area.y + area.height <= display_height && area.x % 2 == 0 &&
         area.width % 2 == 0;
#else
  return false;
#endif
}

size_t epdiy_framebuffer_area_snapshot_size(EpdRect area) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (!epdiy_framebuffer_area_is_valid(area)) {
    return 0;
  }
  return (size_t)(area.width / 2) * (size_t)area.height;
#else
  return 0;
#endif
}

static bool epdiy_copy_framebuffer_area(EpdRect  area,
                                        uint8_t* buffer,
                                        size_t   buffer_size,
                                        bool     restore) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  size_t required = epdiy_framebuffer_area_snapshot_size(area);
  if (required == 0 || !buffer || buffer_size < required) {
    return false;
  }

  size_t   row_bytes         = (size_t)area.width / 2;
  size_t   framebuffer_pitch = (size_t)epd_rotated_display_width() / 2;
  uint8_t* framebuffer_row =
    framebuffer + (size_t)area.y * framebuffer_pitch + (size_t)area.x / 2;

  // The screen pitch can be wider than the copied rect, so copy row by row.
  for (int row = 0; row < area.height; row++) {
    uint8_t* framebuffer_line =
      framebuffer_row + (size_t)row * framebuffer_pitch;
    uint8_t* buffer_line = buffer + (size_t)row * row_bytes;
    if (restore) {
      memcpy(framebuffer_line, buffer_line, row_bytes);
    } else {
      memcpy(buffer_line, framebuffer_line, row_bytes);
    }
  }
  return true;
#else
  return false;
#endif
}

bool epdiy_snapshot_framebuffer_area(EpdRect  area,
                                     uint8_t* buffer,
                                     size_t   buffer_size) {
  epdiy_update_guard guard;
#ifdef CONFIG_IDF_TARGET_ESP32S3
  return epdiy_copy_framebuffer_area(area, buffer, buffer_size, false);
#else
  return false;
#endif
}

bool epdiy_restore_framebuffer_area(EpdRect        area,
                                    const uint8_t* buffer,
                                    size_t         buffer_size,
                                    bool           repaint) {
  // 恢复像素和可选重绘持有同一个锁，其他刷新不能插入两者之间。
  epdiy_update_guard guard;
#ifdef CONFIG_IDF_TARGET_ESP32S3
  bool restored = epdiy_copy_framebuffer_area(
    area, const_cast<uint8_t*>(buffer), buffer_size, true);
  if (!restored) {
    return false;
  }

  if (!repaint) {
    return true;
  }
  return epdiy_update_framebuffer_area(area) == EPD_DRAW_SUCCESS;
#else
  return false;
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
  bool did_poweron = epdiy_prepare_update_area(area, &update_area, &clear_count,
                                               "framebuffer update", 0);
  if (did_poweron) {
    err = epdiy_update_prepared_area(update_area, "framebuffer update",
                                     clear_count);
    if (err != EPD_DRAW_SUCCESS) {
      epdiy_force_full_repaint_after_draw_error(err, "framebuffer update");
    }
  } else {
    err = EPDIY_DRAW_POWER_ON_FAILED;
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

  enum EpdDrawError err  = EPD_DRAW_SUCCESS;
  EpdRect           area = epd_full_screen();

  epdiy_take_update_lock(portMAX_DELAY);

  bool can_update  = true;
  int  clear_count = 1;
  if (need_power) {
    can_update =
      epdiy_prepare_update_area(area, &area, &clear_count, "full repaint", 1);
    if (!can_update) {
      err = EPDIY_DRAW_POWER_ON_FAILED;
    }
  } else {
    area        = epdiy_area_with_pending(area, "full repaint");
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
    auto err =
      epdiy_update_prepared_area(update_area, "repaint area", clear_count);
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
