#include "epdiy_epaper.h"
#include "epd_driver.h"
#include "epd_highlevel.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "time.h"

EpdiyHighlevelState hl;
uint16_t            flushcalls = 0;
uint8_t*            framebuffer;
uint8_t             temperature = 25;
bool                init        = true;
// MODE_DU: Fast monochrome | MODE_GC16 slow with 16 grayscales
enum EpdDrawMode updateMode = MODE_GC16;

// batch update recorder
static uint8_t            batch_update_size   = 0;
static EpdRect*           batch_update_areas  = NULL;
static esp_timer_handle_t batch_timer_handler = NULL;
static void               batch_timer_cb(void* arg);
static void               batch_timer_start();
static void               batch_add_to_list(EpdRect* update_area);
static void               batch_flush(void* arg);

static void batch_timer_cb(void* arg) {
  batch_timer_handler = NULL;
  batch_flush(NULL);
}

static void batch_timer_start() {
  esp_timer_init();

  if (batch_timer_handler != NULL) {
    esp_timer_stop(batch_timer_handler);
    esp_timer_delete(batch_timer_handler);
    batch_timer_handler = NULL;
  }
  esp_timer_create_args_t args = {
    .callback        = batch_timer_cb,
    .arg             = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name            = "batch_timer",
  };
  esp_timer_create(&args, &batch_timer_handler);
  esp_timer_start_once(batch_timer_handler, 100e3);
}

// add to batch
static void batch_add_to_list(EpdRect* update_area) {
  size_t size_unit = sizeof(EpdRect);

  EpdRect* new_update_areas =
    heap_caps_malloc(size_unit * (batch_update_size + 1), MALLOC_CAP_SPIRAM);
  if (batch_update_size > 0) {
    memcpy(new_update_areas, batch_update_areas, batch_update_size * size_unit);
  }
  memcpy(new_update_areas + batch_update_size, update_area, size_unit);
  if (batch_update_areas) {
    heap_caps_free(batch_update_areas);
  }
  batch_update_size++;
  batch_update_areas = new_update_areas;

  batch_timer_start();
}

// batch flush
static void batch_flush(void* arg) {
  if (batch_update_size == 0) {
    return;
  }
  clock_t time_1 = clock();
  epd_poweron();
  for (int8_t ind = 0; ind < batch_update_size; ind++) {
    EpdRect* area = (batch_update_areas + ind);
    epd_hl_update_area(&hl, updateMode, temperature, *area);
  }
  clock_t time_2 = clock();

  heap_caps_free(batch_update_areas);
  batch_update_areas = NULL;

  //   ESP_LOGI("batch_flush", "batch_update_size: %d; flush time: %ld ms; ",
  //            batch_update_size, time_2 - time_1);
  batch_update_size = 0;

  epd_poweroff();
}

/* Display initialization routine */
void epdiy_init(void) {
  epd_init(EPD_OPTIONS_DEFAULT);
  hl          = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  framebuffer = epd_hl_get_framebuffer(&hl);
  epd_poweron();
  //Clear all always in init:
  epd_fullclear(&hl, temperature);
  epd_poweroff();
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
void buf_copy_to_framebuffer(EpdRect update_area, const uint8_t* buf) {
  assert(framebuffer != NULL);

  for (uint32_t i = 0; i < update_area.width * update_area.height; i++) {
    uint8_t val = buf[i];
    int     x   = update_area.x + i % update_area.width;
    int     y   = update_area.y + i / update_area.width;
    if (x < 0 || x >= EPD_WIDTH) {
      continue;
    }
    if (y < 0 || y >= EPD_HEIGHT) {
      continue;
    }
    // use epd_draw_pixel will be slow
    uint8_t* buf_ptr = &framebuffer[y * EPD_WIDTH / 2 + x / 2];
    if (x % 2) {
      *buf_ptr = (*buf_ptr & 0x0F) | (val & 0xF0);
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

  //   ESP_LOGI("epd",
  //            "flush start. count index: %d; "
  //            "x:%d y:%d w:%d h:%d; ",
  //            flushcalls, (uint16_t)area->x1, (uint16_t)area->y1, w, h);

  EpdRect update_area = {
    .x = (uint16_t)area->x1, .y = (uint16_t)area->y1, .width = w, .height = h};

  uint8_t* buf = (uint8_t*)color_map;
  // Buffer debug
  /*
    for (int index=0; index<400; index++) {
        printf("%x ", buf[index]);
    } */

  // UNCOMMENT only one of this options
  // SAFE Option with EPDiy copy of epd_copy_to_framebuffer

  clock_t time_1 = clock();

  buf_copy_to_framebuffer(update_area, buf);

  clock_t time_2 = clock();

  //Faster mode suggested in LVGL forum (Leaves ghosting&prints bad sections / experimental) NOTE: Do NOT use in production
  //buf_area_to_framebuffer(area, buf);

  batch_add_to_list(&update_area);

  clock_t time_3 = clock();

  /* Inform the graphics library that you are ready with the flushing */
  lv_disp_flush_ready(drv);
}

/*
 * Called for each pixel. Designed with the idea to fill the buffer directly, not to set each pixel, see LVGL Forum (buf_area_to_framebuffer)
*/
void epdiy_set_px_cb(lv_disp_drv_t* disp_drv,
                     uint8_t*       buf,
                     lv_coord_t     buf_w,
                     lv_coord_t     x,
                     lv_coord_t     y,
                     lv_color_t     color,
                     lv_opa_t       opa) {
  buf[y * buf_w + x] = color.full;
}
