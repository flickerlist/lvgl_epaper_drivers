/*
* Copyright © 2024 lanistor@163.com
*/

#include <driver/i2c.h>
#include <esp_log.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
  #include <lvgl.h>
#else
  #include <lvgl/lvgl.h>
#endif
#include "CF1133.h"
// Cale touch implementation
#include "CF1133Touch.h"

#define TAG "CF1133"

CF1133Touch touch(CONFIG_LV_TOUCH_INT);

#ifdef CONFIG_IDF_TARGET_ESP32S3
  #include <epdiy.h>
#else
  #include "epd_driver.h"
#endif

CF1133TPoint point;

/**
  * @brief  Initialize for CF1133 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C slave address of CF1133).
  * @retval None
  */
void cf1133_init() {
  int width  = epd_rotated_display_width();
  int height = epd_rotated_display_height();
  ESP_LOGI(TAG, "cf1133_init() Touch initialized: width: %d; height: %d.",
           width, height);
  touch.setRotation(3);
  touch.begin(width, height);
}

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool cf1133_read(lv_indev_drv_t* drv, lv_indev_data_t* data) {
  point         = touch.loop();
  data->point.x = point.x;
  data->point.y = point.y;
  data->state   = (lv_indev_state_t)point.event;
  return false;
}
