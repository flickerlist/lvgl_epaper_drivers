#ifndef CF1133_H
#define CF1133_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
  #include "lvgl.h"
#else
  #include "lvgl/lvgl.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Initialize for FT6x36 communication via I2C
  * @retval None
  */
void cf1133_init();

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool cf1133_read(lv_indev_drv_t* drv, lv_indev_data_t* data);

#ifdef __cplusplus
}
#endif
#endif /* __FT6X06_H */
