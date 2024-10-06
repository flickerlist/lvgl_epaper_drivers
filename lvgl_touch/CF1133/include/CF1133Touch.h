#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#ifndef touch_cf1133_h
  #define touch_cf1133_h

  #define CF1133_ADDR 0x55

  // The size write in firmware of CF1133
  #define CF1133_TOUCH_FIRMWARE_WIDTH 1024
  #define CF1133_TOUCH_FIRMWARE_HEIGHT 758

struct CF1133TPoint {
  uint16_t x;
  uint16_t y;
  uint8_t  event;
  clock_t  timestamp;
};

// handler when the intPin interrupted
typedef void TouchInterruptHandler();

class CF1133Touch {
  typedef struct {
    uint8_t  id;
    uint8_t  event;
    uint16_t x;
    uint16_t y;
  } TouchData_t;

 public:
  CF1133Touch(int8_t intPin);
  ~CF1133Touch();

  static CF1133Touch* instance();
  bool                begin(uint16_t width = 0, uint16_t height = 0);

  CF1133TPoint loop();
  CF1133TPoint processTouch();

  // Pending implementation. How much x->touch y↓touch is placed (In case is smaller than display)
  void    sleep(int32_t try_count = 10);
  void    wakeup(int32_t try_count = 10);
  uint8_t readStatus();

  // handler when the intPin interrupted, can only do very little, and `can't call log`.
  static void registerTouchInterruptHandler(TouchInterruptHandler* fn);

  // Helper functions to make the touch display aware
  void setRotation(uint8_t rotation);
  void setTouchWidth(uint16_t width);
  void setTouchHeight(uint16_t height);

 private:
  CF1133TPoint scanPoint();

  static CF1133Touch* _instance;
  uint8_t             _intPin;

  // Make touch rotation aware:
  uint8_t  _rotation     = 0;
  uint16_t _touch_width  = 0;
  uint16_t _touch_height = 0;

  CF1133TPoint lastTouch;
};

#endif
