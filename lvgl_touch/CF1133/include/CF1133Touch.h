// This is the first experimental Touch component for the LILYGO EPD47 touch overlay
// NOTE: As in LVGL we cannot use blocking functions this is a variation of original library here:
//       https://github.com/martinberlin/FT6X36-IDF
// More info about this epaper:
// https://github.com/martinberlin/cale-idf/wiki/Model-parallel-ED047TC1.h
#include <stdint.h>
//#include <cstdlib>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>

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
  // TwoWire * wire will be replaced by ESP-IDF https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
  CF1133Touch(int8_t intPin);
  ~CF1133Touch();

  static CF1133Touch* instance();

  // handler when the intPin interrupted, can only do very little, and `can't call log`.
  static void registerTouchInterruptHandler(TouchInterruptHandler* fn);

  bool         begin(uint16_t width = 0, uint16_t height = 0);
  CF1133TPoint loop();
  CF1133TPoint processTouch();
  // Helper functions to make the touch display aware
  void setRotation(uint8_t rotation);
  void setTouchWidth(uint16_t width);
  void setTouchHeight(uint16_t height);
  // Pending implementation. How much x->touch y↓touch is placed (In case is smaller than display)
  void sleep(int32_t try_count = 10);
  void wakeup(int32_t try_count = 10);
  // Smart template from EPD to swap x,y:
  template <typename T> static inline void swap(T& a, T& b) {
    T t = a;
    a   = b;
    b   = t;
  }

  TouchData_t data[5];
  // Tap detection is enabled by default
  bool tapDetectionEnabled = true;
  // Only if the time difference between press and release is minor than this milliseconds a Tap even is triggered
  uint16_t tapDetectionMillisDiff = 100;

 private:
  CF1133TPoint scanPoint();

  static CF1133Touch* _instance;
  uint8_t             _intPin;

  // Make touch rotation aware:
  uint8_t  _rotation     = 0;
  uint16_t _touch_width  = 0;
  uint16_t _touch_height = 0;

  uint8_t       _touches;
  uint16_t      _touchX[2], _touchY[2], _touchEvent[2];
  CF1133TPoint  _points[10];
  CF1133TPoint  _point;
  uint8_t       _pointIdx       = 0;
  unsigned long _touchStartTime = 0;
  unsigned long _touchEndTime   = 0;
  uint8_t       lastEvent       = 0;
  uint16_t      lastX           = 0;
  uint16_t      lastY           = 0;
  bool          _dragMode       = false;
  const uint8_t maxDeviation    = 5;
};

#endif
