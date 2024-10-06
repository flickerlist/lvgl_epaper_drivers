#include "CF1133Touch.h"
#include "esp_utils.h"
#include "lvgl.h"

CF1133Touch*       CF1133Touch::_instance = nullptr;
static const char* TAG                    = "CF1133Touch";

/**
 * Record the interrupt of intPin.
 * When interrupt triggered, set to '1', after used, set to '0'.
 *
 * Default '1' to force read data at the first time.
 */
static uint8_t                interrupt_trigger      = 0;
static TouchInterruptHandler* _touchInterruptHandler = nullptr;

// touch interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg) {
  //   ets_printf("touch interrupt level: %d\n",
  //              gpio_get_level((gpio_num_t)CONFIG_LV_TOUCH_INT));
  interrupt_trigger = 1;
  if (_touchInterruptHandler) {
    _touchInterruptHandler();
  }
}

CF1133Touch::CF1133Touch(int8_t intPin) {
  _instance = this;
  _intPin   = intPin;
}

// Destructor does nothing for now
CF1133Touch::~CF1133Touch() {}

CF1133Touch* CF1133Touch::instance() {
  return _instance;
}

bool CF1133Touch::begin(uint16_t width, uint16_t height) {
  ESP_LOGI(TAG, "I2C SDA:%d SCL:%d INT:%d", CONFIG_LV_TOUCH_I2C_SDA,
           CONFIG_LV_TOUCH_I2C_SCL, _intPin);

  _touch_width  = width;
  _touch_height = height;
  if (width == 0 || height == 0) {
    ESP_LOGE(
      TAG, "begin(uint8_t threshold, uint16_t width, uint16_t height) did not "
           "receive the width / height so touch cannot be rotation aware");
  }

// s3 board will init by epdiy
#ifndef CONFIG_IDF_TARGET_ESP32S3
  i2c_config_t conf;
  conf.mode             = I2C_MODE_MASTER;
  conf.sda_io_num       = (gpio_num_t)CONFIG_LV_TOUCH_I2C_SDA;
  conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
  conf.scl_io_num       = (gpio_num_t)CONFIG_LV_TOUCH_I2C_SCL;
  conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 50000;

  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
  // !< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here.
  conf.clk_flags = 0;
  #endif

  i2c_param_config(I2C_NUM_0, &conf);
  esp_err_t i2c_driver = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
  if (i2c_driver == ESP_OK) {
    ESP_LOGI(TAG, "i2c_driver started correctly");
  } else {
    ESP_LOGI(TAG, "i2c_driver error: %d", i2c_driver);
  }
#endif

  // INT pin triggers the callback function on the Falling edge of the GPIO
  gpio_config_t io_conf;
  io_conf.intr_type    = GPIO_INTR_POSEDGE;
  io_conf.pin_bit_mask = 1ULL << CONFIG_LV_TOUCH_INT;
  io_conf.mode         = GPIO_MODE_INPUT;
  // must set enable for inited on 0 level
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  // pull-up mode for touch interrupt
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  // INT gpio interrupt handler
  gpio_isr_handler_add((gpio_num_t)CONFIG_LV_TOUCH_INT, gpio_isr_handler, NULL);

  return true;
}

void CF1133Touch::registerTouchInterruptHandler(TouchInterruptHandler* fn) {
  _touchInterruptHandler = fn;
}

CF1133TPoint CF1133Touch::loop() {
  return processTouch();
}

CF1133TPoint CF1133Touch::processTouch() {
  CF1133TPoint point;
  point.x     = lastTouch.x;
  point.y     = lastTouch.y;
  point.event = lastTouch.event;

  if (interrupt_trigger == 1) {
    interrupt_trigger = 0;
    point             = scanPoint();

    clock_t timestamp = clock();

    if (point.event == 0) {
      if (!point.x && !point.y) {
        // release event may trigger at (0, 0) for L58
        point.x = lastTouch.x;
        point.y = lastTouch.y;
      } else if ((lastTouch.x != point.x && lastTouch.y != point.y) ||
                 timestamp - lastTouch.timestamp > 2000) {
        // repair fast click: can only read release event, so give it a pressed event
        point.event = 1;
        ESP_LOGI(TAG, "processTouch x: %d, y: %d, repaire event to [1]",
                 point.x, point.y);

        // to read again for released event
        lv_async_call([](void* arg) { gpio_isr_handler(NULL); }, nullptr);
      }
    }
    ESP_LOGI(TAG, "processTouch x: %d, y: %d, event: %d", point.x, point.y,
             point.event);

    lastTouch.x         = point.x;
    lastTouch.y         = point.y;
    lastTouch.event     = point.event;
    lastTouch.timestamp = timestamp;
  } else {
    // ESP_LOGI(TAG, "interrupt_trigger=0");
  }

  return point;
}

CF1133TPoint CF1133Touch::scanPoint() {
  CF1133TPoint point{0, 0, 0, 0};

  static uint16_t pre_index   = 0;
  auto            max_touches = 1;

  uint8_t buf[max_touches * 4 + 1];
  auto    ret =
    esp_utils::i2c_read(CF1133_ADDR, 0x11, buf, max_touches * 4 + 1, 150);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "read finger error (%d)", ret);
    return point;
  }

  auto i  = 0;  // touch index
  point.x = (int)(buf[1 + i * 4] & 0x70) << 4 | buf[1 + i * 4 + 1];
  point.y = (int)(buf[1 + i * 4] & 0x0F) << 8 | buf[1 + i * 4 + 2];
  if (buf[1 + 4 * i] & 0x80) {
    pre_index |= 0x01 << i;
    point.event = 1;
  } else if (pre_index & (0x01 << i)) {
    pre_index &= ~(0x01 << i);
    point.event = 0;
  }

  return point;
}

void CF1133Touch::setRotation(uint8_t rotation) {
  _rotation = rotation;
}

void CF1133Touch::setTouchWidth(uint16_t width) {
  ESP_LOGI(TAG, "touch width: %d", width);
  _touch_width = width;
}

void CF1133Touch::setTouchHeight(uint16_t height) {
  ESP_LOGI(TAG, "touch height: %d", height);
  _touch_height = height;
}

void CF1133Touch::sleep(int32_t try_count) {
  uint8_t reg    = 0x02;
  uint8_t buf[1] = {0x02};

  esp_err_t res;
  while (true) {
    res = esp_utils::i2c_write(CF1133_ADDR, reg, buf, sizeof(buf));
    if (res == ESP_OK) {
      break;
    }
    try_count--;
    if (try_count == 0) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  ESP_LOGW(TAG, "sleep result: %d; try count: %d", res, try_count);
}

void CF1133Touch::wakeup(int32_t try_count) {
  uint8_t reg    = 0x02;
  uint8_t buf[1] = {0x01};

  esp_err_t res;
  while (true) {
    res = esp_utils::i2c_write(CF1133_ADDR, reg, buf, sizeof(buf));
    if (res == ESP_OK) {
      break;
    }
    try_count--;
    if (try_count == 0) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  ESP_LOGW(TAG, "wakeup: %d; try count: %d", res, try_count);
}

uint8_t CF1133Touch::readStatus() {
  uint8_t reg = 0x01;
  uint8_t buf[1];
  auto    ret = esp_utils::i2c_read(CF1133_ADDR, reg, buf, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "readStatus error (%d)", ret);
    return 0;
  }
  return buf[0];
}
