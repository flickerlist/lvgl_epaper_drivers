// This is the first experimental Touch component for the LILYGO EPD47 touch overlay
// Controller: L58  -> https://github.com/Xinyuan-LilyGO/LilyGo-EPD47/files/6059098/L58.V1.0.pdf
// Note: Rotation is only working for certain angles (0 works alright, 2 also) Others still need to be corrected
#include "L58Touch.h"
#include "esp_utils.h"

#define CONFIG_L58_DEBUG 0

L58Touch*          L58Touch::_instance = nullptr;
static const char* TAG                 = "L58Touch";

#ifdef CONFIG_IDF_TARGET_ESP32S3
  // 1000ms will call epdiy crash on s3 board (pca9555_set_value)
  #define TOUCH_I2C_TIMEOUT 100
#else
  #define TOUCH_I2C_TIMEOUT 1000
#endif

/**
 * Record the interrupt of intPin.
 * When interrupt triggered, set to '1', after used, set to '0'.
 *
 * Default '1' to force read data at the first time.
 */
static uint8_t                interrupt_trigger      = 1;
static TouchInterruptHandler* _touchInterruptHandler = nullptr;

// touch interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg) {
  if (interrupt_trigger == 0) {
    interrupt_trigger = 1;
  }
  if (_touchInterruptHandler) {
    _touchInterruptHandler();
  }
}

L58Touch::L58Touch(int8_t intPin) {
  _instance = this;
  _intPin   = intPin;
}

// Destructor does nothing for now
L58Touch::~L58Touch() {}

L58Touch* L58Touch::instance() {
  return _instance;
}

bool L58Touch::begin(uint16_t width, uint16_t height) {
  ESP_LOGI(TAG, "I2C SDA:%d SCL:%d INT:%d; SOC_I2C_NUM: %d",
           CONFIG_LV_TOUCH_I2C_SDA, CONFIG_LV_TOUCH_I2C_SCL, _intPin,
           SOC_I2C_NUM);

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
  esp_err_t i2c_driver =
    i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
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
  io_conf.pull_down_en = (gpio_pulldown_t)0;  // disable pull-down mode
  io_conf.pull_up_en   = (gpio_pullup_t)1;  // pull-up mode
  gpio_config(&io_conf);

  // INT gpio interrupt handler
  gpio_isr_handler_add((gpio_num_t)CONFIG_LV_TOUCH_INT, gpio_isr_handler, NULL);

  return true;
}

void L58Touch::registerTouchInterruptHandler(TouchInterruptHandler* fn) {
  _touchInterruptHandler = fn;
}

TPoint L58Touch::loop() {
  _point = processTouch();
  return _point;
}

TPoint L58Touch::processTouch() {
  TPoint point;
  point.x     = lastX;
  point.y     = lastY;
  point.event = lastEvent;

  if (interrupt_trigger == 1) {
    interrupt_trigger = 0;
    point             = scanPoint();
    lastEvent         = point.event;

    /**
     * lastEvent=0 means released from TP
     *
     * L68 default return (0,0) when released, but this will case lvgl error, such as dropdown cannot get currect value: https://forum.lvgl.io/t/item-in-the-dropdown-list-is-not-selected/5381
     *
     */
    if (lastEvent == 0 && !point.x && !point.y) {
      point.x = lastX;
      point.y = lastY;
    } else {
      lastX = point.x;
      lastY = point.y;
    }
  }

  return point;
}

TPoint L58Touch::scanPoint() {
  TPoint  point{0, 0, 0};
  uint8_t buf[7] = {0};

  buf[0] = 0xD0;
  buf[1] = 0x00;
  esp_utils::i2c_write_and_read(L58_ADDR, buf[0], buf + 1, 1, buf, 7,
                                TOUCH_I2C_TIMEOUT);
  uint16_t x     = (uint16_t)((buf[1] << 4) | ((buf[3] >> 4) & 0x0F));
  uint16_t y     = (uint16_t)((buf[2] << 4) | (buf[3] & 0x0F));
  uint8_t  event = (buf[0] & 0x0F) >> 1;
  switch (_rotation) {
    // 0- no rotation: Works OK inverting Y axis
    case 0:
      break;

    case 1:
      swap(x, y);
      x = _touch_height - x;
      break;

    case 2:
      x = _touch_width - x;
      break;

    case 3:
      swap(x, y);
      break;
  }

  if (_touch_width != L58_TOUCH_FIRMWARE_WIDTH ||
      _touch_height != L58_TOUCH_FIRMWARE_HEIGHT) {
    x = ((uint32_t)x) * ((uint32_t)_touch_width) / L58_TOUCH_FIRMWARE_WIDTH;
    y = ((uint32_t)y) * ((uint32_t)_touch_height) / L58_TOUCH_FIRMWARE_HEIGHT;
  }

  point = {x, y, event};
  return point;
}

void L58Touch::fireEvent(TPoint point, TEvent e) {
  if (_touchHandler)
    _touchHandler(point, e);
}

void L58Touch::setRotation(uint8_t rotation) {
  _rotation = rotation;
}

void L58Touch::setTouchWidth(uint16_t width) {
  ESP_LOGI(TAG, "touch width: %d", width);
  _touch_width = width;
}

void L58Touch::setTouchHeight(uint16_t height) {
  ESP_LOGI(TAG, "touch height: %d", height);
  _touch_height = height;
}

void L58Touch::clearFlags() {
  uint8_t reg    = 0xD0;
  uint8_t buf[2] = {0X00, 0XAB};
  esp_utils::i2c_write(L58_ADDR, reg, buf, sizeof(buf), TOUCH_I2C_TIMEOUT);
}

void L58Touch::sleep(int32_t try_count) {
  uint8_t reg    = 0xD1;
  uint8_t buf[1] = {0X05};

  esp_err_t res;
  while (true) {
    res =
      esp_utils::i2c_write(L58_ADDR, reg, buf, sizeof(buf), TOUCH_I2C_TIMEOUT);
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
