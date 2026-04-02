#include "CF1133Touch.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
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
static uint8_t                cf1133_interrupt_trigger = 0;
static TouchInterruptHandler* _touchInterruptHandler   = nullptr;
static CF1133TPoint           _readedPoint;
static gpio_int_type_t        _cf1133_interrupt_type = GPIO_INTR_POSEDGE;

esp_err_t scanPoint(CF1133TPoint& point);

// a new task for cf1133 interrupt
TaskHandle_t          _cf1133_task_handle;
void                  _cf1133_task_cb(void* arg);
static StaticTask_t   _cf1133_task_tcb;
static StackType_t*   _cf1133_task_stack       = nullptr;
static const uint32_t _cf1133_task_stack_depth = 1024 * 4;

// set intr type
void setCF1133IntrType(gpio_int_type_t type) {
  _cf1133_interrupt_type = type;
}

gpio_int_type_t getCF1133IntrType() {
  return _cf1133_interrupt_type;
}

// touch interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg) {
  // ets_printf("touch interrupt level: %d\n",
  //            gpio_get_level((gpio_num_t)getCF1133TouchInt()));
  cf1133_interrupt_trigger = 1;

  // to read cf1133 point immediately
  xTaskResumeFromISR(_cf1133_task_handle);
}

// reset interrupt pin to avoid esp_restart failed
static void _cf1133_before_restart() {
  cf1133_interrupt_trigger = 0;
  auto int_pin             = (gpio_num_t)getCF1133TouchInt();
  gpio_intr_disable(int_pin);
  gpio_isr_handler_remove(int_pin);
  gpio_reset_pin(int_pin);
}

// custom interrupt pin
static int8_t _touchPin;
void          setCF1133TouchInt(int8_t intPin) {
  _touchPin = intPin;
}
int8_t getCF1133TouchInt() {
  return _touchPin;
}

CF1133Touch::CF1133Touch() {
  _instance = this;
}

// Destructor does nothing for now
CF1133Touch::~CF1133Touch() {}

CF1133Touch* CF1133Touch::instance() {
  return _instance;
}

bool CF1133Touch::begin(uint16_t width, uint16_t height) {
  ESP_LOGI(TAG, "I2C SDA:%d SCL:%d INT:%d", CONFIG_LV_TOUCH_I2C_SDA,
           CONFIG_LV_TOUCH_I2C_SCL, getCF1133TouchInt());

  _touch_width  = width;
  _touch_height = height;
  if (width == 0 || height == 0) {
    ESP_LOGE(
      TAG, "begin(uint8_t threshold, uint16_t width, uint16_t height) did not "
           "receive the width / height so touch cannot be rotation aware");
  }

// s3 board will init by main project
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
  io_conf.intr_type    = _cf1133_interrupt_type;
  io_conf.pin_bit_mask = 1ULL << getCF1133TouchInt();
  io_conf.mode         = GPIO_MODE_INPUT;
  if (_cf1133_interrupt_type == GPIO_INTR_NEGEDGE) {
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
  } else {
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
  }
  gpio_config(&io_conf);

  // reset interrupt pin to avoid esp_restart failed
  esp_register_shutdown_handler(_cf1133_before_restart);

  // INT gpio interrupt handler
  gpio_isr_handler_add((gpio_num_t)getCF1133TouchInt(), gpio_isr_handler, NULL);

  // a new task for cf1133 interrupt
  if (_cf1133_task_stack == nullptr) {
    _cf1133_task_stack = (StackType_t*)heap_caps_malloc(
      _cf1133_task_stack_depth * sizeof(StackType_t),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }

  if (_cf1133_task_stack == nullptr) {
    ESP_LOGE(TAG, "failed to alloc cf1133 task stack");
    return false;
  }

  _cf1133_task_handle = xTaskCreateStaticPinnedToCore(
    _cf1133_task_cb, "cf1133_task_cb", _cf1133_task_stack_depth, NULL,
    configMAX_PRIORITIES - 2, _cf1133_task_stack, &_cf1133_task_tcb, 1);
  if (_cf1133_task_handle == NULL) {
    ESP_LOGE(TAG, "xTaskCreateStaticPinnedToCore cf1133_task_cb failed");
    return false;
  }

  return true;
}

void CF1133Touch::registerTouchInterruptHandler(TouchInterruptHandler* fn) {
  _touchInterruptHandler = fn;
}

CF1133TPoint CF1133Touch::loop() {
  return processTouch();
}

CF1133TPoint CF1133Touch::processTouch() {
  CF1133TPoint point = lastTouch;

  if (_readedPoint.timestamp > 0) {
    point                  = _readedPoint;
    _readedPoint.timestamp = 0;

    int64_t timestamp = esp_timer_get_time();

    if (point.event == 0) {
      if (!point.x && !point.y) {
        // release event may trigger at (0, 0) for L58
        point.x = lastTouch.x;
        point.y = lastTouch.y;
      } else if ((lastTouch.x != point.x && lastTouch.y != point.y) ||
                 timestamp - lastTouch.timestamp > 2 * 1000 * 1000) {
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
    // ESP_LOGI(TAG, "cf1133_interrupt_trigger=0");
  }

  return point;
}

esp_err_t scanPoint(CF1133TPoint& point) {
  static uint16_t pre_index   = 0;
  auto            max_touches = 1;

  uint8_t buf[max_touches * 4 + 1];
  auto    ret =
    esp_utils::i2c_read(CF1133_ADDR, 0x11, buf, max_touches * 4 + 1, 150);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "read finger error (%d)", ret);
    return ret;
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

  return ESP_OK;
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
  ESP_LOGW(TAG, "sleep result: %d; try count: %ld", res, (long)try_count);
}

// a new task for cf1133 interrupt
bool _cf1133_task_inited       = false;
int  _cf1133_read_failed_count = 0;
void _cf1133_task_cb(void* arg) {
  while (true) {
    if (!_cf1133_task_inited) {
      _cf1133_task_inited = true;
      vTaskSuspend(_cf1133_task_handle);
    }
    esp_rom_delay_us(100);

    if (cf1133_interrupt_trigger) {
      auto err = scanPoint(_readedPoint);
      ESP_LOGI(TAG, "readedPoint, err: %d, point: %d, %d, %d", err,
               _readedPoint.x, _readedPoint.y, _readedPoint.event);
      if (err == ESP_OK) {
        cf1133_interrupt_trigger  = 0;
        _cf1133_read_failed_count = 0;
        _readedPoint.timestamp    = esp_timer_get_time();
        if (_touchInterruptHandler) {
          _touchInterruptHandler();
        }
      } else {
        _cf1133_read_failed_count++;
        if (_cf1133_read_failed_count > 10) {
          _cf1133_read_failed_count = 0;
          cf1133_interrupt_trigger  = 0;
        } else {
          // if read error, wait 100ms and try again when next loop
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
      }
    } else {
      ESP_LOGI(TAG, "no need to readPoint");
    }

    esp_rom_delay_us(100);

    if (!cf1133_interrupt_trigger) {
      ESP_LOGI(TAG, "vTaskSuspend Touch task");
      vTaskSuspend(_cf1133_task_handle);
    }
  }
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
  ESP_LOGW(TAG, "wakeup: %d; try count: %ld", res, (long)try_count);
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
