// This is the first experimental Touch component for the LILYGO EPD47 touch overlay
// Controller: L58  -> https://github.com/Xinyuan-LilyGO/LilyGo-EPD47/files/6059098/L58.V1.0.pdf
// Note: Rotation is only working for certain angles (0 works alright, 2 also) Others still need to be corrected
#include "L58Touch.h"

#define CONFIG_L58_DEBUG 0

L58Touch*          L58Touch::_instance = nullptr;
static const char* TAG                 = "L58Touch";

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
  printf("I2C sda:%d scl:%d int:%d\n\n", CONFIG_LV_TOUCH_I2C_SDA,
         CONFIG_LV_TOUCH_I2C_SCL, intPin);
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
    printf("i2c_driver started correctly\n");
  } else {
    printf("i2c_driver error: %d\n", i2c_driver);
  }
  _intPin = intPin;
}

// Destructor does nothing for now
L58Touch::~L58Touch() {}

L58Touch* L58Touch::instance() {
  return _instance;
}

bool L58Touch::begin(uint16_t width, uint16_t height) {
  _touch_width  = width;
  _touch_height = height;
  if (width == 0 || height == 0) {
    ESP_LOGE(
      TAG, "begin(uint8_t threshold, uint16_t width, uint16_t height) did not "
           "receive the width / height so touch cannot be rotation aware");
  }

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

void L58Touch::registerTouchHandler(void (*fn)(TPoint point, TEvent e)) {
  _touchHandler = fn;
  if (CONFIG_L58_DEBUG)
    printf("Touch handler function registered\n");
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

    point     = scanPoint();
    lastEvent = point.event;

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

uint8_t L58Touch::read8(uint8_t regName) {
  uint8_t buf;
  readRegister8(regName, &buf);
  return buf;
}

TPoint L58Touch::scanPoint() {
  TPoint  point{0, 0, 0};
  uint8_t buf[7] = {0};

  buf[0] = 0xD0;
  buf[1] = 0x00;
  readBytes(buf, 7);
  uint16_t x     = (uint16_t)((buf[1] << 4) | ((buf[3] >> 4) & 0x0F));
  uint16_t y     = (uint16_t)((buf[2] << 4) | (buf[3] & 0x0F));
  uint8_t  event = (buf[0] & 0x0F) >> 1;
  // TODO: rotation need test for values: 1,2,3
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

  if (_touch_width != L58_TOUCH_FIRMWARE_WIDTH || _touch_height != L58_TOUCH_FIRMWARE_HEIGHT) {
    x = ((uint32_t)x) * ((uint32_t)_touch_width) / L58_TOUCH_FIRMWARE_WIDTH;
    y = ((uint32_t)y) * ((uint32_t)_touch_height) / L58_TOUCH_FIRMWARE_HEIGHT;
  }

  point = {x, y, event};
  return point;
}

void L58Touch::writeRegister8(uint8_t reg, uint8_t value) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, L58_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
}

esp_err_t L58Touch::writeData(uint8_t* data, int len) {
  if (len == 0)
    return ESP_FAIL;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, L58_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write(cmd, data, len, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  auto res = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return res;
}

uint8_t L58Touch::readRegister8(uint8_t reg, uint8_t* data_buf) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, L58_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
  // Research: Why it's started a 2nd time here
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (L58_ADDR << 1) | I2C_MASTER_READ, true);

  i2c_master_read_byte(cmd, data_buf, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

#if defined(CONFIG_L58_DEBUG) && CONFIG_L58_DEBUG == 1
  printf("REG 0x%x: 0x%x\n", reg, ret);
#endif

  return ret;
}

esp_err_t L58Touch::readBytes(uint8_t* data, int len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, L58_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write(cmd, data, 2, ACK_CHECK_EN);

  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, L58_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read(cmd, data, len, (i2c_ack_type_t)ACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK) {
    // ESP_LOGW(TAG, "Read OK");
    for (int i = 0; i < len; i++) {
      printf("0x%02x ", data[i]);
      if ((i + 1) % 16 == 0) {
        printf("\r\n");
      }
    }
    if (len % 16) {
      printf("\r\n");
    }
  } else if (ret == ESP_ERR_TIMEOUT) {
    // Getting a lot of this!, but data may usable even `ESP_ERR_TIMEOUT`
    // ESP_LOGW(TAG, "Bus is busy");
  } else {
    ESP_LOGW(TAG, "Read failed: %d", (int)ret);
  }
  return ret;
}

void L58Touch::fireEvent(TPoint point, TEvent e) {
  if (_touchHandler)
    _touchHandler(point, e);
}

void L58Touch::setRotation(uint8_t rotation) {
  _rotation = rotation;
}

void L58Touch::setTouchWidth(uint16_t width) {
  printf("touch w:%d\n", width);
  _touch_width = width;
}

void L58Touch::setTouchHeight(uint16_t height) {
  printf("touch h:%d\n", height);
  _touch_height = height;
}

void L58Touch::clearFlags() {
  uint8_t buf[3] = {0xD0, 0X00, 0XAB};
  writeData(buf, sizeof(buf));
}

void L58Touch::sleep(int32_t try_count) {
  uint8_t buf[2] = {0xD1, 0X05};

  esp_err_t res;
  while (true) {
    res = writeData(buf, sizeof(buf));
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
