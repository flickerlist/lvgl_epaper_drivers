#pragma once

#include <algorithm>

static const int EPDIY_GC16_STABLE_LCD_PCLK_MHZ = 10;
// 黑白刷新以稳定供数为优先，将默认像素时钟上限设为 20 MHz，减少供数不足的风险。
static const int EPDIY_MONO_STABLE_LCD_PCLK_MHZ = 20;

static inline int epdiy_lcd_pclk_for_stable_mono(int default_pclk_mhz) {
  if (default_pclk_mhz <= 0) {
    return default_pclk_mhz;
  }

  return std::min(default_pclk_mhz, EPDIY_MONO_STABLE_LCD_PCLK_MHZ);
}

static inline int epdiy_lcd_pclk_for_refresh_mode(int  default_pclk_mhz,
                                                  bool gc16_enabled) {
  if (default_pclk_mhz <= 0 || !gc16_enabled) {
    return default_pclk_mhz;
  }

  return std::min(default_pclk_mhz, EPDIY_GC16_STABLE_LCD_PCLK_MHZ);
}
