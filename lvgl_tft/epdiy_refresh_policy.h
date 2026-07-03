#pragma once

#include <algorithm>

static const int EPDIY_GC16_STABLE_LCD_PCLK_MHZ = 10;

static inline int epdiy_lcd_pclk_for_refresh_mode(int  default_pclk_mhz,
                                                  bool gc16_enabled) {
  if (default_pclk_mhz <= 0 || !gc16_enabled) {
    return default_pclk_mhz;
  }

  return std::min(default_pclk_mhz, EPDIY_GC16_STABLE_LCD_PCLK_MHZ);
}
