#pragma once

#include <cstdint>

struct EpdiyCopyArea {
  int x;
  int y;
  int width;
  int height;
};

/**
 * 将 LVGL 的 8 位像素区域复制为 EPDIY 的 4 位打包帧缓冲。
 * 查找表给出每种输入颜色对应的 4 位值，成对写入以减少逐像素计算。
 */
inline void epdiy_copy_lvgl8_to_gray4(uint8_t*       framebuffer,
                                      int            display_width,
                                      int            display_height,
                                      EpdiyCopyArea  area,
                                      const uint8_t* source,
                                      const uint8_t* gray4_lut) {
  if (!framebuffer || !source || !gray4_lut || display_width <= 0 ||
      display_height <= 0 || area.width <= 0 || area.height <= 0) {
    return;
  }

  int clipped_x1 = area.x < 0 ? 0 : area.x;
  int clipped_y1 = area.y < 0 ? 0 : area.y;
  int clipped_x2 = area.x + area.width;
  int clipped_y2 = area.y + area.height;
  if (clipped_x2 > display_width) {
    clipped_x2 = display_width;
  }
  if (clipped_y2 > display_height) {
    clipped_y2 = display_height;
  }
  if (clipped_x1 >= clipped_x2 || clipped_y1 >= clipped_y2) {
    return;
  }

  // 裁剪只改变有效复制范围，源数据仍按原始区域宽度逐行寻址。
  int source_x = clipped_x1 - area.x;
  int source_y = clipped_y1 - area.y;
  int copy_width = clipped_x2 - clipped_x1;
  // EPDIY 布局要求显示宽度为偶数，每字节低四位存偶数列、高四位存奇数列。
  int framebuffer_stride = display_width / 2;

  for (int y = clipped_y1; y < clipped_y2; ++y) {
    const uint8_t* src =
      source + (source_y + y - clipped_y1) * area.width + source_x;
    uint8_t* dst = framebuffer + y * framebuffer_stride + clipped_x1 / 2;
    int remaining = copy_width;

    // 起点在奇数列时保留同字节的左邻像素，随后再成对写入。
    if (clipped_x1 & 1) {
      *dst = (*dst & 0x0F) | (gray4_lut[*src] << 4);
      ++src;
      ++dst;
      --remaining;
    }

    while (remaining >= 2) {
      *dst++ = gray4_lut[src[0]] | (gray4_lut[src[1]] << 4);
      src += 2;
      remaining -= 2;
    }

    // 末尾只剩一个像素时保留右邻像素，防止局部刷新破坏区域外内容。
    if (remaining) {
      *dst = (*dst & 0xF0) | gray4_lut[*src];
    }
  }
}
