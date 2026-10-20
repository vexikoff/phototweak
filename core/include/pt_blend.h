#pragma once

#include "pt_image.h"

namespace pt {

struct Color {
    float r = 0.0f, g = 0.0f, b = 0.0f, a = 1.0f;
};

void blend_images(float* dst, const float* src, size_t count, BlendMode mode, float opacity);
void blend_with_mask(float* dst, const float* src, const float* mask, size_t count, BlendMode mode, float opacity);
void apply_opacity(float* rgba, size_t count, float opacity);
void fill_rect(float* rgba, uint32_t imgW, uint32_t imgH, int x, int y, int w, int h, const Color& c);

}
