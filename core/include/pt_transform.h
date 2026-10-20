#pragma once

#include "pt_image.h"

namespace pt {

void transform_affine(const Image& src, Image& dst, float m00, float m01, float m02, float m10, float m11, float m12);
void transform_scale(const Image& src, Image& dst, float sx, float sy);
void transform_rotate(const Image& src, Image& dst, float angleDeg);
void transform_resize(const Image& src, Image& dst, uint32_t w, uint32_t h, bool bicubic);
void transform_flip_h(const Image& src, Image& dst);
void transform_flip_v(const Image& src, Image& dst);
void transform_rotate_90cw(const Image& src, Image& dst);
void transform_rotate_90ccw(const Image& src, Image& dst);
void transform_rotate_180(const Image& src, Image& dst);
void transform_perspective(const Image& src, Image& dst, const float corners[8]);
void transform_warp_grid(const Image& src, Image& dst, int gridW, int gridH, const std::vector<float>& offsets);
void content_aware_scale(const Image& src, Image& dst, uint32_t newW, uint32_t newH, const std::vector<float>& protectMask);
void puppet_warp(const Image& src, Image& dst, const std::vector<float>& pinsSrc, const std::vector<float>& pinsDst, int count, float meshDensity);

}
