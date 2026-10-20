#pragma once

#include "pt_image.h"
#include <vector>
#include <string>

namespace pt {

void mask_rect(std::vector<float>& mask, uint32_t w, uint32_t h, int x0, int y0, int x1, int y1);
void mask_ellipse(std::vector<float>& mask, uint32_t w, uint32_t h, float cx, float cy, float rx, float ry);
void mask_polygon(std::vector<float>& mask, uint32_t w, uint32_t h, const std::vector<float>& pointsXY, bool invert);
void magic_wand(const Image& img, const std::vector<float>* curMask, std::vector<float>& outMask,
               int x, int y, float tolerance, bool contiguous, bool antiAlias, float featherPx, bool addToSelection);
void quick_selection_grow(const Image& img, std::vector<float>& mask, int cx, int cy, int radius, float tolerance, uint32_t steps);
void feather_mask(std::vector<float>& mask, uint32_t w, uint32_t h, float radius);
void smooth_mask(std::vector<float>& mask, uint32_t w, uint32_t h, float radius, float edgeThin);
void contract_mask(std::vector<float>& mask, uint32_t w, uint32_t h, int px);
void expand_mask(std::vector<float>& mask, uint32_t w, uint32_t h, int px);
void border_mask(std::vector<float>& mask, uint32_t w, uint32_t h, int thickness);
void invert_mask(std::vector<float>& mask);
void refine_edge_std(Image& img, std::vector<float>& mask, float radius, float smooth, float feather, float contrast, float shiftEdge);
std::vector<uint8_t> mask_to_png_bytes(const std::vector<float>& mask, uint32_t w, uint32_t h);
bool png_bytes_to_mask(const uint8_t* bytes, size_t len, std::vector<float>& mask, uint32_t& w, uint32_t& h);
void apply_mask_to_layer(Image& layer, const std::vector<float>& mask);

}
