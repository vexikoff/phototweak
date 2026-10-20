#pragma once

#include "pt_image.h"

namespace pt {

void adjust_levels(Image& img, float inBlack, float inWhite, float gamma, float outBlack, float outWhite);
void adjust_curves(Image& img, const std::vector<float>& lutR, const std::vector<float>& lutG, const std::vector<float>& lutB);
void adjust_exposure(Image& img, float exposureEV, float offset, float gamma);
void adjust_brightness_contrast(Image& img, float brightness, float contrast);
void adjust_hue_saturation(Image& img, float hueShift, float saturation, float lightness, int targetChannel);
void adjust_vibrance(Image& img, float vibrance, float saturation);
void adjust_color_balance(Image& img, float cyanRedSh, float cyanRedM, float cyanRedH, float magentaGreenSh, float magentaGreenM, float magentaGreenH, float yellowBlueSh, float yellowBlueM, float yellowBlueH);
void adjust_black_white(Image& img, float r, float y, float g, float c, float b, float m);
void adjust_photo_filter(Image& img, int filterIndex, float density);
void adjust_channel_mixer(Image& img, bool monochrome, float rR, float gR, float bR, float rG, float gG, float bG, float rB, float gB, float bB, float constants);
void adjust_posterize(Image& img, int levels);
void adjust_threshold(Image& img, float level);
void adjust_invert(Image& img);
void adjust_gradient_map(Image& img, const std::vector<float>& gradientRGBA, int stops);
void adjust_selective_color(Image& img, int correctionRange, float c, float m, float y, float k);
void adjust_shadow_highlight(Image& img, float shadowAmount, float shadowWidth, float highlightAmount, float highlightWidth, float radius);
void auto_contrast(Image& img);
void auto_levels(Image& img);
void auto_tone(Image& img);
void equalize(Image& img);
void desaturate(Image& img);

}
