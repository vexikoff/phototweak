#pragma once

#include "pt_image.h"

namespace pt {

void box_blur(Image& img, int radius);
void gaussian_blur(Image& img, float sigma);
void motion_blur(Image& img, int radius, float angleDeg);
void radial_blur_spin(Image& img, float angleDeg, int samples);
void radial_blur_zoom(Image& img, float amount, int samples);
void iris_blur(Image& img, float cx, float cy, float innerRadius, float outerRadius, float maxBlur);
void field_blur(Image& img, const std::vector<float>& pointsXYR, int pointCount);
void tilt_shift_blur(Image& img, float y0, float y1, float blurMax);
void surface_blur(Image& img, int radius, float threshold);
void shape_blur(Image& img, int radius);
void smart_blur(Image& img, float radius, float threshold);
void lens_blur(Image& img, int radius);
void median_filter(Image& img, int radius);
void max_filter(Image& img, int radius);
void min_filter(Image& img, int radius);
void despeckle(Image& img, int radius, float threshold);
void dust_scratches(Image& img, int radius, float threshold);
void add_noise(Image& img, float amount, bool uniform, bool monochromatic, float seed);
void reduce_noise(Image& img, float strength);
void sharpen(Image& img, float amount, float radius, float threshold);
void unsharp_mask(Image& img, float amount, float radius, float threshold);
void smart_sharpen(Image& img, float amount, float radius, float threshold);
void edge_detect(Image& img, int mode);
void emboss(Image& img, float angleDeg, float distance, float strength);
void outline(Image& img);
void find_edges(Image& img);
void solarize(Image& img);
void wind(Image& img, int direction, float strength);
void mosaic(Image& img, int cellSize);
void pointillize(Image& img, int cellSize);
void color_halftone(Image& img, int cellSize);
void mezzotint(Image& img, int type, float noise);
void crystallize(Image& img, int cellSize);
void color_burn(Image& img, int radius);
void diffused_dither(Image& img);
void tint_flat(Image& img, int size, bool dark, float intensity);
void clouds(Image& img, float seed);
void difference_clouds(Image& img, float seed);
void fibers(Image& img, int variance, float strength, float seed);
void lens_flare(Image& img, float cx, float cy, float brightness, float zoom, float glare, int ringCount, float seed);
void lighting_effect(Image& img, float altitudeDeg, float azimuthDeg, float intensity, float ambient, float material, int style, int textureType, float offsetX, float offsetY);
void displacement_map(Image& img, const Image& mapX, const Image& mapY, float scaleX, float scaleY, int undefinedAction);
void twirl_distort(Image& img, float angleDeg, float strength);
void wave_distort(Image& img, float wavelength, float amplitude, float speedDeg, float phaseDeg, float amplitude2, float wavelength2, float pinch, float skew);
void zigzag_distort(Image& img, float corners, float radius, float amount);
void spherize_distort(Image& img, float amount, float mode);
void polar_to_rect(Image& img, float rotation, bool invert);
void grid_distort(Image& img, float majorX, float majorY, float minorX, float minorY, float radius, bool invert);
void shear_distort(Image& img, float angleDeg);
void ripple_distort(Image& img, float amount, float size);
void ocean_ripple(Image& img, float amount, float size, float seed);
void glass_effect(Image& img, float distortion, float smoothness, bool frosty, float scaling, float seed);
void pinch_distort(Image& img, float amount);
void bulge_distort(Image& img, float amount);
void stretch_distort(Image& img, float quality, float amount);
void squeeze_twirl_recto(Image& img, float angleDeg, float innerBox, float outerBox, float twistDeg, float amount, float radius);
void perspective_crop_warp(Image& img, const float srcQuad[8], const float dstQuad[8]);
void liquify_forward_warp(Image& img, const std::vector<float>& pointsXYZRStrength, int count);

}
