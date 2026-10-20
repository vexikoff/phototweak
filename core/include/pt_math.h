#pragma once

#include "pt_image.h"

namespace pt {

enum class NoiseType : int { Perlin = 0, Simplex, Worley };

float perlin2(float x, float y, int seed);
float perlin3(float x, float y, float z, int seed);
float simplex2(float x, float y, int seed);
float simplex3(float x, float y, float z, int seed);
float worley2(float x, float y, int seed, bool f2);
float worley3(float x, float y, float z, int seed, bool f2);

float fbm_noise(NoiseType type, float x, float y, int octaves, float lacunarity, float gain, int seed);

void generate_procedural_texture(Image& out, uint32_t w, uint32_t h, int noiseType,
                                 float scale, int octaves, float lacunarity, float gain,
                                 float turbulenceAmount, int seed,
                                 const Color& colorLow, const Color& colorHigh);

Color rgb_to_hsl(float r, float g, float b);
void hsl_to_rgb(const Color& hsl, float& r, float& g, float& b);
Color rgb_to_hsv(float r, float g, float b);
void hsv_to_rgb(const Color& hsv, float& r, float& g, float& b);
void rgb_to_lab(float r, float g, float b, float& L, float& a, float& bb);
void lab_to_rgb(float L, float a, float bb, float& r, float& g, float& b);
void rgb_to_cmyk(float r, float g, float b, float& c, float& m, float& y, float& k);
void cmyk_to_rgb(float c, float m, float y, float k, float& r, float& g, float& b);

uint32_t hash_u32(uint32_t x);
float hash_float_2d(int x, int y, uint32_t seed);

}
