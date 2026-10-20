#ifndef PT_C_API_H
#define PT_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PtImagePt* PtImageHandle;

PtImageHandle pt_image_create(unsigned int width, unsigned int height, int channels_kind, float fill);
void pt_image_destroy(PtImageHandle img);
unsigned int pt_image_width(PtImageHandle img);
unsigned int pt_image_height(PtImageHandle img);
float* pt_image_data(PtImageHandle img);
int pt_image_channels_count(PtImageHandle img);
PtImageHandle pt_image_clone(PtImageHandle img);
int pt_image_load_png(const char* path, PtImageHandle* out);
int pt_image_save_png(const char* path, PtImageHandle img);
int pt_image_load_any(const char* path, PtImageHandle* out);
int pt_image_save_any(const char* path, PtImageHandle img);

void pt_blend(float* dst, const float* src, unsigned long long count, int mode, float opacity);
void pt_blend_masked(float* dst, const float* src, const float* mask, unsigned long long count, int mode, float opacity);

void pt_levels(void* img, float inBlack, float inWhite, float gamma, float outBlack, float outWhite);
void pt_gaussian_blur(void* img, float sigma);
void pt_box_blur_c(void* pixels, unsigned int w, unsigned int h, int radius);
void pt_sobel_c(const float* src, float* dst, unsigned int w, unsigned int h);
void pt_hsl_cycle_c(float* rgba, unsigned long long count, float hueShift);
void pt_gamma_lut_c(float* lut, unsigned int n, float gamma);
void pt_bilinear_sample_c(const float* img, unsigned int w, unsigned int h, int ch, float x, float y, float* out4);

void pt_perlin2(float* out, unsigned int w, unsigned int h, float scale, int seed);
void pt_simplex2(float* out, unsigned int w, unsigned int h, float scale, int seed);
void pt_worley2(float* out, unsigned int w, unsigned int h, float scale, int seed, int f2);

void pt_version(char* buf, int len);

#ifdef __cplusplus
}
#endif

#endif
