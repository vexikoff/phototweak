#include "pt_c_api.h"
#include "pt_image.h"
#include "pt_blend.h"
#include "pt_adjust.h"
#include "pt_filters.h"
#include "pt_math.h"
#include <cstring>

using namespace pt;

extern "C" {

PtImageHandle pt_image_create(unsigned int width, unsigned int height, int channels_kind, float fill) {
    Image* img = new Image(create_image(width, height, static_cast<ChannelKind>(channels_kind), fill));
    return reinterpret_cast<PtImageHandle>(img);
}

void pt_image_destroy(PtImageHandle img) { delete reinterpret_cast<Image*>(img); }

unsigned int pt_image_width(PtImageHandle img) { return reinterpret_cast<Image*>(img)->width; }
unsigned int pt_image_height(PtImageHandle img) { return reinterpret_cast<Image*>(img)->height; }
float* pt_image_data(PtImageHandle img) { return reinterpret_cast<Image*>(img)->data(); }

int pt_image_channels_count(PtImageHandle img) {
    ChannelKind ch = reinterpret_cast<Image*>(img)->channels;
    switch (ch) {
        case ChannelKind::RGB: return 4;
        case ChannelKind::CMYK: return 5;
        case ChannelKind::Lab: return 4;
        case ChannelKind::Gray: return 2;
    }
    return 4;
}

PtImageHandle pt_image_clone(PtImageHandle img) {
    return reinterpret_cast<PtImageHandle>(new Image(clone_image(*reinterpret_cast<Image*>(img))));
}

int pt_image_load_png(const char* path, PtImageHandle* out) {
    Image* img = new Image();
    if (load_png(path, *img)) { *out = reinterpret_cast<PtImageHandle>(img); return 1; }
    delete img; return 0;
}

int pt_image_save_png(const char* path, PtImageHandle img) {
    return save_png(path, *reinterpret_cast<Image*>(img)) ? 1 : 0;
}

int pt_image_load_any(const char* path, PtImageHandle* out) {
    Image* img = new Image();
    if (load_image_any(path, *img)) { *out = reinterpret_cast<PtImageHandle>(img); return 1; }
    delete img; return 0;
}

int pt_image_save_any(const char* path, PtImageHandle img) {
    return save_image_any(path, *reinterpret_cast<Image*>(img)) ? 1 : 0;
}

void pt_blend(float* dst, const float* src, unsigned long long count, int mode, float opacity) {
    blend_images(dst, src, count, static_cast<BlendMode>(mode), opacity);
}

void pt_blend_masked(float* dst, const float* src, const float* mask, unsigned long long count, int mode, float opacity) {
    blend_with_mask(dst, src, mask, count, static_cast<BlendMode>(mode), opacity);
}

void pt_levels(void* img, float inBlack, float inWhite, float gamma, float outBlack, float outWhite) {
    adjust_levels(*reinterpret_cast<Image*>(img), inBlack, inWhite, gamma, outBlack, outWhite);
}

void pt_gaussian_blur(void* img, float sigma) {
    gaussian_blur(*reinterpret_cast<Image*>(img), sigma);
}

void pt_version(char* buf, int len) {
    snprintf(buf, len, "%s", "PhotoTweak Core 1.0.0");
}

}
