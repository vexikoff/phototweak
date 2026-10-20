#include "pt_adjust.h"
#include "pt_math.h"
#include "pt_filters.h"
#include <cmath>
#include <algorithm>

namespace pt {

static inline float cl(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

void adjust_levels(Image& img, float inBlack, float inWhite, float gamma, float outBlack, float outWhite) {
    if (inWhite <= inBlack) inWhite = inBlack + 0.001f;
    if (gamma <= 0.0f) gamma = 1.0f;
    float invG = 1.0f / gamma;
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        for (int c = 0; c < 3; ++c) {
            float v = cl((p[c] - inBlack) / (inWhite - inBlack));
            v = std::pow(v, invG);
            p[c] = cl(outBlack + v * (outWhite - outBlack));
        }
    }
}

static void lutApply(std::vector<float>& lut) {
    if (lut.size() != 256) return;
    for (int i = 0; i < 256; ++i) lut[i] = cl(lut[i]);
    for (int pass = 0; pass < 3; ++pass) {
        std::vector<float> tmp = lut;
        for (int i = 1; i < 255; ++i) lut[i] = (tmp[i - 1] + 2.0f * tmp[i] + tmp[i + 1]) * 0.25f;
    }
}

void adjust_curves(Image& img, const std::vector<float>& lutR, const std::vector<float>& lutG, const std::vector<float>& lutB) {
    std::vector<float> r = lutR, g = lutG, b = lutB;
    lutApply(r); lutApply(g); lutApply(b);
    auto sample = [](const std::vector<float>& lut, float v) -> float {
        float f = cl(v) * 255.0f;
        int i0 = static_cast<int>(f);
        int i1 = std::min(255, i0 + 1);
        float t = f - i0;
        return lut[i0] * (1.0f - t) + lut[i1] * t;
    };
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        p[0] = sample(r, p[0]); p[1] = sample(g, p[1]); p[2] = sample(b, p[2]);
    }
}

void adjust_exposure(Image& img, float exposureEV, float offset, float gamma) {
    float mul = std::pow(2.0f, exposureEV);
    if (gamma <= 0.0f) gamma = 1.0f;
    float invG = 1.0f / gamma;
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        for (int c = 0; c < 3; ++c) {
            float v = p[c] * mul + offset;
            v = std::pow(cl(v), invG);
            p[c] = cl(v);
        }
    }
}

void adjust_brightness_contrast(Image& img, float brightness, float contrast) {
    float cf = contrast >= 0 ? (1.0f / std::max(1e-5f, 1.0f - contrast)) : (1.0f + contrast);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        for (int c = 0; c < 3; ++c) {
            float v = p[c] + brightness;
            v = (v - 0.5f) * cf + 0.5f;
            p[c] = cl(v);
        }
    }
}

void adjust_hue_saturation(Image& img, float hueShift, float saturation, float lightness, int targetChannel) {
    float hShift = hueShift / 360.0f;
    float sMul = 1.0f + saturation;
    float lAdd = lightness;
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        float h, s, l;
        Color c{p[0], p[1], p[2], 1};
        Color hsl = rgb_to_hsl(p[0], p[1], p[2]);
        h = hsl.r; s = hsl.g; l = hsl.b;
        bool apply = true;
        if (targetChannel >= 0 && targetChannel <= 6) {
            float center[] = {0.0f, 1.0f/6.0f, 2.0f/6.0f, 3.0f/6.0f, 4.0f/6.0f, 5.0f/6.0f, 0.0f};
            float range = 1.0f / 6.0f;
            if (targetChannel == 6) apply = true;
            else {
                float d = std::fabs(h - center[targetChannel]);
                d = std::min(d, 1.0f - d);
                apply = d < range;
            }
        }
        if (!apply) continue;
        float ns = cl(s * sMul);
        float nl = cl(l + lAdd * (l < 0.5f ? 2.0f * l : 2.0f * (1.0f - l)));
        float nh = std::fmod(h + hShift + 1.0f, 1.0f);
        Color out; out.r = nh; out.g = ns; out.b = nl; out.a = 1;
        float r, g, b;
        hsl_to_rgb(out, r, g, b);
        p[0] = cl(r); p[1] = cl(g); p[2] = cl(b);
    }
}

void adjust_vibrance(Image& img, float vibrance, float saturation) {
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        float mx = std::max({p[0], p[1], p[2]});
        float mn = std::min({p[0], p[1], p[2]});
        float sat = mx > mn ? (mx - mn) / mx : 0.0f;
        float vAmt = vibrance * (1.0f - sat);
        float sAmt = saturation;
        float total = 1.0f + vAmt + sAmt;
        p[0] = cl(p[0] * total); p[1] = cl(p[1] * total); p[2] = cl(p[2] * total);
    }
}

static float cbWeight(float lum, float lo, float hi) {
    if (lum <= lo) return 0.0f;
    if (lum >= hi) return 1.0f;
    float t = (lum - lo) / (hi - lo);
    return t * t * (3.0f - 2.0f * t);
}

void adjust_color_balance(Image& img, float crS, float crM, float crH, float mgS, float mgM, float mgH, float ybS, float ybM, float ybH) {
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        float L = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
        float wS = cbWeight(L, 0.0f, 0.64f);
        float wH = cbWeight(L, 0.36f, 1.0f);
        float wM = 1.0f - wS - wH; if (wM < 0) wM = 0;
        float cr = crS * wS + crM * wM + crH * wH;
        float mg = mgS * wS + mgM * wM + mgH * wH;
        float yb = ybS * wS + ybM * wM + ybH * wH;
        p[0] = cl(p[0] + cr); p[1] = cl(p[1] - mg); p[2] = cl(p[2] - yb);
    }
}

void adjust_black_white(Image& img, float r, float y, float g, float c, float b, float m) {
    float wr = r * 0.6f + 0.0722f, wg = g * 0.587f + y * 0.1f, wb = b * 0.81f + c * 0.05f + m * 0.05f;
    float sum = wr + wg + wb;
    if (sum == 0) sum = 1;
    wr /= sum; wg /= sum; wb /= sum;
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        float v = cl(wr * p[0] + wg * p[1] + wb * p[2]);
        p[0] = p[1] = p[2] = v;
    }
}

void adjust_photo_filter(Image& img, int filterIndex, float density) {
    static const float filters[][3] = {
        {1.0f, 0.9f, 0.7f}, {0.7f, 0.8f, 1.0f}, {1.0f, 0.85f, 0.6f}, {0.6f, 0.75f, 1.0f},
        {1.0f, 0.6f, 0.6f}, {0.6f, 1.0f, 0.6f}, {0.6f, 0.6f, 1.0f}, {1.0f, 1.0f, 0.6f},
        {0.8f, 0.8f, 0.8f}, {0.4f, 0.6f, 1.0f}
    };
    int idx = filterIndex % 10;
    float d = cl(density);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        for (int c = 0; c < 3; ++c) {
            float f = filters[idx][c];
            float filtered = p[c] * f;
            p[c] = cl(p[c] * (1.0f - d) + filtered * d);
        }
    }
}

void adjust_channel_mixer(Image& img, bool mono, float rR, float gR, float bR, float rG, float gG, float bG, float rB, float gB, float bB, float constants) {
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        float r = p[0], g = p[1], b = p[2];
        float nr, ng, nb;
        if (mono) { nr = ng = nb = 0.333f * (r + g + b); }
        else { nr = rR * r + gR * g + bR * b; ng = rG * r + gG * g + bG * b; nb = rB * r + gB * g + bB * b; }
        p[0] = cl(nr + constants); p[1] = cl(ng + constants); p[2] = cl(nb + constants);
    }
}

void adjust_posterize(Image& img, int levels) {
    if (levels < 2) levels = 2;
    float step = 1.0f / (levels - 1);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        for (int c = 0; c < 3; ++c) p[c] = std::round(p[c] / step) * step;
    }
}

void adjust_threshold(Image& img, float level) {
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        float L = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
        float v = L >= level ? 1.0f : 0.0f;
        p[0] = p[1] = p[2] = v;
    }
}

void adjust_invert(Image& img) {
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        p[0] = 1.0f - p[0]; p[1] = 1.0f - p[1]; p[2] = 1.0f - p[2];
    }
}

void adjust_gradient_map(Image& img, const std::vector<float>& gradientRGBA, int stops) {
    if (stops <= 0) return;
    std::vector<float> lutR(256), lutG(256), lutB(256);
    for (int i = 0; i < 256; ++i) {
        float t = i / 255.0f * (stops - 1);
        int s0 = static_cast<int>(t); if (s0 >= stops - 1) s0 = stops - 2; if (s0 < 0) s0 = 0;
        float ft = t - s0;
        const float* a = &gradientRGBA[s0 * 4];
        const float* b = &gradientRGBA[(s0 + 1) * 4];
        lutR[i] = a[0] + (b[0] - a[0]) * ft;
        lutG[i] = a[1] + (b[1] - a[1]) * ft;
        lutB[i] = a[2] + (b[2] - a[2]) * ft;
    }
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        float L = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
        int li = static_cast<int>(cl(L) * 255.0f);
        p[0] = lutR[li]; p[1] = lutG[li]; p[2] = lutB[li];
    }
}

static int selectiveRange(float h, float s, float l, int range) {
    switch (range) {
        case 0: return 1;
        case 1: return l < 0.4f;
        case 2: return l >= 0.4f && l <= 0.6f;
        case 3: return l > 0.6f;
        case 4: return s > 0.3f && h < 0.1f || s > 0.3f && h > 0.9f;
        case 5: return s > 0.3f && h >= 0.1f && h < 0.4f;
        case 6: return s > 0.3f && h >= 0.4f && h < 0.7f;
        case 7: return s > 0.3f && h >= 0.7f;
    }
    return 1;
}

void adjust_selective_color(Image& img, int correctionRange, float c, float m, float y, float k) {
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        Color hsl = rgb_to_hsl(p[0], p[1], p[2]);
        if (!selectiveRange(hsl.r, hsl.g, hsl.b, correctionRange)) continue;
        float cc, mm, yy, kk;
        rgb_to_cmyk(p[0], p[1], p[2], cc, mm, yy, kk);
        cc = cl(cc + c); mm = cl(mm + m); yy = cl(yy + y); kk = cl(kk + k);
        cmyk_to_rgb(cc, mm, yy, kk, p[0], p[1], p[2]);
    }
}

void adjust_shadow_highlight(Image& img, float shadowAmount, float shadowWidth, float highlightAmount, float highlightWidth, float radius) {
    Image tmp = clone_image(img);
    gaussian_blur(tmp, radius > 0 ? radius : 10.0f);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i * 4];
        const float* tp = &tmp.pixels[i * 4];
        for (int c = 0; c < 3; ++c) {
            float localAvg = tp[c];
            float diff = p[c] - localAvg;
            if (diff < 0) p[c] = cl(p[c] + (-diff) * shadowAmount * shadowWidth);
            else p[c] = cl(p[c] - diff * highlightAmount * highlightWidth);
        }
    }
}

void auto_contrast(Image& img) {
    float hist[256] = {0};
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float L = 0.299f * img.pixels[i*4] + 0.587f * img.pixels[i*4+1] + 0.114f * img.pixels[i*4+2];
        hist[static_cast<int>(cl(L) * 255)] += 1.0f;
    }
    float clip = n * 0.005f;
    float acc = 0; int lo = 0;
    for (int i = 0; i < 256; ++i) { acc += hist[i]; if (acc > clip) { lo = i; break; } }
    acc = 0; int hi = 255;
    for (int i = 255; i >= 0; --i) { acc += hist[i]; if (acc > clip) { hi = i; break; } }
    if (hi <= lo) hi = lo + 1;
    adjust_levels(img, lo / 255.0f, hi / 255.0f, 1.0f, 0.0f, 1.0f);
}

void auto_levels(Image& img) {
    float lo = 1.0f, hi = 0.0f;
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        for (int c = 0; c < 3; ++c) { lo = std::min(lo, img.pixels[i*4+c]); hi = std::max(hi, img.pixels[i*4+c]); }
    }
    if (hi <= lo) hi = lo + 0.001f;
    adjust_levels(img, lo, hi, 1.0f, 0.0f, 1.0f);
}

void auto_tone(Image& img) {
    float hist[256] = {0};
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float L = 0.299f * img.pixels[i*4] + 0.587f * img.pixels[i*4+1] + 0.114f * img.pixels[i*4+2];
        hist[static_cast<int>(cl(L)*255)] += 1;
    }
    float mean = 0; for (int i = 0; i < 256; ++i) mean += hist[i] * i; mean /= std::max<size_t>(1, n);
    float target = 128.0f;
    float gamma = std::pow(2.0f, (target - mean) / 64.0f);
    adjust_levels(img, 0, 1, gamma, 0, 1);
}

void equalize(Image& img) {
    float hist[256] = {0};
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float L = 0.299f * img.pixels[i*4] + 0.587f * img.pixels[i*4+1] + 0.114f * img.pixels[i*4+2];
        hist[static_cast<int>(cl(L)*255)] += 1;
    }
    float cdf[256]; cdf[0] = hist[0];
    for (int i = 1; i < 256; ++i) cdf[i] = cdf[i-1] + hist[i];
    float denom = std::max(1.0f, cdf[255]);
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i*4];
        float L = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
        int idx = static_cast<int>(cl(L)*255);
        float v = cdf[idx] / denom;
        p[0] = p[1] = p[2] = v;
    }
}

void desaturate(Image& img) {
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        float* p = &img.pixels[i*4];
        float L = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
        p[0] = p[1] = p[2] = L;
    }
}

}
