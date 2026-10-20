#include "pt_blend.h"
#include <cmath>
#include <algorithm>

namespace pt {

static float clampf(float v) { return v < 0.0f ? (v > 1.0f ? 1.0f : v) : (v < 0.0f ? 0.0f : v); }

static float darkenB(float b, float s) { return std::min(b, s); }
static float multiplyB(float b, float s) { return b * s; }
static float colorBurnB(float b, float s) { return s <= 0.0f ? 0.0f : clampf(1.0f - (1.0f - b) / s); }
static float linearBurnB(float b, float s) { return clampf(b + s - 1.0f); }
static float lightenB(float b, float s) { return std::max(b, s); }
static float screenB(float b, float s) { return 1.0f - (1.0f - b) * (1.0f - s); }
static float colorDodgeB(float b, float s) { return s >= 1.0f ? 1.0f : clampf(b / (1.0f - s)); }
static float linearDodgeB(float b, float s) { return clampf(b + s); }
static float hardLightB(float b, float s) { return s <= 0.5f ? (s == 0.5f ? b : (2.0f * s * b)) : 1.0f - 2.0f * (1.0f - s) * (1.0f - b); }
static float softLightB(float b, float s) {
    if (s <= 0.5f) return b - (1.0f - 2.0f * s) * b * (1.0f - b);
    float d = b > 0.25f ? std::sqrt(b) : ((16.0f * b - 12.0f) * b + 4.0f) * b;
    return b + (2.0f * s - 1.0f) * (d - b);
}
static float vividLightB(float b, float s) { return s <= 0.5f ? colorBurnB(b, 2.0f * s) : colorDodgeB(b, 2.0f * s - 1.0f); }
static float linearLightB(float b, float s) { return s <= 0.5f ? linearBurnB(b, 2.0f * s) : linearDodgeB(b, 2.0f * s - 1.0f); }
static float pinLightB(float b, float s) { return s <= 0.5f ? std::min(b, 2.0f * s) : std::max(b, 2.0f * s - 1.0f); }
static float differenceB(float b, float s) { return std::fabs(b - s); }
static float exclusionB(float b, float s) { return b + s - 2.0f * b * s; }

static void lumRGB(float r, float g, float b, float& L) { L = 0.299f * r + 0.587f * g + 0.114f * b; }

static float satOf(float r, float g, float b) {
    float mx = std::max({r, g, b});
    float mn = std::min({r, g, b});
    return mx - mn;
}

static void clipColor(float& r, float& g, float& b, float L) {
    float mn = std::min({r, g, b});
    float mx = std::max({r, g, b});
    if (mn < 0.0f) { r = L + (r - L) * L / (L - mn); g = L + (g - L) * L / (L - mn); b = L + (b - L) * L / (L - mn); }
    mx = std::max({r, g, b});
    if (mx > 1.0f) {
        float l2 = L;
        r = l2 + (r - l2) * (1.0f - l2) / (mx - l2);
        g = l2 + (g - l2) * (1.0f - l2) / (mx - l2);
        b = l2 + (b - l2) * (1.0f - l2) / (mx - l2);
    }
}

static void setSaturation(float& r, float& g, float& b, float s) {
    float mx = std::max({r, g, b}), mn = std::min({r, g, b});
    float L; lumRGB(r, g, b, L);
    if (mx > mn) {
        r = (r - mn) * s / (mx - mn);
        g = (g - mn) * s / (mx - mn);
        b = (b - mn) * s / (mx - mn);
    } else { r = g = b = L; }
    clipColor(r, g, b, L);
}

static void setLuminosity(float& r, float& g, float& b, float L) {
    float cur; lumRGB(r, g, b, cur);
    float d = L - cur;
    r += d; g += d; b += d;
    clipColor(r, g, b, L);
}

static void colorOfHSL(float p, float q, float t, float& out) {
    if (t < 0) t += 1; if (t > 1) t -= 1;
    if (t < 1.0f/6.0f) out = p + (q - p) * 6.0f * t;
    else if (t < 1.0f/2.0f) out = q;
    else if (t < 2.0f/3.0f) out = p + (q - p) * (2.0f/3.0f - t) * 6.0f;
    else out = p;
}

static void rgbToHsl(float r, float g, float b, float& h, float& s, float& l) {
    float mx = std::max({r, g, b}), mn = std::min({r, g, b});
    l = (mx + mn) * 0.5f;
    if (mx == mn) { h = 0; s = 0; return; }
    float d = mx - mn;
    s = l > 0.5f ? d / (2.0f - mx - mn) : d / (mx + mn);
    if (mx == r) h = (g - b) / d + (g < b ? 6 : 0);
    else if (mx == g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
    h /= 6.0f;
}

static void hslToRgb(float h, float s, float l, float& r, float& g, float& b) {
    if (s == 0) { r = g = b = l; return; }
    float q = l < 0.5f ? l * (1 + s) : l + s - l * s;
    float p = 2 * l - q;
    colorOfHSL(p, q, h + 1.0f/3.0f, r);
    colorOfHSL(p, q, h, g);
    colorOfHSL(p, q, h - 1.0f/3.0f, b);
}

static void blendPixel(float* d, const float* s, BlendMode mode) {
    float br = d[0], bg = d[1], bb = d[2];
    float sr = s[0], sg = s[1], sb = s[2];
    float outR = br, outG = bg, outB = bb;
    switch (mode) {
        case BlendMode::Normal: case BlendMode::PassThrough: outR = sr; outG = sg; outB = sb; break;
        case BlendMode::Darken: outR = darkenB(br,sr); outG = darkenB(bg,sg); outB = darkenB(bb,sb); break;
        case BlendMode::Multiply: outR = multiplyB(br,sr); outG = multiplyB(bg,sg); outB = multiplyB(bb,sb); break;
        case BlendMode::ColorBurn: outR = colorBurnB(br,sr); outG = colorBurnB(bg,sg); outB = colorBurnB(bb,sb); break;
        case BlendMode::LinearBurn: outR = linearBurnB(br,sr); outG = linearBurnB(bg,sg); outB = linearBurnB(bb,sb); break;
        case BlendMode::Lighter: outR = lightenB(br,sr); outG = lightenB(bg,sg); outB = lightenB(bb,sb); break;
        case BlendMode::Screen: outR = screenB(br,sr); outG = screenB(bg,sg); outB = screenB(bb,sb); break;
        case BlendMode::ColorDodge: outR = colorDodgeB(br,sr); outG = colorDodgeB(bg,sg); outB = colorDodgeB(bb,sb); break;
        case BlendMode::LinearDodge: outR = linearDodgeB(br,sr); outG = linearDodgeB(bg,sg); outB = linearDodgeB(bb,sb); break;
        case BlendMode::HardLight: outR = hardLightB(br,sr); outG = hardLightB(bg,sg); outB = hardLightB(bb,sb); break;
        case BlendMode::SoftLight: outR = softLightB(br,sr); outG = softLightB(bg,sg); outB = softLightB(bb,sb); break;
        case BlendMode::VividLight: outR = vividLightB(br,sr); outG = vividLightB(bg,sg); outB = vividLightB(bb,sb); break;
        case BlendMode::LinearLight: outR = linearLightB(br,sr); outG = linearLightB(bg,sg); outB = linearLightB(bb,sb); break;
        case BlendMode::PinLight: outR = pinLightB(br,sr); outG = pinLightB(bg,sg); outB = pinLightB(bb,sb); break;
        case BlendMode::Difference: outR = differenceB(br,sr); outG = differenceB(bg,sg); outB = differenceB(bb,sb); break;
        case BlendMode::Exclusion: outR = exclusionB(br,sr); outG = exclusionB(bg,sg); outB = exclusionB(bb,sb); break;
        case BlendMode::Hue: {
            float bh, bsat, bl, sh, ssat, sl;
            rgbToHsl(br,bg,bb,bh,bsat,bl); rgbToHsl(sr,sg,sb,sh,ssat,sl);
            float r1,g1,b1; hslToRgb(sh,bsat,bl,r1,g1,b1);
            outR=r1; outG=g1; outB=b1; break;
        }
        case BlendMode::Saturation: {
            float bh, bsat, bl, sh, ssat, sl;
            rgbToHsl(br,bg,bb,bh,bsat,bl); rgbToHsl(sr,sg,sb,sh,ssat,sl);
            float r1,g1,b1; hslToRgb(bh,ssat,bl,r1,g1,b1);
            outR=r1; outG=g1; outB=b1; break;
        }
        case BlendMode::Color: {
            float L; lumRGB(sr,sg,sb,L);
            float r1=sr,g1=sg,b1=sb;
            setLuminosity(r1,g1,b1,L);
            float bl; lumRGB(br,bg,bb,bl);
            outR=r1; outG=g1; outB=b1;
            float dummy; lumRGB(outR,outG,outB,dummy);
            float rr=outR,gg=outG,bb2=outB;
            setSaturation(rr,gg,bb2,satOf(sr,sg,sb));
            outR=rr;outG=gg;outB=bb2;
            break;
        }
        case BlendMode::Luminosity: {
            float L; lumRGB(sr,sg,sb,L);
            float r1=br,g1=bg,b1=bb;
            setLuminosity(r1,g1,b1,L);
            outR=r1; outG=g1; outB=b1; break;
        }
        case BlendMode::Dissolve: {
            uint32_t h = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(d) / 4u);
            h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15;
            float rnd = (h & 0xffff) / 65535.0f;
            if (rnd < s[3]) { outR=sr; outG=sg; outB=sb; } else { outR=br; outG=bg; outB=bb; }
            break;
        }
    }
    d[0] = clampf(outR); d[1] = clampf(outG); d[2] = clampf(outB);
}

void blend_images(float* dst, const float* src, size_t count, BlendMode mode, float opacity) {
    for (size_t i = 0; i < count; ++i) {
        float* d = dst + i * 4;
        const float* s = src + i * 4;
        float sa = s[3] * opacity;
        if (sa <= 0.0f) continue;
        float da = d[3];
        float oldR = d[0], oldG = d[1], oldB = d[2];
        blendPixel(d, s, mode);
        float nr = oldR + (d[0] - oldR) * sa;
        float ng = oldG + (d[1] - oldG) * sa;
        float nb = oldB + (d[2] - oldB) * sa;
        float na = sa + da * (1.0f - sa);
        if (na > 0.0f) { d[0] = (nr * sa + oldR * da * (1.0f - sa)) / na; d[1] = (ng * sa + oldG * da * (1.0f - sa)) / na; d[2] = (nb * sa + oldB * da * (1.0f - sa)) / na; }
        d[3] = na;
    }
}

void blend_with_mask(float* dst, const float* src, const float* mask, size_t count, BlendMode mode, float opacity) {
    for (size_t i = 0; i < count; ++i) {
        float m = mask[i];
        if (m <= 0.0f) continue;
        float* d = dst + i * 4;
        const float* s = src + i * 4;
        float sa = s[3] * opacity * m;
        if (sa <= 0.0f) continue;
        float da = d[3];
        float oldR = d[0], oldG = d[1], oldB = d[2];
        blendPixel(d, s, mode);
        float nr = oldR + (d[0] - oldR) * sa;
        float ng = oldG + (d[1] - oldG) * sa;
        float nb = oldB + (d[2] - oldB) * sa;
        float na = sa + da * (1.0f - sa);
        if (na > 0.0f) { d[0] = (nr * sa + oldR * da * (1.0f - sa)) / na; d[1] = (ng * sa + oldG * da * (1.0f - sa)) / na; d[2] = (nb * sa + oldB * da * (1.0f - sa)) / na; }
        d[3] = na;
    }
}

void apply_opacity(float* rgba, size_t count, float opacity) {
    for (size_t i = 0; i < count; ++i) rgba[i * 4 + 3] *= opacity;
}

void fill_rect(float* rgba, uint32_t imgW, uint32_t imgH, int x, int y, int w, int h, const Color& c) {
    int x0 = std::max(0, x), y0 = std::max(0, y);
    int x1 = std::min(static_cast<int>(imgW), x + w), y1 = std::min(static_cast<int>(imgH), y + h);
    for (int yy = y0; yy < y1; ++yy) {
        float* row = rgba + static_cast<size_t>(yy) * imgW * 4;
        for (int xx = x0; xx < x1; ++xx) {
            float* p = row + xx * 4;
            float a = c.a;
            p[0] = p[0] * (1 - a) + c.r * a;
            p[1] = p[1] * (1 - a) + c.g * a;
            p[2] = p[2] * (1 - a) + c.b * a;
            p[3] = std::min(1.0f, p[3] + a);
        }
    }
}

}
