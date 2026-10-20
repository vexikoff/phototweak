#include "pt_math.h"
#include <cmath>
#include <cstring>
#include <cstdint>

namespace pt {

static uint32_t g_seed = 0x12345678u;

void set_global_seed(uint32_t s) { g_seed = s; }

uint32_t hash_u32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

float hash_float_2d(int x, int y, uint32_t seed) {
    uint32_t h = static_cast<uint32_t>(x) * 374761393u + static_cast<uint32_t>(y) * 668265263u + seed;
    h = hash_u32(h);
    return (h & 0xffffff) / float(0x1000000);
}

static const uint8_t perm[256] = {
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
    190,6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,77,33,88,237,62,55,18,218,153,
    77,73,103,166,68,113,37,161,100,87,153,80,87,250,145,5,165,93,174,155,163,124,168,189,24,86,80,1,
    191,115,104,76,162,101,126,110,74,163,93,40,87,159,107,71,7,25,171,28,193,190,84,154,239,72,14,
    158,16,84,208,99,22,68,232,40,83,121,74,153,38,54,171,14,200,133,130,30,16,152,33,178,85,166,123,
    123,151,66,167,136,122,253,195,98,222,117,103,110,183,87,193,253,131,130,116,1,80,194,251,10,180,
    118,238,68,135,84,163,193,17,26,206,245,102,144,43,103,176,41,141,122,59,28,41,16,66,35,116,190,
    56,8,125,118,182,169,235,86,239,122,69,163,36,143,9,98,150,219,142,134,167,176,168,195,2,126,147,
    41,20,138,106,124,163,195,17,178,179,9,15,160,181,183,6,31,107,149,71,163,193,86,158,9,13,176,166,
    111,196,182,35,61,2,117,16,55,117,52,63,8,151,122,180,136,141,157,114,121,149,63,98,107,108,176,166,
    113,165,31,17,3,101,146,139,222,221,93,100,154,27,170,5,160,61,30,173,131,177,182,85,240,109,169
};

static uint8_t ptable[512];
static bool ptableInit = false;

static void ensurePerm() {
    if (!ptableInit) {
        for (int i = 0; i < 256; ++i) { ptable[i] = perm[i]; ptable[256 + i] = perm[i]; }
        ptableInit = true;
    }
}

static float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
static float lerp(float a, float b, float t) { return a + t * (b - a); }

static float grad2(int h, float x, float y) {
    switch (h & 3) {
        case 0: return x + y;
        case 1: return -x + y;
        case 2: return x - y;
        default: return -x - y;
    }
}

static float grad3(int h, float x, float y, float z) {
    switch (h & 15) {
        case 0: case 12: return x + y;
        case 1: case 14: return -x + y;
        case 2: return x - y;
        case 4: case 13: return x + z;
        case 5: case 15: return -x + z;
        case 6: case 9: return x - z;
        case 7: case 10: return -x - z;
        case 3: return -x - y;
        case 8: return -y + z;
        case 11: return y + z;
        default: return y - z;
    }
}

float perlin2(float x, float y, int seed) {
    ensurePerm();
    int X = static_cast<int>(std::floor(x)) & 255;
    int Y = static_cast<int>(std::floor(y)) & 255;
    x -= std::floor(x); y -= std::floor(y);
    int u = static_cast<int>(fade(x) * 1000), v = static_cast<int>(fade(y) * 1000);
    float fu = fade(x), fv = fade(y);
    int A = ptable[X] + Y + (seed & 255), AA = ptable[A], AB = ptable[A + 1];
    int B = ptable[X + 1] + Y + (seed & 255), BA = ptable[B], BB = ptable[B + 1];
    float n00 = grad2(ptable[AA], x, y);
    float n10 = grad2(ptable[BA], x - 1, y);
    float n01 = grad2(ptable[AB], x, y - 1);
    float n11 = grad2(ptable[BB], x - 1, y - 1);
    return lerp(lerp(n00, n10, fu), lerp(n01, n11, fu), fv) * 0.7f;
}

float perlin3(float x, float y, float z, int seed) {
    ensurePerm();
    int X = static_cast<int>(std::floor(x)) & 255;
    int Y = static_cast<int>(std::floor(y)) & 255;
    int Z = static_cast<int>(std::floor(z)) & 255;
    x -= std::floor(x); y -= std::floor(y); z -= std::floor(z);
    float fu = fade(x), fv = fade(y), fw = fade(z);
    int A = ptable[X] + Y, AA = ptable[A] + Z, AB = ptable[A + 1] + Z;
    int B = ptable[X + 1] + Y, BA = ptable[B] + Z, BB = ptable[B + 1] + Z;
    int s = seed & 255;
    float n000 = grad3(ptable[AA + s], x, y, z);
    float n100 = grad3(ptable[BA + s], x - 1, y, z);
    float n010 = grad3(ptable[AB + s], x, y - 1, z);
    float n110 = grad3(ptable[BB + s], x - 1, y - 1, z);
    float n001 = grad3(ptable[AA + 1 + s], x, y, z - 1);
    float n101 = grad3(ptable[BA + 1 + s], x - 1, y, z - 1);
    float n011 = grad3(ptable[AB + 1 + s], x, y - 1, z - 1);
    float n111 = grad3(ptable[BB + 1 + s], x - 1, y - 1, z - 1);
    return lerp(lerp(lerp(n000, n100, fu), lerp(n010, n110, fu), fv),
                lerp(lerp(n001, n101, fu), lerp(n011, n111, fu), fv), fw);
}

static const int simplexGrad3[12][3] = {
    {1,1,0},{-1,1,0},{1,-1,0},{-1,-1,0},
    {1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1},
    {0,1,1},{0,-1,1},{0,1,-1},{0,-1,-1}
};

float simplex2(float x, float y, int seed) {
    ensurePerm();
    const float F2 = 0.5f * (std::sqrt(3.0f) - 1.0f);
    const float G2 = (3.0f - std::sqrt(3.0f)) / 6.0f;
    float s = (x + y) * F2;
    int i = static_cast<int>(std::floor(x + s));
    int j = static_cast<int>(std::floor(y + s));
    float t = (i + j) * G2;
    float X0 = i - t, Y0 = j - t;
    float x0 = x - X0, y0 = y - Y0;
    int i1, j1;
    if (x0 > y0) { i1 = 1; j1 = 0; } else { i1 = 0; j1 = 1; }
    float x1 = x0 - i1 + G2, y1 = y0 - j1 + G2;
    float x2 = x0 - 1.0f + 2.0f * G2, y2 = y0 - 1.0f + 2.0f * G2;
    int ii = i & 255, jj = j & 255;
    int gi0 = ptable[ii + ptable[jj + (seed & 255)]] % 12;
    int gi1 = ptable[ii + i1 + ptable[jj + j1 + (seed & 255)]] % 12;
    int gi2 = ptable[ii + 1 + ptable[jj + 1 + (seed & 255)]] % 12;
    float n0 = 0, n1 = 0, n2 = 0;
    float t0 = 0.5f - x0*x0 - y0*y0;
    if (t0 >= 0) { t0 *= t0; n0 = t0 * t0 * (simplexGrad3[gi0][0]*x0 + simplexGrad3[gi0][1]*y0); }
    float t1 = 0.5f - x1*x1 - y1*y1;
    if (t1 >= 0) { t1 *= t1; n1 = t1 * t1 * (simplexGrad3[gi1][0]*x1 + simplexGrad3[gi1][1]*y1); }
    float t2 = 0.5f - x2*x2 - y2*y2;
    if (t2 >= 0) { t2 *= t2; n2 = t2 * t2 * (simplexGrad3[gi2][0]*x2 + simplexGrad3[gi2][1]*y2); }
    return 70.0f * (n0 + n1 + n2);
}

float simplex3(float x, float y, float z, int seed) {
    ensurePerm();
    const float F3 = 1.0f / 3.0f;
    const float G3 = 1.0f / 6.0f;
    float s = (x + y + z) * F3;
    int i = static_cast<int>(std::floor(x + s)), j = static_cast<int>(std::floor(y + s)), k = static_cast<int>(std::floor(z + s));
    float t = (i + j + k) * G3;
    float x0 = x - (i - t), y0 = y - (j - t), z0 = z - (k - t);
    int i1, j1, k1, i2, j2, k2;
    if (x0 >= y0) {
        if (y0 >= z0) { i1=1;j1=0;k1=0;i2=1;j2=1;k2=0; }
        else if (x0 >= z0) { i1=1;j1=0;k1=0;i2=1;j2=0;k2=1; }
        else { i1=0;j1=0;k1=1;i2=1;j2=0;k2=1; }
    } else {
        if (y0 < z0) { i1=0;j1=0;k1=1;i2=0;j2=1;k2=1; }
        else if (x0 < z0) { i1=0;j1=1;k1=0;i2=0;j2=1;k2=1; }
        else { i1=0;j1=1;k1=0;i2=1;j2=1;k2=0; }
    }
    float x1=x0-i1+G3, y1=y0-j1+G3, z1=z0-k1+G3;
    float x2=x0-i2+2*G3, y2=y0-j2+2*G3, z2=z0-k2+2*G3;
    float x3=x0-1+3*G3, y3=y0-1+3*G3, z3=z0-1+3*G3;
    int ii=i&255, jj=j&255, kk=k&255;
    int sd = seed & 255;
    int gi0 = ptable[ii+ptable[jj+ptable[kk+sd]]] % 12;
    int gi1 = ptable[ii+i1+ptable[jj+j1+ptable[kk+k1+sd]]] % 12;
    int gi2 = ptable[ii+i2+ptable[jj+j2+ptable[kk+k2+sd]]] % 12;
    int gi3 = ptable[ii+1+ptable[jj+1+ptable[kk+1+sd]]] % 12;
    auto contrib = [](float xx, float yy, float zz, int gi) -> float {
        float tt = 0.6f - xx*xx - yy*yy - zz*zz;
        if (tt < 0) return 0;
        tt *= tt;
        return tt * tt * (simplexGrad3[gi][0]*xx + simplexGrad3[gi][1]*yy + simplexGrad3[gi][2]*zz);
    };
    return 32.0f * (contrib(x0,y0,z0,gi0)+contrib(x1,y1,z1,gi1)+contrib(x2,y2,z2,gi2)+contrib(x3,y3,z3,gi3));
}

float worley2(float x, float y, int seed, bool f2) {
    int cx = static_cast<int>(std::floor(x)), cy = static_cast<int>(std::floor(y));
    float d1 = 1e9f, d2 = 1e9f;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        int gx = cx + dx, gy = cy + dy;
        float px = gx + hash_float_2d(gx, gy, seed);
        float py = gy + hash_float_2d(gx, gy, seed ^ 0x9e3779b9);
        float dist = std::hypot(px - x, py - y);
        if (dist < d1) { d2 = d1; d1 = dist; }
        else if (dist < d2) d2 = dist;
    }
    return f2 ? d2 : d1;
}

float worley3(float x, float y, float z, int seed, bool f2) {
    int cx = static_cast<int>(std::floor(x)), cy = static_cast<int>(std::floor(y)), cz = static_cast<int>(std::floor(z));
    float d1 = 1e9f, d2 = 1e9f;
    for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        int gx = cx+dx, gy = cy+dy, gz = cz+dz;
        float px = gx + hash_float_2d(gx, gy + gz * 31, seed);
        float py = gy + hash_float_2d(gx + gz * 17, gy, seed ^ 0x85ebca6b);
        float pz = gz + hash_float_2d(gx, gy + gz * 13, seed ^ 0xc2b2ae35);
        float dist = std::sqrt((px-x)*(px-x)+(py-y)*(py-y)+(pz-z)*(pz-z));
        if (dist < d1) { d2 = d1; d1 = dist; }
        else if (dist < d2) d2 = dist;
    }
    return f2 ? d2 : d1;
}

float fbm_noise(NoiseType type, float x, float y, int octaves, float lacunarity, float gain, int seed) {
    float sum = 0, amp = 1, freq = 1, norm = 0;
    for (int o = 0; o < octaves; ++o) {
        float v = 0;
        if (type == NoiseType::Perlin) v = perlin2(x * freq, y * freq, seed + o * 131);
        else if (type == NoiseType::Simplex) v = simplex2(x * freq, y * freq, seed + o * 131);
        else v = worley2(x * freq, y * freq, seed + o * 131, false) * 2.0f - 1.0f;
        sum += v * amp;
        norm += amp;
        amp *= gain;
        freq *= lacunarity;
    }
    return norm > 0 ? sum / norm : 0;
}

Color rgb_to_hsl(float r, float g, float b) {
    Color out;
    float mx = std::max({r,g,b}), mn = std::min({r,g,b});
    float l = (mx + mn) * 0.5f;
    float h = 0, s = 0;
    if (mx != mn) {
        float d = mx - mn;
        s = l > 0.5f ? d / (2 - mx - mn) : d / (mx + mn);
        if (mx == r) h = (g - b) / d + (g < b ? 6 : 0);
        else if (mx == g) h = (b - r) / d + 2;
        else h = (r - g) / d + 4;
        h /= 6;
    }
    out.r = h; out.g = s; out.b = l; out.a = 1;
    return out;
}

static void hue2rgb(float p, float q, float t, float& out) {
    if (t < 0) t += 1; if (t > 1) t -= 1;
    if (t < 1/6.f) out = p + (q-p)*6*t;
    else if (t < 1/2.f) out = q;
    else if (t < 2/3.f) out = p + (q-p)*(2/3.f - t)*6;
    else out = p;
}

void hsl_to_rgb(const Color& hsl, float& r, float& g, float& b) {
    float h = hsl.r, s = hsl.g, l = hsl.b;
    if (s == 0) { r = g = b = l; return; }
    float q = l < 0.5f ? l*(1+s) : l+s-l*s;
    float p = 2*l - q;
    hue2rgb(p,q,h+1/3.f,r); hue2rgb(p,q,h,g); hue2rgb(p,q,h-1/3.f,b);
}

Color rgb_to_hsv(float r, float g, float b) {
    Color out;
    float mx = std::max({r,g,b}), mn = std::min({r,g,b});
    float d = mx - mn;
    out.r = 0; out.g = mx > 0 ? d/mx : 0; out.b = mx; out.a = 1;
    if (d > 0) {
        if (mx == r) out.r = std::fmod((g-b)/d, 6);
        else if (mx == g) out.r = (b-r)/d + 2;
        else out.r = (r-g)/d + 4;
        out.r /= 6; if (out.r < 0) out.r += 1;
    }
    return out;
}

void hsv_to_rgb(const Color& hsv, float& r, float& g, float& b) {
    float h = hsv.r * 6, s = hsv.g, v = hsv.b;
    int i = static_cast<int>(std::floor(h));
    float f = h - i;
    float p = v*(1-s), q = v*(1-s*f), t = v*(1-s*(1-f));
    switch (i % 6) {
        case 0: r=v;g=t;b=p;break;
        case 1: r=q;g=v;b=p;break;
        case 2: r=p;g=v;b=t;break;
        case 3: r=p;g=q;b=v;break;
        case 4: r=t;g=p;b=v;break;
        default: r=v;g=p;b=q;break;
    }
}

void rgb_to_lab(float r, float g, float b, float& L, float& a, float& bb) {
    float rr = r > 0.04045f ? std::pow((r+0.055f)/1.055f, 2.4f) : r/12.92f;
    float gg = g > 0.04045f ? std::pow((g+0.055f)/1.055f, 2.4f) : g/12.92f;
    float bbb = b > 0.04045f ? std::pow((b+0.055f)/1.055f, 2.4f) : b/12.92f;
    float X = rr*0.4124f + gg*0.3576f + bbb*0.1805f;
    float Y = rr*0.2126f + gg*0.7152f + bbb*0.0722f;
    float Z = rr*0.0193f + gg*0.1192f + bbb*0.9505f;
    X /= 0.95047f; Z /= 1.08883f;
    auto f = [](float t){ return t > 0.008856f ? std::cbrt(t) : (7.787f*t + 16.0f/116.0f); };
    float fx = f(X), fy = f(Y), fz = f(Z);
    L = 1.16f*fy - 0.16f;
    a = 5.0f*(fx - fy);
    bb = 2.0f*(fy - fz);
}

void lab_to_rgb(float L, float a, float bb, float& r, float& g, float& b) {
    float fy = (L + 0.16f)/1.16f;
    float fx = a/5.0f + fy;
    float fz = fy - bb/2.0f;
    auto finv = [](float t){ return t*t*t > 0.008856f ? t*t*t : (t - 16.0f/116.0f)/7.787f; };
    float X = finv(fx)*0.95047f, Y = finv(fy), Z = finv(fz)*1.08883f;
    float rr = X*3.2406f - Y*1.5372f - Z*0.4986f;
    float gg = -X*0.9689f + Y*1.8758f + Z*0.0415f;
    float bbb = X*0.0557f - Y*0.2040f + Z*1.0570f;
    auto gam = [](float c){ return c > 0.0031308f ? 1.055f*std::pow(c, 1/2.4f)-0.055f : 12.92f*c; };
    r = gam(rr); g = gam(gg); b = gam(bbb);
}

void rgb_to_cmyk(float r, float g, float b, float& c, float& m, float& y, float& k) {
    k = 1.0f - std::max({r,g,b});
    if (k >= 1.0f) { c=m=y=0; return; }
    c = (1.0f - r - k)/(1.0f - k);
    m = (1.0f - g - k)/(1.0f - k);
    y = (1.0f - b - k)/(1.0f - k);
}

void cmyk_to_rgb(float c, float m, float y, float k, float& r, float& g, float& b) {
    r = (1-c)*(1-k); g = (1-m)*(1-k); b = (1-y)*(1-k);
}

void generate_procedural_texture(Image& out, uint32_t w, uint32_t h, int noiseType,
                                 float scale, int octaves, float lacunarity, float gain,
                                 float turbulenceAmount, int seed,
                                 const Color& colorLow, const Color& colorHigh) {
    out = create_image(w, h, ChannelKind::RGB, 0.0f);
    NoiseType nt = static_cast<NoiseType>(noiseType);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            float nx = x / scale, ny = y / scale;
            float v = fbm_noise(nt, nx, ny, octaves, lacunarity, gain, seed);
            if (turbulenceAmount > 0) {
                float tx = fbm_noise(nt, nx + v*turbulenceAmount, ny + v*turbulenceAmount, octaves, lacunarity, gain, seed+1);
                v = v*(1-turbulenceAmount) + tx*turbulenceAmount;
            }
            float t = v*0.5f + 0.5f;
            if (nt == NoiseType::Worley) t = 1.0f - std::min(1.0f, std::max(0.0f, v));
            size_t idx = (static_cast<size_t>(y)*w + x)*4;
            out.pixels[idx+0] = colorLow.r + (colorHigh.r-colorLow.r)*t;
            out.pixels[idx+1] = colorLow.g + (colorHigh.g-colorLow.g)*t;
            out.pixels[idx+2] = colorLow.b + (colorHigh.b-colorLow.b)*t;
            out.pixels[idx+3] = 1.0f;
        }
    }
}

}
