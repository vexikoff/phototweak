#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define PT_SSE 1
#endif

#if defined(__AVX__)
#include <immintrin.h>
#define PT_AVX 1
#endif

static inline int x_clamp(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float hue2rgb(float p, float q, float t) {
    if (t < 0) t += 1;
    if (t > 1) t -= 1;
    if (t < 1.0f/6.0f) return p + (q - p) * 6 * t;
    if (t < 1.0f/2.0f) return q;
    if (t < 2.0f/3.0f) return p + (q - p) * (2.0f/3.0f - t) * 6;
    return p;
}

static const float kx[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};
static const float ky[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};

void pt_box_blur_c(float* pixels, unsigned int w, unsigned int h, int radius) {
    if (radius <= 0 || !pixels || w == 0 || h == 0) return;
    size_t n = (size_t)w * h;
    float* tmp = (float*)malloc(n * sizeof(float));
    if (!tmp) return;
    int win = 2 * radius + 1;
    for (int ch = 0; ch < 4; ++ch) {
        for (unsigned int y = 0; y < h; ++y) {
            float* row = pixels + ((size_t)y * w) * 4 + ch;
            float sum = 0.0f;
            for (int k = -radius; k <= radius; ++k) {
                int sx = (int)x_clamp((int)k, 0, (int)w - 1);
                sum += row[sx * 4];
            }
            for (unsigned int x = 0; x < w; ++x) {
                tmp[(size_t)y * w + x] = sum / (float)win;
                int addX = x_clamp((int)(x + radius + 1), 0, (int)w - 1);
                int subX = x_clamp((int)x - radius, 0, (int)w - 1);
                sum += row[addX * 4] - row[subX * 4];
            }
        }
        for (unsigned int x = 0; x < w; ++x) {
            float sum = 0.0f;
            for (int k = -radius; k <= radius; ++k) {
                int sy = x_clamp(k, 0, (int)h - 1);
                sum += tmp[(size_t)sy * w + x];
            }
            for (unsigned int y = 0; y < h; ++y) {
                float* px = pixels + ((size_t)y * w + x) * 4 + ch;
                *px = sum / (float)win;
                int addY = x_clamp((int)y + radius + 1, 0, (int)h - 1);
                int subY = x_clamp((int)y - radius, 0, (int)h - 1);
                sum += tmp[(size_t)addY * w + x] - tmp[(size_t)subY * w + x];
            }
        }
    }
    free(tmp);
}

void pt_sobel_c(const float* src, float* dst, unsigned int w, unsigned int h) {
    if (!src || !dst) return;
    for (unsigned int y = 0; y < h; ++y) {
        for (unsigned int x = 0; x < w; ++x) {
            float gx = 0.0f, gy = 0.0f;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int sx = x_clamp((int)x + dx, 0, (int)w - 1);
                    int sy = x_clamp((int)y + dy, 0, (int)h - 1);
                    const float* p = src + ((size_t)sy * w + sx) * 4;
                    float lum = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
                    gx += kx[dy + 1][dx + 1] * lum;
                    gy += ky[dy + 1][dx + 1] * lum;
                }
            }
            float mag = sqrtf(gx * gx + gy * gy);
            if (mag > 1.0f) mag = 1.0f;
            float* d = dst + ((size_t)y * w + x) * 4;
            d[0] = d[1] = d[2] = mag;
            d[3] = src[((size_t)y * w + x) * 4 + 3];
        }
    }
}

void pt_hsl_cycle_c(float* rgba, unsigned long long count, float hueShift) {
#ifdef PT_AVX
    __m256 shift = _mm256_set1_ps(hueShift);
    __m256 one = _mm256_set1_ps(1.0f);
    __m256 zero = _mm256_set1_ps(0.0f);
    __m256i maskIdx = _mm256_setr_epi32(0,1,2,3,4,5,6,7);
    (void)maskIdx;
    unsigned long long i = 0;
    for (; i + 8 <= count; i += 8) {
        float r[8], g[8], b[8];
        for (int k = 0; k < 8; ++k) { r[k]=rgba[(i+k)*4];g[k]=rgba[(i+k)*4+1];b[k]=rgba[(i+k)*4+2]; }
        for (int k = 0; k < 8; ++k) {
            float mx = r[k]>g[k]?(r[k]>b[k]?r[k]:b[k]):(g[k]>b[k]?g[k]:b[k]);
            float mn = r[k]<g[k]?(r[k]<b[k]?r[k]:b[k]):(g[k]<b[k]?g[k]:b[k]);
            float l=(mx+mn)*0.5f,s=0,h=0;
            if(mx!=mn){
                float d=mx-mn;
                s=l>0.5f?d/(2.0f-mx-mn):d/(mx+mn);
                if(mx==r[k])h=((g[k]-b[k])/d+(g[k]<b[k]?6.0f:0.0f))/6.0f;
                else if(mx==g[k])h=((b[k]-r[k])/d+2.0f)/6.0f;
                else h=((r[k]-g[k])/d+4.0f)/6.0f;
            }
            h+=hueShift;h-=floorf(h);
            if(s<=0){r[k]=g[k]=b[k]=l;}
            else{
                float q=l<0.5f?l*(1+s):l+s-l*s,p=2*l-q;
                r[k]=hue2rgb(p,q,h+1.0f/3.0f);g[k]=hue2rgb(p,q,h);b[k]=hue2rgb(p,q,h-1.0f/3.0f);
            }
        }
        for(int k=0;k<8;++k){rgba[(i+k)*4]=r[k];rgba[(i+k)*4+1]=g[k];rgba[(i+k)*4+2]=b[k];}
    }
    for(;i<count;++i){
        float*r=rgba+i*4;
        float mx=fmaxf(r[0],fmaxf(r[1],r[2])),mn=fminf(r[0],fminf(r[1],r[2]));
        float l=(mx+mn)*0.5f,s=0,h=0;
        if(mx!=mn){float d=mx-mn;s=l>0.5f?d/(2-mx-mn):d/(mx+mn);
            if(mx==r[0])h=((r[1]-r[2])/d+(r[1]<r[2]?6:0))/6;
            else if(mx==r[1])h=((r[2]-r[0])/d+2)/6;else h=((r[0]-r[1])/d+4)/6;}
        h+=hueShift;h-=floorf(h);
        if(s<=0)r[0]=r[1]=r[2]=l;
        else{float q=l<0.5f?l*(1+s):l+s-l*s,p=2*l-q;r[0]=hue2rgb(p,q,h+1.f/3);r[1]=hue2rgb(p,q,h);r[2]=hue2rgb(p,q,h-1.f/3);}
    }
#else
    for (unsigned long long i = 0; i < count; ++i) {
        float* r = rgba + i * 4;
        float mx = fmaxf(r[0], fmaxf(r[1], r[2]));
        float mn = fminf(r[0], fminf(r[1], r[2]));
        float l = (mx + mn) * 0.5f, s = 0, h = 0;
        if (mx != mn) {
            float d = mx - mn;
            s = l > 0.5f ? d / (2 - mx - mn) : d / (mx + mn);
            if (mx == r[0]) h = ((r[1] - r[2]) / d + (r[1] < r[2] ? 6 : 0)) / 6;
            else if (mx == r[1]) h = ((r[2] - r[0]) / d + 2) / 6;
            else h = ((r[0] - r[1]) / d + 4) / 6;
        }
        h += hueShift; h -= floorf(h);
        if (s <= 0) r[0] = r[1] = r[2] = l;
        else {
            float q = l < 0.5f ? l * (1 + s) : l + s - l * s, p = 2 * l - q;
            r[0] = hue2rgb(p, q, h + 1.0f/3.0f);
            r[1] = hue2rgb(p, q, h);
            r[2] = hue2rgb(p, q, h - 1.0f/3.0f);
        }
    }
#endif
}

void pt_gamma_lut_c(float* lut, unsigned int n, float gamma) {
    if (!lut || n == 0 || gamma <= 0) return;
    float inv = 1.0f / gamma;
#ifdef PT_SSE
    if (n >= 8) {
        __m128 invv = _mm_set1_ps(inv);
        __m128 onev = _mm_set1_ps(1.0f);
        for (unsigned int i = 0; i < n; i += 4) {
            __m128 idx = _mm_set_ps((float)(i+3), (float)(i+2), (float)(i+1), (float)i);
            __m128 base = _mm_div_ps(idx, _mm_set1_ps((float)(n - 1)));
            float tmp[4];
            _mm_storeu_ps(tmp, base);
            for (int k = 0; k < 4; ++k) {
                float v = tmp[k];
                lut[i + k] = powf(v, inv);
            }
            (void)invv;(void)onev;
        }
        return;
    }
#endif
    for (unsigned int i = 0; i < n; ++i) lut[i] = powf((float)i / (float)(n - 1), inv);
}

void pt_bilinear_sample_c(const float* img, unsigned int w, unsigned int h, int ch, float x, float y, float* out4) {
    if (!img || !out4) return;
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    float tx = x - x0, ty = y - y0;
    int x1 = x0 + 1, y1 = y0 + 1;
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
    if (x1 > (int)w - 1) x1 = (int)w - 1;
    if (y1 > (int)h - 1) y1 = (int)h - 1;
    for (int c = 0; c < ch; ++c) {
        float p00 = img[((size_t)y0 * w + x0) * ch + c];
        float p10 = img[((size_t)y0 * w + x1) * ch + c];
        float p01 = img[((size_t)y1 * w + x0) * ch + c];
        float p11 = img[((size_t)y1 * w + x1) * ch + c];
        out4[c] = p00 * (1-tx)*(1-ty) + p10 * tx*(1-ty) + p01 * (1-tx)*ty + p11 * tx*ty;
    }
}

static uint32_t pcg_hash(uint32_t s) {
    s ^= s >> 16; s *= 0x7feb352dU; s ^= s >> 15; s *= 0x846ca68bU; s ^= s >> 16;
    return s;
}

static float grad2_c(int h, float x, float y) {
    switch (h & 3) { case 0: return x+y; case 1: return -x+y; case 2: return x-y; default: return -x-y; }
}

static float fade_c(float t) { return t*t*t*(t*(t*6-15)+10); }

void pt_perlin2(float* out, unsigned int w, unsigned int h, float scale, int seed) {
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
    uint8_t ptab[512];
    for (int i = 0; i < 256; ++i) { ptab[i] = perm[i]; ptab[256+i] = perm[i]; }
    if (scale <= 0) scale = 1;
    for (unsigned int y = 0; y < h; ++y) {
        for (unsigned int x = 0; x < w; ++x) {
            float fx = (float)x / scale, fy = (float)y / scale;
            int X = ((int)floorf(fx)) & 255, Y = ((int)floorf(fy)) & 255;
            fx -= floorf(fx); fy -= floorf(fy);
            float u = fade_c(fx), v = fade_c(fy);
            int A = ptab[X] + Y + (seed & 255), AA = ptab[A], AB = ptab[A+1];
            int B = ptab[X+1] + Y + (seed & 255), BA = ptab[B], BB = ptab[B+1];
            float n00 = grad2_c(ptab[AA], fx, fy);
            float n10 = grad2_c(ptab[BA], fx-1, fy);
            float n01 = grad2_c(ptab[AB], fx, fy-1);
            float n11 = grad2_c(ptab[BB], fx-1, fy-1);
            float nx0 = n00 + u*(n10-n00);
            float nx1 = n01 + u*(n11-n01);
            out[(size_t)y*w+x] = (nx0 + v*(nx1-nx0))*0.7f;
        }
    }
}

void pt_simplex2(float* out, unsigned int w, unsigned int h, float scale, int seed) {
    if (scale <= 0) scale = 1;
    const float F2 = 0.36602540378f, G2 = 0.21132486540f;
    static const int grads[12][2] = {{1,1},{-1,1},{1,-1},{-1,-1},{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};
    uint8_t ptab[512];
    for (int i = 0; i < 512; ++i) ptab[i] = (uint8_t)pcg_hash((uint32_t)i ^ (uint32_t)seed);
    for (unsigned int y = 0; y < h; ++y) {
        for (unsigned int x = 0; x < w; ++x) {
            float nx = (float)x/scale, ny = (float)y/scale;
            float s = (nx+ny)*F2;
            int i0 = (int)floorf(nx+s), j0 = (int)floorf(ny+s);
            float t = (i0+j0)*G2;
            float x0 = nx-(i0-t), y0 = ny-(j0-t);
            int i1,j1;
            if(x0>y0){i1=1;j1=0;}else{i1=0;j1=1;}
            float x1=x0-i1+G2,y1=y0-j1+G2;
            float x2=x0-1+2*G2,y2=y0-1+2*G2;
            int ii=i0&255,jj=j0&255;
            int gi0=ptab[ii+ptab[jj]]%12,gi1=ptab[ii+i1+ptab[jj+j1]]%12,gi2=ptab[ii+1+ptab[jj+1]]%12;
            float n0=0,n1=0,n2=0;
            float tt0=0.5f-x0*x0-y0*y0;
            if(tt0>0){tt0*=tt0;n0=tt0*tt0*(grads[gi0][0]*x0+grads[gi0][1]*y0);}
            float tt1=0.5f-x1*x1-y1*y1;
            if(tt1>0){tt1*=tt1;n1=tt1*tt1*(grads[gi1][0]*x1+grads[gi1][1]*y1);}
            float tt2=0.5f-x2*x2-y2*y2;
            if(tt2>0){tt2*=tt2;n2=tt2*tt2*(grads[gi2][0]*x2+grads[gi2][1]*y2);}
            out[(size_t)y*w+x]=70.0f*(n0+n1+n2);
        }
    }
}

void pt_worley2(float* out, unsigned int w, unsigned int h, float scale, int seed, int f2) {
    if (scale <= 0) scale = 1;
    for (unsigned int y = 0; y < h; ++y) {
        for (unsigned int x = 0; x < w; ++x) {
            float nx = (float)x/scale, ny = (float)y/scale;
            int cx=(int)floorf(nx),cy=(int)floorf(ny);
            float d1=1e9f,d2=1e9f;
            for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
                int gx=cx+dx,gy=cy+dy;
                uint32_t hh=pcg_hash(((uint32_t)gx*374761393u)^((uint32_t)gy*668265263u)^(uint32_t)seed);
                float px=gx+((hh&0xffff)/65535.0f);
                uint32_t hh2=pcg_hash(hh^0x9e3779b9u);
                float py=gy+((hh2&0xffff)/65535.0f);
                float dist=sqrtf((px-nx)*(px-nx)+(py-ny)*(py-ny));
                if(dist<d1){d2=d1;d1=dist;}else if(dist<d2)d2=dist;
            }
            out[(size_t)y*w+x]=f2?d2:d1;
        }
    }
}
