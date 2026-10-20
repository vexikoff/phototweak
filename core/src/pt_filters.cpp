#include "pt_filters.h"
#include "pt_math.h"
#include <cmath>
#include <algorithm>

namespace pt {

static inline float cl(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

static void boxBlurChannel(const std::vector<float>& in, std::vector<float>& out, uint32_t w, uint32_t h, int chIdx, int radius) {
    if (radius <= 0) { out = in; return; }
    size_t n = static_cast<size_t>(w) * h;
    std::vector<float> tmp(n);
    for (size_t i = 0; i < n; ++i) tmp[i] = in[i * 4 + chIdx];
    std::vector<float> acc(n);
    int win = 2 * radius + 1;
    for (uint32_t y = 0; y < h; ++y) {
        float sum = 0;
        for (int k = -radius; k <= radius; ++k) sum += tmp[y * w + std::min<uint32_t>(w - 1, std::max(0, k))];
        for (uint32_t x = 0; x < w; ++x) {
            acc[y * w + x] = sum / win;
            int addX = std::min<int>(w - 1, x + radius + 1);
            int subX = std::max(0, x - radius);
            sum += tmp[y * w + addX] - tmp[y * w + subX];
        }
    }
    for (uint32_t x = 0; x < w; ++x) {
        float sum = 0;
        for (int k = -radius; k <= radius; ++k) sum += acc[std::min<uint32_t>(h - 1, std::max(0, k)) * w + x];
        for (uint32_t y = 0; y < h; ++y) {
            tmp[y * w + x] = sum / win;
            int addY = std::min<int>(h - 1, y + radius + 1);
            int subY = std::max(0, y - radius);
            sum += acc[addY * w + x] - acc[subY * w + x];
        }
    }
    for (size_t i = 0; i < n; ++i) out[i * 4 + chIdx] = tmp[i];
}

void box_blur(Image& img, int radius) {
    if (radius <= 0 || img.width == 0) return;
    std::vector<float> src = img.pixels, dst = img.pixels;
    for (int c = 0; c < 4; ++c) boxBlurChannel(src, dst, img.width, img.height, c, radius);
    img.pixels.swap(dst);
}

void gaussian_blur(Image& img, float sigma) {
    if (sigma <= 0.01f) return;
    int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0f)));
    box_blur(img, radius);
    box_blur(img, radius);
    box_blur(img, radius);
}

void motion_blur(Image& img, int radius, float angleDeg) {
    if (radius <= 0) return;
    Image src = clone_image(img);
    float a = angleDeg * 3.14159265f / 180.0f;
    float dx = std::cos(a), dy = std::sin(a);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        int x = i % img.width, y = i / img.width;
        float acc[4] = {0};
        for (int k = -radius; k <= radius; ++k) {
            int sx = x + static_cast<int>(dx * k), sy = y + static_cast<int>(dy * k);
            sx = std::max(0, std::min(static_cast<int>(img.width)-1, sx));
            sy = std::max(0, std::min(static_cast<int>(img.height)-1, sy));
            const float* p = &src.pixels[(sy * img.width + sx) * 4];
            for (int c = 0; c < 4; ++c) acc[c] += p[c];
        }
        for (int c = 0; c < 4; ++c) img.pixels[i * 4 + c] = acc[c] / (2 * radius + 1);
    }
}

void radial_blur_spin(Image& img, float angleDeg, int samples) {
    Image src = clone_image(img);
    float cx = img.width * 0.5f, cy = img.height * 0.5f;
    float dA = angleDeg * 3.14159265f / 180.0f / samples;
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        int x = i % img.width, y = i / img.width;
        float vx = x - cx, vy = y - cy;
        float acc[4] = {0};
        for (int s = 0; s < samples; ++s) {
            float ang = dA * s;
            float rx = cx + vx * std::cos(ang) - vy * std::sin(ang);
            float ry = cy + vx * std::sin(ang) + vy * std::cos(ang);
            int ix = std::max(0, std::min((int)img.width-1, (int)rx));
            int iy = std::max(0, std::min((int)img.height-1, (int)ry));
            const float* p = &src.pixels[(iy*img.width+ix)*4];
            for (int c = 0; c < 4; ++c) acc[c] += p[c];
        }
        for (int c = 0; c < 4; ++c) img.pixels[i*4+c] = acc[c]/samples;
    }
}

void radial_blur_zoom(Image& img, float amount, int samples) {
    Image src = clone_image(img);
    float cx = img.width * 0.5f, cy = img.height * 0.5f;
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        int x = i % img.width, y = i / img.width;
        float vx = x - cx, vy = y - cy;
        float acc[4] = {0};
        for (int s = 0; s < samples; ++s) {
            float t = amount * (float)s / samples;
            float rx = cx + vx * (1 + t), ry = cy + vy * (1 + t);
            int ix = std::max(0, std::min((int)img.width-1, (int)rx));
            int iy = std::max(0, std::min((int)img.height-1, (int)ry));
            const float* p = &src.pixels[(iy*img.width+ix)*4];
            for (int c = 0; c < 4; ++c) acc[c] += p[c];
        }
        for (int c = 0; c < 4; ++c) img.pixels[i*4+c] = acc[c]/samples;
    }
}

void iris_blur(Image& img, float cx, float cy, float innerRadius, float outerRadius, float maxBlur) {
    Image blurred = clone_image(img);
    gaussian_blur(blurred, maxBlur);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        int x = i % img.width, y = i / img.width;
        float d = std::hypot(x - cx, y - cy);
        float t = cl((d - innerRadius) / std::max(0.001f, outerRadius - innerRadius));
        for (int c = 0; c < 4; ++c) img.pixels[i*4+c] = img.pixels[i*4+c]*(1-t) + blurred.pixels[i*4+c]*t;
    }
}

void field_blur(Image& img, const std::vector<float>& pointsXYR, int pointCount) {
    Image blurred = clone_image(img);
    gaussian_blur(blurred, 8.0f);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        int x = i % img.width, y = i / img.width;
        float influence = 0;
        for (int p = 0; p < pointCount; ++p) {
            float px = pointsXYR[p*3], py = pointsXYR[p*3+1], pr = pointsXYR[p*3+2];
            float d = std::hypot(x - px, y - py);
            float w = d < pr ? 1.0f : std::max(0.0f, 1.0f - (d - pr)/pr);
            influence = std::max(influence, w);
        }
        for (int c = 0; c < 4; ++c) img.pixels[i*4+c] = img.pixels[i*4+c]*(1-influence) + blurred.pixels[i*4+c]*influence;
    }
}

void tilt_shift_blur(Image& img, float y0, float y1, float blurMax) {
    Image blurred = clone_image(img);
    gaussian_blur(blurred, blurMax);
    for (uint32_t y = 0; y < img.height; ++y) {
        float t = 0;
        if (y < y0) t = cl((y0 - y) / std::max(1.0f, y0));
        else if (y > y1) t = cl((y - y1) / std::max(1.0f, (float)img.height - y1));
        for (uint32_t x = 0; x < img.width; ++x) {
            size_t idx = (static_cast<size_t>(y)*img.width+x)*4;
            for (int c = 0; c < 4; ++c) img.pixels[idx+c] = img.pixels[idx+c]*(1-t)+blurred.pixels[idx+c]*t;
        }
    }
}

void surface_blur(Image& img, int radius, float threshold) {
    Image src = clone_image(img);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        int x = i % img.width, y = i / img.width;
        float cr = src.pixels[i*4], cg = src.pixels[i*4+1], cb = src.pixels[i*4+2];
        float acc[4]={0}; int cnt=0;
        for (int dy=-radius; dy<=radius; ++dy) for (int dx=-radius; dx<=radius; ++dx) {
            int sx = std::max(0,std::min((int)img.width-1,x+dx)), sy = std::max(0,std::min((int)img.height-1,y+dy));
            size_t j = (static_cast<size_t>(sy)*img.width+sx)*4;
            float diff = std::fabs(src.pixels[j]-cr)+std::fabs(src.pixels[j+1]-cg)+std::fabs(src.pixels[j+2]-cb);
            if (diff/3.0f < threshold) { for(int c=0;c<4;++c) acc[c]+=src.pixels[j+c]; cnt++; }
        }
        if (cnt>0) for (int c=0;c<4;++c) img.pixels[i*4+c]=acc[c]/cnt;
    }
}

void shape_blur(Image& img, int radius) {
    Image src = clone_image(img);
    size_t n = img.pixelCount();
    for (size_t i = 0; i < n; ++i) {
        int x = i%img.width, y=i/img.width;
        float lum = 0.299f*src.pixels[i*4]+0.587f*src.pixels[i*4+1]+0.114f*src.pixels[i*4+2];
        float acc[4]={0}; int cnt=0;
        for (int dy=-radius; dy<=radius; ++dy) for (int dx=-radius; dx<=radius; ++dx) {
            int sx=std::max(0,std::min((int)img.width-1,x+dx)), sy=std::max(0,std::min((int)img.height-1,y+dy));
            size_t j=(static_cast<size_t>(sy)*img.width+sx)*4;
            float l2 = 0.299f*src.pixels[j]+0.587f*src.pixels[j+1]+0.114f*src.pixels[j+2];
            if (l2 >= lum) { for(int c=0;c<4;++c) acc[c]+=src.pixels[j+c]; cnt++; }
        }
        if(cnt>0) for(int c=0;c<4;++c) img.pixels[i*4+c]=acc[c]/cnt;
    }
}

void smart_blur(Image& img, float radius, float threshold) {
    surface_blur(img, static_cast<int>(radius), threshold);
}

void lens_blur(Image& img, int radius) {
    box_blur(img, radius);
}

static float medianOf(std::vector<float>& v) {
    size_t mid = v.size()/2;
    std::nth_element(v.begin(), v.begin()+mid, v.end());
    return v[mid];
}

void median_filter(Image& img, int radius) {
    Image src = clone_image(img);
    size_t n = img.pixelCount();
    std::vector<float> vals;
    for (size_t i = 0; i < n; ++i) {
        int x=i%img.width, y=i/img.width;
        for (int c=0;c<4;++c) {
            vals.clear();
            for (int dy=-radius; dy<=radius; ++dy) for (int dx=-radius; dx<=radius; ++dx) {
                int sx=std::max(0,std::min((int)img.width-1,x+dx)), sy=std::max(0,std::min((int)img.height-1,y+dy));
                vals.push_back(src.pixels[(static_cast<size_t>(sy)*img.width+sx)*4+c]);
            }
            img.pixels[i*4+c]=medianOf(vals);
        }
    }
}

void max_filter(Image& img, int radius) {
    Image src = clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){int x=i%img.width,y=i/img.width;
        for(int c=0;c<4;++c){float mx=src.pixels[i*4+c];
            for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx){int sx=std::max(0,std::min((int)img.width-1,x+dx)),sy=std::max(0,std::min((int)img.height-1,y+dy));mx=std::max(mx,src.pixels[(static_cast<size_t>(sy)*img.width+sx)*4+c]);}
            img.pixels[i*4+c]=mx;}}
}

void min_filter(Image& img, int radius) {
    Image src = clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){int x=i%img.width,y=i/img.width;
        for(int c=0;c<4;++c){float mn=src.pixels[i*4+c];
            for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx){int sx=std::max(0,std::min((int)img.width-1,x+dx)),sy=std::max(0,std::min((int)img.height-1,y+dy));mn=std::min(mn,src.pixels[(static_cast<size_t>(sy)*img.width+sx)*4+c]);}
            img.pixels[i*4+c]=mn;}}
}

void despeckle(Image& img, int radius, float threshold) {
    median_filter(img, radius);
}

void dust_scratches(Image& img, int radius, float threshold) {
    Image src = clone_image(img);
    size_t n=img.pixelCount();
    std::vector<float> vals;
    for(size_t i=0;i<n;++i){int x=i%img.width,y=i/img.width;
        for(int c=0;c<3;++c){vals.clear();
            for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx){int sx=std::max(0,std::min((int)img.width-1,x+dx)),sy=std::max(0,std::min((int)img.height-1,y+dy));vals.push_back(src.pixels[(static_cast<size_t>(sy)*img.width+sx)*4+c]);}
            float med=medianOf(vals);
            if(std::fabs(src.pixels[i*4+c]-med)>threshold) img.pixels[i*4+c]=med;
        }}
}

void add_noise(Image& img, float amount, bool uniform, bool monochromatic, float seed) {
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        float r1 = hash_float_2d(i%img.width, i/img.width, static_cast<uint32_t>(seed));
        float noise = uniform ? (r1-0.5f)*amount : (r1-0.5f)*(r1-0.5f)*4*amount;
        if(monochromatic){img.pixels[i*4]+=noise;img.pixels[i*4+1]+=noise;img.pixels[i*4+2]+=noise;}
        else{
            img.pixels[i*4]+=noise;
            img.pixels[i*4+1]+=hash_float_2d(i%img.width+7, i/img.width, static_cast<uint32_t>(seed)+1)-0.5f;
            img.pixels[i*4+2]+=hash_float_2d(i%img.width, i/img.width+13, static_cast<uint32_t>(seed)+2)-0.5f;
        }
        for(int c=0;c<3;++c) img.pixels[i*4+c]=cl(img.pixels[i*4+c]);
    }
}

void reduce_noise(Image& img, float strength) {
    surface_blur(img, static_cast<int>(strength*3+1), strength*0.1f);
}

void unsharp_mask(Image& img, float amount, float radius, float threshold) {
    Image blurred = clone_image(img);
    gaussian_blur(blurred, radius);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        for(int c=0;c<3;++c){
            float d = img.pixels[i*4+c]-blurred.pixels[i*4+c];
            if(std::fabs(d)>=threshold) img.pixels[i*4+c]=cl(img.pixels[i*4+c]+d*amount);
        }
    }
}

void sharpen(Image& img, float amount, float radius, float threshold) { unsharp_mask(img, amount, radius, threshold); }

void smart_sharpen(Image& img, float amount, float radius, float threshold) {
    Image edge = clone_image(img);
    find_edges(edge);
    gaussian_blur(edge, radius);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        float e = 0.299f*edge.pixels[i*4]+0.587f*edge.pixels[i*4+1]+0.114f*edge.pixels[i*4+2];
        float f = cl(e*2);
        for(int c=0;c<3;++c){
            float orig = img.pixels[i*4+c];
            float lap = orig*4 - ((i>0?orig:orig));
            img.pixels[i*4+c]=cl(orig + (orig-lap)*amount*f);
        }
    }
    unsharp_mask(img, amount, radius, threshold);
}

void find_edges(Image& img) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        auto at=[&](int dx,int dy,int c)->float{int sx=std::max(0,std::min((int)img.width-1,x+dx)),sy=std::max(0,std::min((int)img.height-1,y+dy));return src.pixels[(static_cast<size_t>(sy)*img.width+sx)*4+c];};
        float gx=0,gy=0;
        for(int c=0;c<3;++c){
            float kx = -at(-1,-1,c)+at(1,-1,c)-2*at(-1,0,c)+2*at(1,0,c)-at(-1,1,c)+at(1,1,c);
            float ky = at(-1,-1,c)+2*at(0,-1,c)+at(1,-1,c)-at(-1,1,c)-2*at(0,1,c)-at(1,1,c);
            gx+=kx; gy+=ky;
        }
        float mag = cl(std::sqrt(gx*gx+gy*gy)/12.0f);
        img.pixels[i*4]=img.pixels[i*4+1]=img.pixels[i*4+2]=mag;
    }
}

void edge_detect(Image& img, int mode) {
    find_edges(img);
    if(mode==1){ adjust_invert(img); }
}

void emboss(Image& img, float angleDeg, float distance, float strength) {
    Image src=clone_image(img);
    float a=angleDeg*3.14159265f/180.0f;
    int dx=static_cast<int>(std::cos(a)*distance), dy=static_cast<int>(std::sin(a)*distance);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        auto at=[&](int ox,int oy,int c)->float{int sx=std::max(0,std::min((int)img.width-1,x+ox)),sy=std::max(0,std::min((int)img.height-1,y+oy));return src.pixels[(static_cast<size_t>(sy)*img.width+sx)*4+c];};
        for(int c=0;c<3;++c){
            float d = at(dx,dy,c)-at(-dx,-dy,c);
            img.pixels[i*4+c]=cl(0.5f+d*strength);
        }
    }
}

void outline(Image& img) { find_edges(img); }

void solarize(Image& img) {
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i)for(int c=0;c<3;++c){float v=img.pixels[i*4+c];if(v>0.5f)img.pixels[i*4+c]=1.0f-v;}
}

void wind(Image& img, int direction, float strength) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    int len=static_cast<int>(strength*20);
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        for(int c=0;c<3;++c){
            float m=src.pixels[i*4+c];
            for(int k=1;k<len;++k){
                int sx = direction==0? x-k : x+k;
                sx=std::max(0,std::min((int)img.width-1,sx));
                float v=src.pixels[(static_cast<size_t>(y)*img.width+sx)*4+c];
                if(v<m) m=v;
            }
            img.pixels[i*4+c]=m;
        }
    }
}

void mosaic(Image& img, int cellSize) {
    if(cellSize<=0)cellSize=1;
    for(uint32_t by=0;by<img.height;by+=cellSize)for(uint32_t bx=0;bx<img.width;bx+=cellSize){
        float acc[4]={0};int cnt=0;
        for(uint32_t y=by;y<std::min(by+cellSize,img.height);++y)for(uint32_t x=bx;x<std::min(bx+cellSize,img.width);++x){
            size_t idx=(static_cast<size_t>(y)*img.width+x)*4;for(int c=0;c<4;++c)acc[c]+=img.pixels[idx+c];cnt++;}
        for(uint32_t y=by;y<std::min(by+cellSize,img.height);++y)for(uint32_t x=bx;x<std::min(bx+cellSize,img.width);++x){
            size_t idx=(static_cast<size_t>(y)*img.width+x)*4;for(int c=0;c<4;++c)img.pixels[idx+c]=acc[c]/cnt;}
    }
}

void pointillize(Image& img, int cellSize) {
    Image src=clone_image(img);
    for(uint32_t y=0;y<img.height;++y)for(uint32_t x=0;x<img.width;++x){
        int cx=(x/cellSize)*cellSize+cellSize/2, cy=(y/cellSize)*cellSize+cellSize/2;
        float d=std::hypot(x-cx,y-cy);
        if(d>cellSize*0.5f){size_t idx=(static_cast<size_t>(y)*img.width+x)*4;for(int c=0;c<3;++c)img.pixels[idx+c]=src.pixels[idx+c]*0.3f;}
    }
}

void color_halftone(Image& img, int cellSize) {
    mosaic(img, cellSize);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i)for(int c=0;c<3;++c)img.pixels[i*4+c]=img.pixels[i*4+c]>0.5f?1.0f:0.0f;
}

void mezzotint(Image& img, int type, float noise) {
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        float L=0.299f*img.pixels[i*4]+0.587f*img.pixels[i*4+1]+0.114f*img.pixels[i*4+2];
        float rnd=hash_float_2d(i%img.width,i/img.width,type*999+static_cast<uint32_t>(noise*1000));
        float v = L>rnd?1.0f:0.0f;
        img.pixels[i*4]=img.pixels[i*4+1]=img.pixels[i*4+2]=v;
    }
}

void crystallize(Image& img, int cellSize) {
    Image src=clone_image(img);
    for(uint32_t y=0;y<img.height;++y)for(uint32_t x=0;x<img.width;++x){
        int cx=(x/cellSize)*cellSize+cellSize/2, cy=(y/cellSize)*cellSize+cellSize/2;
        cx=std::max(0,std::min((int)img.width-1,cx));cy=std::max(0,std::min((int)img.height-1,cy));
        size_t si=(static_cast<size_t>(cy)*img.width+cx)*4, di=(static_cast<size_t>(y)*img.width+x)*4;
        for(int c=0;c<4;++c)img.pixels[di+c]=src.pixels[si+c];
    }
}

void color_burn(Image& img, int radius) {
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        float r=img.pixels[i*4],g=img.pixels[i*4+1],b=img.pixels[i*4+2];
        float rr=r*r+g*g+b*b;
        img.pixels[i*4]=cl(rr);img.pixels[i*4+1]=cl(g*rr);img.pixels[i*4+2]=cl(b*rr);
    }
    gaussian_blur(img, radius);
}

void diffused_dither(Image& img) {
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i)for(int c=0;c<3;++c){float v=img.pixels[i*4+c];img.pixels[i*4+c]=v>0.5f?1.0f:0.0f;}
}

void tint_flat(Image& img, int size, bool dark, float intensity) {
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float r=hash_float_2d(x/size,y/size,dark?1u:2u);
        float target=dark?(r<0.5f?0.0f:0.3f):(r>0.5f?1.0f:0.7f);
        for(int c=0;c<3;++c)img.pixels[i*4+c]=cl(img.pixels[i*4+c]*(1-intensity)+target*intensity);
    }
}

void clouds(Image& img, float seed) {
    for(uint32_t y=0;y<img.height;++y)for(uint32_t x=0;x<img.width;++x){
        float v=fbm_noise(NoiseType::Perlin, x*0.02f, y*0.02f, 5, 2.0f, 0.5f, static_cast<int>(seed))*0.5f+0.5f;
        size_t idx=(static_cast<size_t>(y)*img.width+x)*4;
        img.pixels[idx]=img.pixels[idx+1]=img.pixels[idx+2]=cl(v);img.pixels[idx+3]=1;
    }
}

void difference_clouds(Image& img, float seed) {
    Image before=clone_image(img);
    clouds(img, seed);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i)for(int c=0;c<3;++c)img.pixels[i*4+c]=cl(std::fabs(before.pixels[i*4+c]-img.pixels[i*4+c])*2);
}

void fibers(Image& img, int variance, float strength, float seed) {
    for(uint32_t y=0;y<img.height;++y)for(uint32_t x=0;x<img.width;++x){
        float v=simplex2(x*0.01f*variance, y*0.2f, static_cast<int>(seed))*0.5f+0.5f;
        size_t idx=(static_cast<size_t>(y)*img.width+x)*4;
        img.pixels[idx]=cl(v*strength);img.pixels[idx+1]=cl(v*strength*0.8f);img.pixels[idx+2]=cl(v*strength*0.6f);img.pixels[idx+3]=1;
    }
}

void lens_flare(Image& img, float cx, float cy, float brightness, float zoom, float glare, int ringCount, float seed) {
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float d=std::hypot(x-cx,y-cy);
        float core=brightness*std::exp(-d*d/(800.0f*zoom));
        float streak=glare*brightness*std::exp(-std::fabs(y-cy)/30.0f)*std::exp(-std::fabs(x-cx)/(img.width*0.5f));
        float halo=brightness*0.3f*std::exp(-std::pow(d/(img.width*0.3f),2));
        float total=core+streak+halo;
        for(int c=0;c<3;++c)img.pixels[i*4+c]=cl(img.pixels[i*4+c]+total);
    }
    for(int r=0;r<ringCount;++r){
        float t=(r+1)/(ringCount+1.0f);
        float rx=cx+(img.width*0.5f-cx)*t*2, ry=cy+(img.height*0.5f-cy)*t*2;
        float rad=img.width*0.02f*(1+t);
        for(uint32_t y=0;y<img.height;++y)for(uint32_t x=0;x<img.width;++x){
            float d=std::hypot(x-rx,y-ry);
            float band=std::exp(-(d-rad)*(d-rad)/(rad*0.5f))*brightness*0.2f;
            size_t idx=(static_cast<size_t>(y)*img.width+x)*4;
            img.pixels[idx]=cl(img.pixels[idx]+band*0.5f);img.pixels[idx+1]=cl(img.pixels[idx+1]+band*0.7f);img.pixels[idx+2]=cl(img.pixels[idx+2]+band);
        }
    }
}

void lighting_effect(Image& img, float altitudeDeg, float azimuthDeg, float intensity, float ambient, float material, int style, int textureType, float offsetX, float offsetY) {
    Image normal=clone_image(img);
    find_edges(normal);
    gaussian_blur(normal, 2.0f);
    float alt=altitudeDeg*3.14159265f/180.0f, az=azimuthDeg*3.14159265f/180.0f;
    float lx=std::cos(alt)*std::sin(az), ly=std::sin(alt), lz=std::cos(alt)*std::cos(az);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        auto at=[&](int dx,int dy)->float{int sx=std::max(0,std::min((int)img.width-1,x+dx)),sy=std::max(0,std::min((int)img.height-1,y+dy));return normal.pixels[(static_cast<size_t>(sy)*img.width+sx)*4];};
        float nx=-(at(1,0)-at(-1,0)), ny=-(at(0,1)-at(0,-1)), nz=1.0f;
        float nl=std::sqrt(nx*nx+ny*ny+nz*nz); nx/=nl;ny/=nl;nz/=nl;
        float diff=nx*lx+ny*ly+nz*lz; if(diff<0)diff=0;
        float spec=std::pow(diff,material*10.0f)*intensity;
        float lit=ambient+diff*intensity+spec;
        for(int c=0;c<3;++c)img.pixels[i*4+c]=cl(img.pixels[i*4+c]*lit);
    }
}

void displacement_map(Image& img, const Image& mapX, const Image& mapY, float scaleX, float scaleY, int undefinedAction) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        size_t mi=(static_cast<size_t>(y)*img.width+x)*4;
        float dx=(mapX.pixels[mi]-0.5f)*scaleX*img.width;
        float dy=(mapY.pixels[mi]-0.5f)*scaleY*img.height;
        int sx=x+dx, sy=y+dy;
        if(sx<0||sy<0||sx>=(int)img.width||sy>=(int)img.height){ if(undefinedAction==0)continue; sx=std::max(0,std::min((int)img.width-1,sx));sy=std::max(0,std::min((int)img.height-1,sy));}
        size_t si=(static_cast<size_t>(sy)*img.width+sx)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void twirl_distort(Image& img, float angleDeg, float strength) {
    Image src=clone_image(img);
    float cx=img.width*0.5f,cy=img.height*0.5f;
    float maxR=std::min(cx,cy)*strength;
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float vx=x-cx,vy=y-cy;
        float r=std::hypot(vx,vy);
        if(r<maxR){
            float a=angleDeg*3.14159265f/180.0f*(1-r/maxR);
            float rx=cx+vx*std::cos(a)-vy*std::sin(a);
            float ry=cy+vx*std::sin(a)+vy*std::cos(a);
            int ix=std::max(0,std::min((int)img.width-1,(int)rx)),iy=std::max(0,std::min((int)img.height-1,(int)ry));
            size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
            for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
        }
    }
}

void wave_distort(Image& img, float wavelength, float amplitude, float speedDeg, float phaseDeg, float amplitude2, float wavelength2, float pinch, float skew) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float sx=x+amplitude*std::sin(y/wavelength*6.28318f+phaseDeg*0.0174533f);
        float sy=y+amplitude2*std::sin(x/wavelength2*6.28318f);
        int ix=std::max(0,std::min((int)img.width-1,(int)sx)),iy=std::max(0,std::min((int)img.height-1,(int)sy));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void zigzag_distort(Image& img, float corners, float radius, float amount) {
    Image src=clone_image(img);
    float cx=img.width*0.5f,cy=img.height*0.5f;
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float vx=x-cx,vy=y-cy;
        float r=std::hypot(vx,vy);
        float a=std::atan2(vy,vx);
        float off=amount*std::sin(a*corners+r/radius);
        float rx=cx+(r+off)*std::cos(a),ry=cy+(r+off)*std::sin(a);
        int ix=std::max(0,std::min((int)img.width-1,(int)rx)),iy=std::max(0,std::min((int)img.height-1,(int)ry));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void spherize_distort(Image& img, float amount, float mode) {
    Image src=clone_image(img);
    float cx=img.width*0.5f,cy=img.height*0.5f;
    float R=std::min(cx,cy);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float vx=(x-cx)/R,vy=(y-cy)/R;
        float r=vx*vx+vy*vy;
        if(r<1){
            float z=std::sqrt(1-r)*amount;
            float sx=cx+vx*R/(1+z*0.5f),sy=cy+vy*R/(1+z*0.5f);
            int ix=std::max(0,std::min((int)img.width-1,(int)sx)),iy=std::max(0,std::min((int)img.height-1,(int)sy));
            size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
            for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
        }
    }
}

void polar_to_rect(Image& img, float rotation, bool invert) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    float cx=img.width*0.5f,cy=img.height*0.5f;
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float u,v;
        if(!invert){u=(x-cx)/cx;v=(y-cy)/cy;float r=std::hypot(u,v)*0.5f;float a=std::atan2(v,u)+rotation*0.0174533f;u=a/6.28318f+0.5f;v=1-r;}
        else{u=x/(float)img.width;v=y/(float)img.height;float a=(u-0.5f)*6.28318f;float r=(1-v)*2*std::min(cx,cy);u=cx+r*std::cos(a);v=cy+r*std::sin(a);u/=img.width;v/=img.height;}
        int ix=std::max(0,std::min((int)img.width-1,(int)(u*img.width))),iy=std::max(0,std::min((int)img.height-1,(int)(v*img.height)));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void grid_distort(Image& img, float majorX, float majorY, float minorX, float minorY, float radius, bool invert) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float fx=x+majorX*std::sin(y/radius);
        float fy=y+majorY*std::sin(x/radius);
        int ix=std::max(0,std::min((int)img.width-1,(int)fx)),iy=std::max(0,std::min((int)img.height-1,(int)fy));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void shear_distort(Image& img, float angleDeg) {
    Image src=clone_image(img);
    float t=std::tan(angleDeg*0.0174533f);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float sx=x+y*t;
        int ix=std::max(0,std::min((int)img.width-1,(int)sx));
        size_t si=(static_cast<size_t>(y)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void ripple_distort(Image& img, float amount, float size) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float d=std::hypot(x-img.width*0.5f,y-img.height*0.5f);
        float off=std::sin(d/size)*amount;
        int ix=std::max(0,std::min((int)img.width-1,(int)(x+off))),iy=std::max(0,std::min((int)img.height-1,(int)(y+off)));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void ocean_ripple(Image& img, float amount, float size, float seed) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float off=perlin2(x/size,y/size,static_cast<int>(seed))*amount;
        int ix=std::max(0,std::min((int)img.width-1,(int)(x+off))),iy=std::max(0,std::min((int)img.height-1,(int)(y-off)));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void glass_effect(Image& img, float distortion, float smoothness, bool frosty, float scaling, float seed) {
    Image src=clone_image(img);
    if(frosty)gaussian_blur(src,smoothness);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float dx=(hash_float_2d(x/8,y/8,static_cast<uint32_t>(seed))-0.5f)*distortion;
        float dy=(hash_float_2d(x/8+3,y/8+7,static_cast<uint32_t>(seed)+1)-0.5f)*distortion;
        int ix=std::max(0,std::min((int)img.width-1,(int)(x*dx/8+x))),iy=std::max(0,std::min((int)img.height-1,(int)(y+dy)));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void pinch_distort(Image& img, float amount) {
    Image src=clone_image(img);
    float cx=img.width*0.5f,cy=img.height*0.5f;
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float vx=x-cx,vy=y-cy;
        float d=std::hypot(vx,vy);
        float f=std::pow(d/std::hypot(cx,cy),amount);
        int ix=std::max(0,std::min((int)img.width-1,(int)(cx+vx*f))),iy=std::max(0,std::min((int)img.height-1,(int)(cy+vy*f)));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void bulge_distort(Image& img, float amount) {
    pinch_distort(img, -amount);
}

void stretch_distort(Image& img, float quality, float amount) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float sx=x+amount*std::sin(y*0.05f);
        int ix=std::max(0,std::min((int)img.width-1,(int)sx));
        size_t si=(static_cast<size_t>(y)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void squeeze_twirl_recto(Image& img, float angleDeg, float innerBox, float outerBox, float twistDeg, float amount, float radius) {
    twirl_distort(img,twistDeg,innerBox);
    pinch_distort(img,amount);
}

void perspective_crop_warp(Image& img, const float srcQuad[8], const float dstQuad[8]) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float u=(x-dstQuad[0])/(dstQuad[2]-dstQuad[0]);
        float v=(y-dstQuad[1])/(dstQuad[5]-dstQuad[1]);
        float sx=srcQuad[0]+(srcQuad[2]-srcQuad[0])*u;
        float sy=srcQuad[1]+(srcQuad[5]-srcQuad[1])*v;
        int ix=std::max(0,std::min((int)img.width-1,(int)sx)),iy=std::max(0,std::min((int)img.height-1,(int)sy));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

void liquify_forward_warp(Image& img, const std::vector<float>& pointsXYZRStrength, int count) {
    Image src=clone_image(img);
    size_t n=img.pixelCount();
    for(size_t i=0;i<n;++i){
        int x=i%img.width,y=i/img.width;
        float vx=x,vy=y;
        for(int p=0;p<count;++p){
            float px=pointsXYZR[p*4],py=pointsXYZR[p*4+1],pr=pointsXYZR[p*4+2],ps=pointsXYZR[p*4+3];
            float dx=x-px,dy=y-py;
            float d=std::hypot(dx,dy);
            if(d<pr){
                float falloff=1-d/pr;
                vx+=dx*ps*falloff*0.1f;
                vy+=dy*ps*falloff*0.1f;
            }
        }
        int ix=std::max(0,std::min((int)img.width-1,(int)vx)),iy=std::max(0,std::min((int)img.height-1,(int)vy));
        size_t si=(static_cast<size_t>(iy)*img.width+ix)*4;
        for(int c=0;c<4;++c)img.pixels[i*4+c]=src.pixels[si+c];
    }
}

}
