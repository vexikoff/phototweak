#include "pt_transform.h"
#include <cmath>
#include <algorithm>

namespace pt {

static inline float cl(float v){return v<0?0:(v>1?1:v);}

static void sampleBilinear(const Image& src, float x, float y, float out[4]) {
    if (src.width == 0) { out[0]=out[1]=out[2]=out[3]=0; return; }
    float fx = x, fy = y;
    int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
    int x1 = x0 + 1, y1 = y0 + 1;
    float tx = fx - x0, ty = fy - y0;
    auto at = [&](int xx, int yy)->const float*{
        xx = std::max(0, std::min((int)src.width-1, xx));
        yy = std::max(0, std::min((int)src.height-1, yy));
        return &src.pixels[(static_cast<size_t>(yy)*src.width+xx)*4];
    };
    const float* p00 = at(x0,y0); const float* p10 = at(x1,y0);
    const float* p01 = at(x0,y1); const float* p11 = at(x1,y1);
    for(int c=0;c<4;++c) out[c] = p00[c]*(1-tx)*(1-ty)+p10[c]*tx*(1-ty)+p01[c]*(1-tx)*ty+p11[c]*tx*ty;
}

void transform_affine(const Image& src, Image& dst, float m00, float m01, float m02, float m10, float m11, float m12) {
    float det = m00*m11 - m01*m10;
    if (std::fabs(det) < 1e-8f) { dst = Image(); return; }
    float inv[4] = { m11/det, -m01/det, (m01*m12-m11*m02)/det, -m10/det, m00/det, (m10*m02-m00*m12)/det };
    uint32_t w = src.width, h = src.height;
    dst = create_image(w, h, ChannelKind::RGB, 0.0f);
    for (uint32_t y=0;y<h;++y) for(uint32_t x=0;x<w;++x){
        float sx = inv[0]*x + inv[1]*y + inv[2];
        float sy = inv[3]*x + inv[4]*y + inv[5];
        if (sx<-1||sy<-1||sx>(int)w||sy>(int)h) continue;
        float out[4]; sampleBilinear(src,sx,sy,out);
        size_t idx=(static_cast<size_t>(y)*w+x)*4;
        for(int c=0;c<4;++c) dst.pixels[idx+c]=out[c];
    }
}

void transform_scale(const Image& src, Image& dst, float sx, float sy) {
    uint32_t w = static_cast<uint32_t>(src.width*sx), h = static_cast<uint32_t>(src.height*sy);
    if (w==0||h==0) { dst=Image(); return; }
    dst = create_image(w,h,ChannelKind::RGB,0.0f);
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float out[4];sampleBilinear(src,x/sx,y/sy,out);
        size_t idx=(static_cast<size_t>(y)*w+x)*4;for(int c=0;c<4;++c)dst.pixels[idx+c]=out[c];
    }
}

void transform_rotate(const Image& src, Image& dst, float angleDeg) {
    float a=angleDeg*3.14159265f/180.0f;
    float cs=std::cos(a), sn=std::sin(a);
    float cx=src.width*0.5f, cy=src.height*0.5f;
    transform_affine(src,dst,cs,-sn,cx-cs*cx+sn*cy,sn,cs,cy-sn*cx-cs*cy);
}

void transform_resize(const Image& src, Image& dst, uint32_t w, uint32_t h, bool bicubic) {
    if(w==0||h==0){dst=Image();return;}
    dst=create_image(w,h,ChannelKind::RGB,0.0f);
    float rx=(float)src.width/w, ry=(float)src.height/h;
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float out[4];sampleBilinear(src,x*rx,y*ry,out);
        size_t idx=(static_cast<size_t>(y)*w+x)*4;for(int c=0;c<4;++c)dst.pixels[idx+c]=out[c];
    }
}

void transform_flip_h(const Image& src, Image& dst) {
    dst=create_image(src.width,src.height,ChannelKind::RGB,0.0f);
    for(uint32_t y=0;y<src.height;++y)for(uint32_t x=0;x<src.width;++x){
        size_t si=(static_cast<size_t>(y)*src.width+(src.width-1-x))*4, di=(static_cast<size_t>(y)*src.width+x)*4;
        for(int c=0;c<4;++c)dst.pixels[di+c]=src.pixels[si+c];
    }
}

void transform_flip_v(const Image& src, Image& dst) {
    dst=create_image(src.width,src.height,ChannelKind::RGB,0.0f);
    for(uint32_t y=0;y<src.height;++y)for(uint32_t x=0;x<src.width;++x){
        size_t si=(static_cast<size_t>(src.height-1-y)*src.width+x)*4, di=(static_cast<size_t>(y)*src.width+x)*4;
        for(int c=0;c<4;++c)dst.pixels[di+c]=src.pixels[si+c];
    }
}

void transform_rotate_90cw(const Image& src, Image& dst) {
    dst=create_image(src.height,src.width,ChannelKind::RGB,0.0f);
    for(uint32_t y=0;y<src.height;++y)for(uint32_t x=0;x<src.width;++x){
        size_t si=(static_cast<size_t>(y)*src.width+x)*4;
        size_t di=(static_cast<size_t>(src.width-1-x)*src.height+y)*4;
        for(int c=0;c<4;++c)dst.pixels[di+c]=src.pixels[si+c];
    }
}

void transform_rotate_90ccw(const Image& src, Image& dst) {
    dst=create_image(src.height,src.width,ChannelKind::RGB,0.0f);
    for(uint32_t y=0;y<src.height;++y)for(uint32_t x=0;x<src.width;++x){
        size_t si=(static_cast<size_t>(y)*src.width+x)*4;
        size_t di=(static_cast<size_t>(x)*src.height+(src.height-1-y))*4;
        for(int c=0;c<4;++c)dst.pixels[di+c]=src.pixels[si+c];
    }
}

void transform_rotate_180(const Image& src, Image& dst) {
    dst=create_image(src.width,src.height,ChannelKind::RGB,0.0f);
    for(uint32_t y=0;y<src.height;++y)for(uint32_t x=0;x<src.width;++x){
        size_t si=(static_cast<size_t>(y)*src.width+x)*4;
        size_t di=(static_cast<size_t>((src.height-1-y)*src.width+(src.width-1-x)))*4;
        for(int c=0;c<4;++c)dst.pixels[di+c]=src.pixels[si+c];
    }
}

void transform_perspective(const Image& src, Image& dst, const float corners[8]) {
    uint32_t w=src.width,h=src.height;
    dst=create_image(w,h,ChannelKind::RGB,0.0f);
    float dx1=corners[0],dy1=corners[1],dx2=corners[2],dy2=corners[3],dx3=corners[4],dy3=corners[5],dx4=corners[6],dy4=corners[7];
    float dX1=dx2-dx1,dX2=dx4-dx1,dX3=dx1-dx2+dx3-dx4;
    float dY1=dy2-dy1,dY2=dy4-dy1,dY3=dy1-dy2+dy3-dy4;
    float den=dX3*dY2-dX2*dY3;
    float g=den!=0?(dY3*dX1-dY1*dX3)/den:0;
    float hh=den!=0?(dX2*dY1-dX1*dY2)/den:0;
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float u=x/(float)w,v=y/(float)h;
        float denom=g*u+hh*v+1;
        if(std::fabs(denom)<1e-6f)continue;
        float sx=w*(u+g*u*denom)/denom;
        float sy=h*(v+hh*v*denom)/denom;
        float out[4];sampleBilinear(src,sx,sy,out);
        size_t idx=(static_cast<size_t>(y)*w+x)*4;for(int c=0;c<4;++c)dst.pixels[idx+c]=out[c];
    }
}

void transform_warp_grid(const Image& src, Image& dst, int gridW, int gridH, const std::vector<float>& offsets) {
    uint32_t w=src.width,h=src.height;
    dst=create_image(w,h,ChannelKind::RGB,0.0f);
    std::vector<float> gx(gridW*gridH),gy(gridW*gridH);
    for(int j=0;j<gridH;++j)for(int i=0;i<gridW;++i){
        gx[j*gridW+i]=(i/(float)(gridW-1))*w+offsets[(j*gridW+i)*2];
        gy[j*gridW+i]=(j/(float)(gridH-1))*h+offsets[(j*gridW+i)*2+1];
    }
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float fi=x/(float)w*(gridW-1),fj=y/(float)h*(gridH-1);
        int i0=std::min(gridW-2,(int)fi),j0=std::min(gridH-2,(int)fj);
        float ti=fi-i0,tj=fj-j0;
        auto bil=[&](const std::vector<float>& g)->float{
            float a=g[j0*gridW+i0]*(1-ti)+g[j0*gridW+i0+1]*ti;
            float b=g[(j0+1)*gridW+i0]*(1-ti)+g[(j0+1)*gridW+i0+1]*ti;
            return a*(1-tj)+b*tj;};
        float sx=bil(gx),sy=bil(gy);
        float out[4];sampleBilinear(src,sx,sy,out);
        size_t idx=(static_cast<size_t>(y)*w+x)*4;for(int c=0;c<4;++c)dst.pixels[idx+c]=out[c];
    }
}

void content_aware_scale(const Image& src, Image& dst, uint32_t newW, uint32_t newH, const std::vector<float>& protectMask) {
    size_t n = src.pixelCount();
    std::vector<float> energy(n);
    auto lumAt = [&](int xx, int yy)->float {
        xx = std::max(0, std::min((int)src.width-1, xx));
        yy = std::max(0, std::min((int)src.height-1, yy));
        const float* p = &src.pixels[(static_cast<size_t>(yy)*src.width+xx)*4];
        return 0.299f*p[0]+0.587f*p[1]+0.114f*p[2];
    };
    for (size_t i = 0; i < n; ++i) {
        int x = i % src.width, y = i / src.width;
        float gx = lumAt(x+1,y)-lumAt(x-1,y), gy = lumAt(x,y+1)-lumAt(x,y-1);
        energy[i] = std::sqrt(gx*gx+gy*gy);
        if (!protectMask.empty()) energy[i] += protectMask[i]*100.0f;
    }
    std::vector<uint32_t> order(n);
    for (size_t i = 0; i < n; ++i) order[i] = static_cast<uint32_t>(i);
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b){ return energy[a] < energy[b]; });
    size_t targetPixels = static_cast<size_t>(newW)*newH;
    if (targetPixels > n) targetPixels = n;
    std::vector<char> removed(n, 0);
    size_t removeCount = n - targetPixels;
    for (size_t k = 0; k < removeCount; ++k) removed[order[k]] = 1;
    std::vector<float> keptX, keptY;
    keptX.reserve(targetPixels); keptY.reserve(targetPixels);
    for (uint32_t y = 0; y < src.height; ++y)
        for (uint32_t x = 0; x < src.width; ++x) {
            size_t idx = static_cast<size_t>(y)*src.width + x;
            if (!removed[idx]) { keptX.push_back((float)x); keptY.push_back((float)y); }
        }
    dst = create_image(newW, newH, ChannelKind::RGB, 0.0f);
    if (keptX.empty()) return;
    size_t m = keptX.size();
    for (uint32_t y = 0; y < newH; ++y)
        for (uint32_t x = 0; x < newW; ++x) {
            size_t ki = std::min(m-1, (static_cast<size_t>(y)*newW + x) * m / targetPixels);
            float out[4];
            sampleBilinear(src, keptX[ki], keptY[ki], out);
            size_t di = (static_cast<size_t>(y)*newW + x)*4;
            for (int c = 0; c < 4; ++c) dst.pixels[di+c] = out[c];
        }
}

void puppet_warp(const Image& src, Image& dst, const std::vector<float>& pinsSrc, const std::vector<float>& pinsDst, int count, float meshDensity) {
    uint32_t w=src.width,h=src.height;
    dst=create_image(w,h,ChannelKind::RGB,0.0f);
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float totalW=0,dx=0,dy=0;
        for(int p=0;p<count;++p){
            float sx=pinsSrc[p*2],sy=pinsSrc[p*2+1];
            float ex=pinsDst[p*2],ey=pinsDst[p*2+1];
            float d=std::hypot(x-sx,y-sy)+meshDensity*50.0f;
            float wgt=1.0f/(d*d);
            totalW+=wgt;dx+=(ex-sx)*wgt;dy+=(ey-sy)*wgt;
        }
        float out[4];sampleBilinear(src,x-totalW>0?dx/totalW:0,y+ (totalW>0?dy/totalW:0),out);
        size_t idx=(static_cast<size_t>(y)*w+x)*4;for(int c=0;c<4;++c)dst.pixels[idx+c]=out[c];
    }
}

}
