#include "pt_selection.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <cstring>

namespace pt {

void mask_rect(std::vector<float>& mask, uint32_t w, uint32_t h, int x0, int y0, int x1, int y1) {
    mask.assign(static_cast<size_t>(w)*h, 0.0f);
    int a = std::max(0, std::min(x0,x1)), b = std::max(0, std::min(y0,y1));
    int c = std::min((int)w, std::max(x0,x1)), d = std::min((int)h, std::max(y0,y1));
    for (int y=b;y<d;++y) for(int x=a;x<c;++x) mask[y*w+x]=1.0f;
}

void mask_ellipse(std::vector<float>& mask, uint32_t w, uint32_t h, float cx, float cy, float rx, float ry) {
    mask.assign(static_cast<size_t>(w)*h,0.0f);
    if(rx<=0||ry<=0)return;
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float dx=(x+0.5f-cx)/rx,dy=(y+0.5f-cy)/ry;
        float d=dx*dx+dy*dy;
        if(d<=1.0f)mask[y*w+x]=d>0.85f?(1.0f-d)/0.15f:1.0f;
    }
}

void mask_polygon(std::vector<float>& mask, uint32_t w, uint32_t h, const std::vector<float>& pointsXY, bool invert) {
    mask.assign(static_cast<size_t>(w)*h,0.0f);
    size_t npts=pointsXY.size()/2;
    if(npts<3)return;
    for(uint32_t y=0;y<h;++y){
        std::vector<float> xs;
        float py=y+0.5f;
        for(size_t i=0;i<npts;++i){
            float x0=pointsXY[i*2],y0=pointsXY[i*2+1];
            float x1=pointsXY[((i+1)%npts)*2],y1=pointsXY[((i+1)%npts)*2+1];
            if((y0<=py&&y1>py)||(y1<=py&&y0>py)){
                float t=(py-y0)/(y1-y0);
                xs.push_back(x0+t*(x1-x0));
            }
        }
        std::sort(xs.begin(),xs.end());
        for(size_t k=0;k+1<xs.size();k+=2){
            int xa=std::max(0,(int)xs[k]),xb=std::min((int)w-1,(int)xs[k+1]);
            for(int x=xa;x<=xb;++x)mask[y*w+x]=invert?0.0f:1.0f;
        }
    }
    if(invert)for(auto&v:mask)v=1.0f-v;
}

static float clm(float v){return v<0?0:(v>1?1:v);}

static void floodFillWand(const Image& img, std::vector<float>& mask, int sx, int sy, float tol, bool contiguous) {
    uint32_t w=img.width,h=img.height;
    auto idx=[&](int x,int y){return static_cast<size_t>(y)*w+x;};
    float r0=img.pixels[idx(sx,sy)*4],g0=img.pixels[idx(sx,sy)*4+1],b0=img.pixels[idx(sx,sy)*4+2];
    std::vector<char> visited(w*h,0);
    std::queue<std::pair<int,int>> q;
    q.push({sx,sy});visited[idx(sx,sy)]=1;mask[idx(sx,sy)]=1.0f;
    while(!q.empty()){
        auto[x,y]=q.front();q.pop();
        int dirs[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
        for(auto&d:dirs){
            int nx=x+d[0],ny=y+d[1];
            if(nx<0||ny<0||nx>=(int)w||ny>=(int)h)continue;
            size_t ni=idx(nx,ny);
            if(visited[ni])continue;
            visited[ni]=1;
            float dr=std::fabs(img.pixels[ni*4]-r0),dg=std::fabs(img.pixels[ni*4+1]-g0),db=std::fabs(img.pixels[ni*4+2]-b0);
            if((dr+dg+db)/3.0f<=tol){mask[ni]=1.0f;q.push({nx,ny});}
        }
    }
    if(!contiguous){
        for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
            size_t i=idx(x,y);
            float dr=std::fabs(img.pixels[i*4]-r0),dg=std::fabs(img.pixels[i*4+1]-g0),db=std::fabs(img.pixels[i*4+2]-b0);
            if((dr+dg+db)/3.0f<=tol)mask[i]=1.0f;
        }
    }
}

void magic_wand(const Image& img, const std::vector<float>* curMask, std::vector<float>& outMask,
               int x, int y, float tolerance, bool contiguous, bool antiAlias, float featherPx, bool addToSelection) {
    uint32_t w=img.width,h=img.height;
    std::vector<float> fresh(static_cast<size_t>(w)*h,0.0f);
    x=std::max(0,std::min((int)w-1,x));y=std::max(0,std::min((int)h-1,y));
    floodFillWand(img,fresh,x,y,tolerance,contiguous);
    if(antiAlias){
        std::vector<float> tmp=fresh;
        for(uint32_t yy=1;yy+1<h;++yy)for(uint32_t xx=1;xx+1<w;++xx){
            size_t i=yy*w+xx;float avg=(tmp[i-1]+tmp[i+1]+tmp[i-w]+tmp[i+w])*0.25f;
            fresh[i]=std::max(fresh[i],avg*0.7f);
        }
    }
    if(featherPx>0)feather_mask(fresh,w,h,featherPx);
    if(addToSelection&&curMask&&curMask->size()==fresh.size())
        for(size_t i=0;i<fresh.size();++i)outMask[i]=std::max((*curMask)[i],fresh[i]);
    else outMask.swap(fresh);
}

void quick_selection_grow(const Image& img, std::vector<float>& mask, int cx, int cy, int radius, float tolerance, uint32_t steps) {
    uint32_t w=img.width,h=img.height;
    if(mask.size()!=static_cast<size_t>(w)*h)mask.assign(static_cast<size_t>(w)*h,0.0f);
    for(uint32_t s=0;s<steps;++s){
        std::vector<float> newMask=mask;
        for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
            size_t i=y*w+x;
            if(mask[i]>0.5f)continue;
            bool near=false;
            for(int dy=-radius;dy<=radius&&!near;++dy)for(int dx=-radius;dx<=radius;++dx){
                int sx=x+dx,sy=y+dy;
                if(sx<0||sy<0||sx>=(int)w||sy>=(int)h)continue;
                if(mask[sy*w+sx]>0.5f){near=true;break;}
            }
            if(!near)continue;
            float lr=0,lg=0,lb=0;int cnt=0;
            for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx){
                int sx=x+dx,sy=y+dy;
                if(sx<0||sy<0||sx>=(int)w||sy>=(int)h)continue;
                if(mask[sy*w+sx]>0.5f){lr+=img.pixels[(sy*w+sx)*4];lg+=img.pixels[(sy*w+sx)*4+1];lb+=img.pixels[(sy*w+sx)*4+2];cnt++;}
            }
            if(cnt==0)continue;
            lr/=cnt;lg/=cnt;lb/=cnt;
            float d=(std::fabs(img.pixels[i*4]-lr)+std::fabs(img.pixels[i*4+1]-lg)+std::fabs(img.pixels[i*4+2]-lb))/3.0f;
            if(d<=tolerance)newMask[i]=1.0f;
        }
        mask.swap(newMask);
    }
}

void feather_mask(std::vector<float>& mask, uint32_t w, uint32_t h, float radius) {
    if(radius<=0)return;
    int r=static_cast<int>(radius);
    std::vector<float> tmp(mask.size()),acc(mask.size());
    int win=2*r+1;
    for(uint32_t y=0;y<h;++y){
        float sum=0;
        for(int k=-r;k<=r;++k)sum+=mask[y*w+std::min<uint32_t>(w-1,std::max(0,k))];
        for(uint32_t x=0;x<w;++x){
            acc[y*w+x]=sum/win;
            sum+=mask[y*w+std::min<uint32_t>(w-1,x+r+1)]-mask[y*w+std::max(0,(int)x-r)];
        }
    }
    for(uint32_t x=0;x<w;++x){
        float sum=0;
        for(int k=-r;k<=r;++k)sum+=acc[std::min<uint32_t>(h-1,std::max(0,k))*w+x];
        for(uint32_t y=0;y<h;++y){
            tmp[y*w+x]=sum/win;
            sum+=acc[std::min<uint32_t>(h-1,y+r+1)*w+x]-acc[std::max(0,(int)y-r)*w+x];
        }
    }
    mask.swap(tmp);
}

void smooth_mask(std::vector<float>& mask, uint32_t w, uint32_t h, float radius, float edgeThin) {
    feather_mask(mask,w,h,radius);
    for(auto&v:mask)v=v>0.5f?1.0f:0.0f;
    if(edgeThin!=0){
        for(auto&v:mask)v=clm(v-edgeThin*0.1f);
    }
}

static float cl(float v){return v<0?0:(v>1?1:v);}

void contract_mask(std::vector<float>& mask, uint32_t w, uint32_t h, int px) {
    std::vector<float> tmp(mask.size());
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float mn=1.0f;
        for(int dy=-px;dy<=px;++dy)for(int dx=-px;dx<=px;++dx){
            int sx=std::max(0,std::min((int)w-1,x+dx)),sy=std::max(0,std::min((int)h-1,y+dy));
            mn=std::min(mn,mask[sy*w+sx]);
        }
        tmp[y*w+x]=mn;
    }
    mask.swap(tmp);
}

void expand_mask(std::vector<float>& mask, uint32_t w, uint32_t h, int px) {
    std::vector<float> tmp(mask.size());
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        float mx=0.0f;
        for(int dy=-px;dy<=px;++dy)for(int dx=-px;dx<=px;++dx){
            int sx=std::max(0,std::min((int)w-1,x+dx)),sy=std::max(0,std::min((int)h-1,y+dy));
            mx=std::max(mx,mask[sy*w+sx]);
        }
        tmp[y*w+x]=mx;
    }
    mask.swap(tmp);
}

void border_mask(std::vector<float>& mask, uint32_t w, uint32_t h, int thickness) {
    std::vector<float> outer=mask,inner=mask;
    expand_mask(outer,w,h,thickness/2);
    contract_mask(inner,w,h,thickness/2);
    for(size_t i=0;i<mask.size();++i)mask[i]=clm(outer[i]-inner[i]);
}

void invert_mask(std::vector<float>& mask) { for(auto&v:mask)v=1.0f-v; }

void refine_edge_std(Image& img, std::vector<float>& mask, float radius, float smooth, float feather, float contrast, float shiftEdge) {
    uint32_t w=img.width,h=img.height;
    if(shiftEdge!=0){
        if(shiftEdge>0)expand_mask(mask,w,h,(int)shiftEdge);
        else contract_mask(mask,w,h,(int)-shiftEdge);
    }
    if(smooth>0)smooth_mask(mask,w,h,smooth,0);
    if(feather>0)feather_mask(mask,w,h,feather);
    if(contrast!=0)for(auto&v:mask)v=clm((v-0.5f)*(1.0f+contrast)+0.5f);
    (void)radius;
}

std::vector<uint8_t> mask_to_png_bytes(const std::vector<float>& mask, uint32_t w, uint32_t h) {
    Image m=create_image(w,h,ChannelKind::RGB,0);
    for(size_t i=0;i<mask.size();++i){m.pixels[i*4]=m.pixels[i*4+1]=m.pixels[i*4+2]=mask[i];m.pixels[i*4+3]=1;}
    std::string tmp="/tmp/pt_mask.png";
    save_png(tmp,m);
    FILE*f=fopen(tmp.c_str(),"rb");
    std::vector<uint8_t> bytes;
    if(f){fseek(f,0,SEEK_END);long sz=ftell(f);fseek(f,0,SEEK_SET);bytes.resize(sz);fread(bytes.data(),1,sz,f);fclose(f);}
    return bytes;
}

bool png_bytes_to_mask(const uint8_t* bytes, size_t len, std::vector<float>& mask, uint32_t& w, uint32_t& h) {
    std::string tmp="/tmp/pt_mask_in.png";
    FILE*f=fopen(tmp.c_str(),"wb");if(!f)return false;
    fwrite(bytes,1,len,f);fclose(f);
    Image img;if(!load_png(tmp,img))return false;
    w=img.width;h=img.height;
    mask.resize(img.pixelCount());
    for(size_t i=0;i<mask.size();++i)mask[i]=img.pixels[i*4];
    return true;
}

void apply_mask_to_layer(Image& layer, const std::vector<float>& mask) {
    size_t n=layer.pixelCount();
    if(mask.size()!=n)return;
    for(size_t i=0;i<n;++i)layer.pixels[i*4+3]*=mask[i];
}

}
