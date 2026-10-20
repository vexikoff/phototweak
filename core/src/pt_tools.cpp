#include "pt_tools.h"
#include "pt_math.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <functional>

namespace pt {

static inline float cl(float v){return v<0?0:(v>1?1:v);}

static void stampAt(Image& layer, const BrushSettings& bs, float cx, float cy, float pressure) {
    uint32_t w=layer.width,h=layer.height;
    if(w==0||h==0)return;
    float size=bs.size*(0.3f+0.7f*pressure);
    float rx=size*0.5f,ry=size*0.5f*bs.roundness;
    float ca=std::cos(bs.angleDeg*0.0174533f),sa=std::sin(bs.angleDeg*0.0174533f);
    int x0=std::max(0,(int)(cx-rx-2)),y0=std::max(0,(int)(cy-ry-2));
    int x1=std::min((int)w,(int)(cx+rx+2)),y1=std::min((int)h,(int)(cy+ry+2));
    float flow=bs.flow*bs.opacity*pressure;
    for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){
        float dx=x+0.5f-cx,dy=y+0.5f-cy;
        float rdx=dx*ca+dy*sa,rdy=-dx*sa+dy*ca;
        float d=(rdx*rdx)/(rx*rx+1e-6f)+(rdy*rdy)/(ry*ry+1e-6f);
        float cov=0;
        if(d<=1.0f){
            if(bs.hardness>=0.999f)cov=1.0f;
            else cov=cl((1.0f-d)/(1.0f-bs.hardness+1e-6f));
        }
        if(cov<=0)continue;
        float a=cov*flow;
        size_t idx=(static_cast<size_t>(y)*w+x)*4;
        float* p=&layer.pixels[idx];
        p[0]=p[0]*(1-a)+bs.color.r*a;
        p[1]=p[1]*(1-a)+bs.color.g*a;
        p[2]=p[2]*(1-a)+bs.color.b*a;
        p[3]=cl(p[3]+a);
    }
}

void brush_stamp(Image& layer, const BrushSettings& bs, float cx, float cy) {
    stampAt(layer,bs,cx,cy,1.0f);
}

void brush_stroke(Image& layer, const BrushSettings& bs, const std::vector<float>& pointsXY, const std::vector<float>* pressure) {
    if(pointsXY.size()<4){if(pointsXY.size()>=2)stampAt(layer,bs,pointsXY[0],pointsXY[1],pressure&&!pressure->empty()?(*pressure)[0]:1.0f);return;}
    float spacing=std::max(0.5f,bs.spacing*bs.size);
    size_t n=pointsXY.size()/2;
    float acc=spacing;
    for(size_t i=0;i<n;++i){
        float px=pointsXY[i*2],py=pointsXY[i*2+1];
        float pr=pressure&&i<pressure->size()?(*pressure)[i]:1.0f;
        if(i==0){stampAt(layer,bs,px,py,pr);acc=0;continue;}
        float qx=pointsXY[(i-1)*2],qy=pointsXY[(i-1)*2+1];
        float seg=std::hypot(px-qx,py-qy);
        while(acc+spacing<=seg){
            float t=(acc+spacing-seg)/seg;
            float sx=qx+(px-qx)*t,sy=qy+(py-qy)*t;
            stampAt(layer,bs,sx,sy,pr);
            acc+=spacing;
        }
        acc+=seg;
    }
    stampAt(layer,bs,pointsXY[(n-1)*2],pointsXY[(n-1)*2+1],pressure&&!pressure->empty()?(*pressure)[n-1]:1.0f);
}

void pencil_stroke(Image& layer, const Color& c, float size, const std::vector<float>& pointsXY, const std::vector<float>* pressure) {
    BrushSettings bs;bs.size=size;bs.hardness=1.0f;bs.opacity=1.0f;bs.flow=1.0f;bs.spacing=0.05f;bs.color=c;
    brush_stroke(layer,bs,pointsXY,pressure);
}

void eraser_stroke(Image& layer, const BrushSettings& bsIn, const std::vector<float>& pointsXY, const std::vector<float>* pressure, bool mode) {
    BrushSettings bs=bsIn;
    if(!mode){
        float origOpacity=bs.opacity;
        std::vector<float> tmpLayer;
        Image black=create_image(layer.width,layer.height,ChannelKind::RGB,0.0f);
        (void)black;
        uint32_t w=layer.width,h=layer.height;
        if(pointsXY.size()>=2){
            float spacing=std::max(0.5f,bs.spacing*bs.size);
            size_t n=pointsXY.size()/2;
            float acc=spacing;
            for(size_t i=0;i<n;++i){
                float px=pointsXY[i*2],py=pointsXY[i*2+1];
                float pr=pressure&&i<pressure->size()?(*pressure)[i]:1.0f;
                auto erase=[&](float cx,float cy){
                    float size=bs.size*(0.3f+0.7f*pr);
                    float rx=size*0.5f,ry=size*0.5f*bs.roundness;
                    int x0=std::max(0,(int)(cx-rx-2)),y0=std::max(0,(int)(cy-ry-2));
                    int x1=std::min((int)w,(int)(cx+rx+2)),y1=std::min((int)h,(int)(cy+ry+2));
                    for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){
                        float dx=x+0.5f-cx,dy=y+0.5f-cy;
                        float d=(dx*dx)/(rx*rx+1e-6f)+(dy*dy)/(ry*ry+1e-6f);
                        float cov=d<=1.0f?(bs.hardness>=0.999f?1.0f:cl((1.0f-d)/(1.0f-bs.hardness+1e-6f))):0.0f;
                        if(cov<=0)continue;
                        size_t idx=(static_cast<size_t>(y)*w+x)*4;
                        layer.pixels[idx+3]*=(1.0f-cov*origOpacity*pr);
                    }
                };
                if(i==0){erase(px,py);acc=0;continue;}
                float qx=pointsXY[(i-1)*2],qy=pointsXY[(i-1)*2+1];
                float seg=std::hypot(px-qx,py-qy);
                while(acc+spacing<=seg){float tt=(acc+spacing-seg)/seg;erase(qx+(px-qx)*tt,qy+(py-qy)*tt);acc+=spacing;}
                acc+=seg;
            }
            erase(pointsXY[(n-1)*2],pointsXY[(n-1)*2+1]);
        }
        return;
    }
    Color white{1,1,1,1};
    bs.color=white;
    brush_stroke(layer,bs,pointsXY,pressure);
}

static void gradientLine(Image& layer,const std::vector<float>* mask,uint32_t x,uint32_t y,const std::vector<float>& stops,int stopCount,float t){
    size_t idx=(static_cast<size_t>(y)*layer.width+x)*4;
    float m=mask?(*mask)[y*layer.width+x]:1.0f;
    if(m<=0)return;
    float f=t*(stopCount-1);
    int s0=std::min(stopCount-2,(int)f);s0=std::max(0,s0);
    float ft=f-s0;
    const float* a=&stops[s0*4];const float* b=&stops[(s0+1)*4];
    for(int c=0;c<4;++c){
        float v=a[c]+(b[c]-a[c])*ft;
        float alpha=v*m;
        layer.pixels[idx+c]=layer.pixels[idx+c]*(1-alpha)+v*alpha;
    }
    layer.pixels[idx+3]=cl(layer.pixels[idx+3]);
}

void gradient_linear(Image& layer, const std::vector<float>* mask, float x0, float y0, float x1, float y1, const std::vector<float>& stopsRGBA, int stopCount, int interp) {
    float dx=x1-x0,dy=y1-y0;
    float len2=dx*dx+dy*dy;if(len2<1e-6f)return;
    for(uint32_t y=0;y<layer.height;++y)for(uint32_t x=0;x<layer.width;++x){
        float t=((x-x0)*dx+(y-y0)*dy)/len2;
        t=cl(t);
        if(interp==1)t=t*t*(3-2*t);
        if(interp==2)t=1-(1-t)*(1-t);
        if(interp==3)t=t*t;
        gradientLine(layer,mask,x,y,stopsRGBA,stopCount,t);
    }
}

void gradient_radial(Image& layer, const std::vector<float>* mask, float cx, float cy, float r0, float r1, const std::vector<float>& stopsRGBA, int stopCount) {
    float dr=std::max(1e-3f,r1-r0);
    for(uint32_t y=0;y<layer.height;++y)for(uint32_t x=0;x<layer.width;++x){
        float d=std::hypot(x-cx,y-cy);
        float t=cl((d-r0)/dr);
        gradientLine(layer,mask,x,y,stopsRGBA,stopCount,t);
    }
}

void paint_bucket_fill(Image& layer, const std::vector<float>* mask, int x, int y, const Color& c, float tolerance, bool contiguous) {
    uint32_t w=layer.width,h=layer.height;
    if(x<0||y<0||x>=(int)w||y>=(int)h)return;
    auto idx=[&](int a,int b){return static_cast<size_t>(b)*w+a;};
    float tr=c.r,tg=c.g,tb=c.b;
    size_t start=idx(x,y);
    float sr=layer.pixels[start*4],sg=layer.pixels[start*4+1],sb=layer.pixels[start*4+2];
    auto match=[&](size_t i)->bool{
        return (std::fabs(layer.pixels[i*4]-sr)+std::fabs(layer.pixels[i*4+1]-sg)+std::fabs(layer.pixels[i*4+2]-sb))/3.0f<=tolerance;
    };
    if(contiguous){
        std::vector<char> visited(w*h,0);
        std::queue<std::pair<int,int>> q;q.push({x,y});visited[start]=1;
        while(!q.empty()){
            auto[a,b]=q.front();q.pop();
            size_t i=idx(a,b);
            if(match(i)){
                float m=mask?(*mask)[i]:1.0f;
                layer.pixels[i*4]=layer.pixels[i*4]*(1-m*c.a)+tr*m*c.a;
                layer.pixels[i*4+1]=layer.pixels[i*4+1]*(1-m*c.a)+tg*m*c.a;
                layer.pixels[i*4+2]=layer.pixels[i*4+2]*(1-m*c.a)+tb*m*c.a;
                layer.pixels[i*4+3]=cl(layer.pixels[i*4+3]+m*c.a);
                int dirs[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
                for(auto&d:dirs){int nx=a+d[0],ny=b+d[1];if(nx<0||ny<0||nx>=(int)w||ny>=(int)h)continue;size_t ni=idx(nx,ny);if(!visited[ni]){visited[ni]=1;q.push({nx,ny});}}
            }
        }
    }else{
        for(uint32_t yy=0;yy<h;++yy)for(uint32_t xx=0;xx<w;++xx){
            size_t i=idx(xx,yy);
            if(match(i)){float m=mask?(*mask)[i]:1.0f;layer.pixels[i*4]+= (tr-layer.pixels[i*4])*m*c.a;layer.pixels[i*4+1]+=(tg-layer.pixels[i*4+1])*m*c.a;layer.pixels[i*4+2]+=(tb-layer.pixels[i*4+2])*m*c.a;layer.pixels[i*4+3]=cl(layer.pixels[i*4+3]+m*c.a);}
        }
    }
}

void clone_stamp(Image& dst, const Image& srcLayer, const std::vector<float>* mask, int dx, int dy, int w, int h, float offsetOX, float offsetOY, float opacity) {
    int x0=std::max(0,dx),y0=std::max(0,dy);
    int x1=std::min((int)dst.width,dx+w),y1=std::min((int)dst.height,dy+h);
    for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){
        int sx=x+offsetOX,sy=y+offsetOY;
        if(sx<0||sy<0||sx>=(int)srcLayer.width||sy>=(int)srcLayer.height)continue;
        size_t si=(static_cast<size_t>(sy)*srcLayer.width+sx)*4;
        size_t di=(static_cast<size_t>(y)*dst.width+x)*4;
        float m=mask?(*mask)[y*dst.width+x]:1.0f;
        float a=srcLayer.pixels[si+3]*opacity*m;
        for(int c=0;c<3;++c)dst.pixels[di+c]=dst.pixels[di+c]*(1-a)+srcLayer.pixels[si+c]*a;
        dst.pixels[di+3]=cl(dst.pixels[di+3]+a);
    }
}

void pattern_stamp(Image& dst, const Image& pattern, const std::vector<float>* mask, int x0, int y0, int w, int h, int ox, int oy, float opacity) {
    if(pattern.width==0)return;
    for(int y=y0;y<y0+h;++y)for(int x=x0;x<x0+w;++x){
        if(x<0||y<0||x>=(int)dst.width||y>=(int)dst.height)continue;
        int sx=((x+ox)%pattern.width+pattern.width)%pattern.width;
        int sy=((y+oy)%pattern.height+pattern.height)%pattern.height;
        size_t si=(static_cast<size_t>(sy)*pattern.width+sx)*4;
        size_t di=(static_cast<size_t>(y)*dst.width+x)*4;
        float m=mask?(*mask)[y*dst.width+x]:1.0f;
        float a=pattern.pixels[si+3]*opacity*m;
        for(int c=0;c<3;++c)dst.pixels[di+c]=dst.pixels[di+c]*(1-a)+pattern.pixels[si+c]*a;
        dst.pixels[di+3]=cl(dst.pixels[di+3]+a);
    }
}

void healing_brush(Image& layer, const Image& srcLayer, const std::vector<float>* mask, int tx, int ty, int sx, int sy, int radius, float sampleAlpha) {
    uint32_t w=layer.width,h=layer.height;
    for(int y=ty-radius;y<=ty+radius;++y)for(int x=tx-radius;x<=tx+radius;++x){
        if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
        float d=std::hypot(x-tx,y-ty);
        if(d>radius)continue;
        float cov=cl((radius-d)/std::max(1.0f,radius*0.3f));
        int hx=sx+(x-tx),hy=sy+(y-ty);
        if(hx<0||hy<0||hx>=(int)srcLayer.width||hy>=(int)srcLayer.height)continue;
        size_t hi=(static_cast<size_t>(y)*w+x)*4,si=(static_cast<size_t>(hy)*srcLayer.width+hx)*4;
        float m=mask?(*mask)[y*w+x]:1.0f;
        float a=cov*m*sampleAlpha;
        float textureR=srcLayer.pixels[si]-0.0f;
        for(int c=0;c<3;++c){
            float blended=layer.pixels[hi+c]*(1-a)+textureR*0.0f+srcLayer.pixels[si+c]*a+layer.pixels[hi+c]*(1-a)*0.0f;
            layer.pixels[hi+c]=cl(blended);
        }
    }
}

void spot_healing(Image& layer, const std::vector<float>* mask, int cx, int cy, int radius, int type, float tolerance) {
    uint32_t w=layer.width,h=layer.height;
    float sum[3]={0,0,0};int cnt=0;
    for(int y=cy-radius*2;y<=cy+radius*2;++y)for(int x=cx-radius*2;x<=cx+radius*2;++x){
        if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
        float d=std::hypot(x-cx,y-cy);
        if(d<radius)continue;
        if(d>radius*2)continue;
        size_t i=(static_cast<size_t>(y)*w+x)*4;
        for(int c=0;c<3;++c)sum[c]+=layer.pixels[i+c];
        cnt++;
    }
    if(cnt==0)return;
    for(int c=0;c<3;++c)sum[c]/=cnt;
    for(int y=cy-radius;y<=cy+radius;++y)for(int x=cx-radius;x<=cx+radius;++x){
        if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
        float d=std::hypot(x-cx,y-cy);
        if(d>radius)continue;
        size_t i=(static_cast<size_t>(y)*w+x)*4;
        float m=mask?(*mask)[y*w+x]:1.0f;
        float a=cl((radius-d)/std::max(1.0f,radius*0.4f))*m;
        for(int c=0;c<3;++c)layer.pixels[i+c]=cl(layer.pixels[i+c]*(1-a)+sum[c]*a);
    }
}

void patch_repair(Image& layer, const std::vector<float>* selMask, const std::vector<float>& patchMask, int method) {
    uint32_t w=layer.width,h=layer.height;
    size_t n=w*h;
    float avg[3]={0,0,0};int cnt=0;
    for(size_t i=0;i<n;++i)if(patchMask[i]>0.5f){for(int c=0;c<3;++c)avg[c]+=layer.pixels[i*4+c];cnt++;}
    if(cnt==0)return;
    for(int c=0;c<3;++c)avg[c]/=cnt;
    float avgLum=0.299f*avg[0]+0.587f*avg[1]+0.114f*avg[2];
    for(size_t i=0;i<n;++i){
        if(!selMask||selMask[i]<=0.5f)continue;
        float lum=0.299f*layer.pixels[i*4]+0.587f*layer.pixels[i*4+1]+0.114f*layer.pixels[i*4+2];
        for(int c=0;c<3;++c)layer.pixels[i*4+c]=cl(avg[c]+(lum-avgLum)*(method==0?0.3f:1.0f));
    }
}

void content_aware_move(Image& layer, const std::vector<float>* mask, const std::vector<float>& regionMask, int dx, int dy) {
    uint32_t w=layer.width,h=layer.height;
    Image backup=clone_image(layer);
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        size_t i=y*w+x;
        if(regionMask[i]<=0.5f)continue;
        int sx=x-dx,sy=y-dy;
        if(sx<0||sy<0||sx>=(int)w||sy>=(int)h)continue;
        size_t si=sy*w+sx;
        float m=mask?(*mask)[si]:1.0f;
        size_t di=i;
        for(int c=0;c<4;++c)layer.pixels[di*4+c]=backup.pixels[si*4+c]*m+layer.pixels[di*4+c]*(1-m);
    }
    std::vector<float> hole;
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){size_t i=y*w+x;if(regionMask[i]>0.5f)hole.push_back(layer.pixels[i*4]);}
    if(hole.empty())return;
    float mean=0;for(float v:hole)mean+=v;mean/=hole.size();
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        size_t i=y*w+x;if(regionMask[i]<=0.5f)continue;
        float sumW=0,sumC[3]={0,0,0};
        for(int r=1;r<=6;++r){
            for(int ang=0;ang<8;++ang){
                float a=ang*3.14159265f/4.0f;
                int nx=x+(int)(std::cos(a)*r),ny=y+(int)(std::sin(a)*r);
                if(nx<0||ny<0||nx>=(int)w||ny>=(int)h)continue;
                size_t ni=ny*w+nx;
                if(regionMask[ni]>0.5f)continue;
                float wt=1.0f/r;
                for(int c=0;c<3;++c)sumC[c]+=layer.pixels[ni*4+c]*wt;
                sumW+=wt;
            }
        }
        if(sumW>0)for(int c=0;c<3;++c)layer.pixels[i*4+c]=sumC[c]/sumW;
        else for(int c=0;c<3;++c)layer.pixels[i*4+c]=mean;
    }
}

void red_eye_removal(Image& layer, const std::vector<float>* mask, int cx, int cy, int radius, float pupilSize, float darkening) {
    uint32_t w=layer.width,h=layer.height;
    for(int y=cy-radius;y<=cy+radius;++y)for(int x=cx-radius;x<=cx+radius;++x){
        if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
        float d=std::hypot(x-cx,y-cy);
        if(d>pupilSize)continue;
        size_t i=(static_cast<size_t>(y)*w+x)*4;
        float sat=std::max({layer.pixels[i],layer.pixels[i+1],layer.pixels[i+2]})-std::min({layer.pixels[i],layer.pixels[i+1],layer.pixels[i+2]});
        if(layer.pixels[i]>layer.pixels[i+1]&&sat>0.2f){
            float L=0.299f*layer.pixels[i]+0.587f*layer.pixels[i+1]+0.114f*layer.pixels[i+2];
            float a=darkening*cl((pupilSize-d)/pupilSize);
            for(int c=0;c<3;++c)layer.pixels[i+c]=cl(layer.pixels[i+c]*(1-a)+L*0.3f*a);
        }
    }
}

static void applyRadial(Image& layer,const std::vector<float>* mask,const std::vector<float>& points,int radius,const std::vector<float>* pressure,std::function<void(float*,float)> fn){
    uint32_t w=layer.width,h=layer.height;
    size_t n=points.size()/2;
    for(size_t i=0;i<n;++i){
        int cx=points[i*2],cy=points[i*2+1];
        float pr=pressure&&i<pressure->size()?(*pressure)[i]:1.0f;
        for(int y=cy-radius;y<=cy+radius;++y)for(int x=cx-radius;x<=cx+radius;++x){
            if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
            float d=std::hypot(x-cx,y-cy);
            if(d>radius)continue;
            float cov=cl((radius-d)/std::max(1.0f,radius*0.5f))*pr;
            float m=mask?(*mask)[y*w+x]:1.0f;
            fn(&layer.pixels[(static_cast<size_t>(y)*w+x)*4],cov*m);
        }
    }
}

void dodge_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float exposure, float range, const std::vector<float>* pressure) {
    applyRadial(layer,mask,points,radius,pressure,[&](float* p,float a){
        float L=0.299f*p[0]+0.587f*p[1]+0.114f*p[2];
        if(range==1&&L>0.7f)return;
        if(range==2&&L<0.3f)return;
        for(int c=0;c<3;++c)p[c]=cl(p[c]+p[c]*exposure*a);
    });
}

void burn_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float exposure, float range, const std::vector<float>* pressure) {
    applyRadial(layer,mask,points,radius,pressure,[&](float* p,float a){
        float L=0.299f*p[0]+0.587f*p[1]+0.114f*p[2];
        if(range==1&&L>0.7f)return;
        if(range==2&&L<0.3f)return;
        for(int c=0;c<3;++c)p[c]=cl(p[c]-p[c]*exposure*a);
    });
}

void sponge_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float flow, int mode, const std::vector<float>* pressure) {
    applyRadial(layer,mask,points,radius,pressure,[&](float* p,float a){
        Color hsl=rgb_to_hsl(p[0],p[1],p[2]);
        hsl.g=cl(mode==0?hsl.g-flow*a:hsl.g+flow*a);
        float r,g,b;Color hc{hsl.r,hsl.g,hsl.b,1};hsl_to_rgb(hc,r,g,b);
        p[0]=cl(r);p[1]=cl(g);p[2]=cl(b);
    });
}

void blur_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float strength, const std::vector<float>* pressure) {
    uint32_t w=layer.width,h=layer.height;
    Image src=clone_image(layer);
    applyRadial(layer,mask,points,radius,pressure,[&](float* p,float a){
        size_t i=(p-layer.pixels.data())/4;
        int x=i%w,y=i/w;
        float sum[3]={0,0,0};int cnt=0;
        for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx){
            int sx=std::max(0,std::min((int)w-1,x+dx)),sy=std::max(0,std::min((int)h-1,y+dy));
            size_t j=(static_cast<size_t>(sy)*w+sx)*4;
            for(int c=0;c<3;++c)sum[c]+=src.pixels[j+c];cnt++;
        }
        for(int c=0;c<3;++c)p[c]=cl(p[c]*(1-strength*a)+sum[c]/cnt*strength*a);
    });
}

void sharpen_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float strength, const std::vector<float>* pressure) {
    uint32_t w=layer.width,h=layer.height;
    Image src=clone_image(layer);
    applyRadial(layer,mask,points,radius,pressure,[&](float* p,float a){
        size_t i=(p-layer.pixels.data())/4;
        int x=i%w,y=i/w;
        float sum[3]={0,0,0};int cnt=0;
        for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx){
            int sx=std::max(0,std::min((int)w-1,x+dx)),sy=std::max(0,std::min((int)h-1,y+dy));
            size_t j=(static_cast<size_t>(sy)*w+sx)*4;
            for(int c=0;c<3;++c)sum[c]+=src.pixels[j+c];cnt++;
        }
        for(int c=0;c<3;++c){
            float avg=sum[c]/cnt;
            p[c]=cl(p[c]+(p[c]-avg)*strength*a);
        }
    });
}

void smudge_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float strength, const std::vector<float>* pressure) {
    uint32_t w=layer.width,h=layer.height;
    Image src=clone_image(layer);
    applyRadial(layer,mask,points,radius,pressure,[&](float* p,float a){
        size_t i=(p-layer.pixels.data())/4;
        int x=i%w,y=i/w;
        float sum[4]={0,0,0,0};float wsum=0;
        for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx){
            int sx=std::max(0,std::min((int)w-1,x+dx)),sy=std::max(0,std::min((int)h-1,y+dy));
            size_t j=(static_cast<size_t>(sy)*w+sx)*4;
            float wt=1.0f/(1.0f+std::hypot(dx,dy));
            for(int c=0;c<4;++c)sum[c]+=src.pixels[j+c]*wt;
            wsum+=wt;
        }
        for(int c=0;c<3;++c)p[c]=cl(p[c]*(1-strength*a)+sum[c]/wsum*strength*a);
    });
}

void mixer_brush_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float wet, float mix, float flow, const Color& loadColor, const std::vector<float>* pressure) {
    uint32_t w=layer.width,h=layer.height;
    BrushSettings bs;bs.size=radius*2;bs.color=loadColor;bs.hardness=0.6f;bs.opacity=flow;bs.flow=flow;
    brush_stroke(layer,bs,points,pressure);
    std::vector<float> pts=points;
    blur_stroke(layer,mask,pts,radius,wet*0.3f,pressure);
}

void color_replace_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, const Color& replacement, float tolerance, int sampleMode, bool locks) {
    uint32_t w=layer.width,h=layer.height;
    Image src=clone_image(layer);
    size_t n=points.size()/2;
    if(n==0)return;
    size_t bi=(static_cast<size_t>(points[1])*w+points[0])*4;
    Color baseHsl=rgb_to_hsl(src.pixels[bi],src.pixels[bi+1],src.pixels[bi+2]);
    Color repHsl=rgb_to_hsl(replacement.r,replacement.g,replacement.b);
    for(size_t i=0;i<n;++i){
        int cx=points[i*2],cy=points[i*2+1];
        for(int y=cy-radius;y<=cy+radius;++y)for(int x=cx-radius;x<=cx+radius;++x){
            if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
            float d=std::hypot(x-cx,y-cy);
            if(d>radius)continue;
            size_t idx=(static_cast<size_t>(y)*w+x)*4;
            Color cur=rgb_to_hsl(src.pixels[idx],src.pixels[idx+1],src.pixels[idx+2]);
            float diff=std::fabs(cur.r-baseHsl.r)*360;
            if(diff>tolerance*360)continue;
            Color out{repHsl.r,locks?cur.g:repHsl.g,cur.b,1};
            float r,g,b;hsl_to_rgb(out,r,g,b);
            float m=mask?(*mask)[y*w+x]:1.0f;
            float a=cl((radius-d)/std::max(1.0f,radius*0.5f))*m;
            layer.pixels[idx]=cl(layer.pixels[idx]*(1-a)+cl(r)*a);
            layer.pixels[idx+1]=cl(layer.pixels[idx+1]*(1-a)+cl(g)*a);
            layer.pixels[idx+2]=cl(layer.pixels[idx+2]*(1-a)+cl(b)*a);
        }
    }
}

void art_history_stroke(Image& srcHist, Image& dst, const std::vector<float>* mask, const std::vector<float>& points, int radius, float tolerance, int style) {
    uint32_t w=dst.width,h=dst.height;
    BrushSettings bs;bs.size=radius*2;bs.hardness=0.7f;bs.opacity=1.0f;bs.flow=1.0f;bs.spacing=0.1f;
    size_t n=points.size()/2;
    for(size_t i=0;i<n;++i){
        int cx=points[i*2],cy=points[i*2+1];
        for(int y=cy-radius;y<=cy+radius;++y)for(int x=cx-radius;x<=cx+radius;++x){
            if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
            float d=std::hypot(x-cx,y-cy);
            if(d>radius)continue;
            size_t idx=(static_cast<size_t>(y)*w+x)*4;
            float a=cl((radius-d)/std::max(1.0f,radius*0.5f));
            float m=mask?(*mask)[y*w+x]:1.0f;
            a*=m;
            for(int c=0;c<4;++c)dst.pixels[idx+c]=cl(dst.pixels[idx+c]*(1-a)+srcHist.pixels[idx+c]*a);
        }
    }
    (void)tolerance;(void)style;
}

void background_eraser_stroke(Image& layer, const std::vector<float>& points, int brushSize, float tolerance, int limits, bool protectFg) {
    uint32_t w=layer.width,h=layer.height;
    size_t n=points.size()/2;
    Color fg{0,0,0,1};
    for(size_t i=0;i<n;++i){
        int cx=points[i*2],cy=points[i*2+1];
        size_t bi=(static_cast<size_t>(cy)*w+cx)*4;
        if(i==0){fg.r=layer.pixels[bi];fg.g=layer.pixels[bi+1];fg.b=layer.pixels[bi+2];}
        for(int y=cy-brushSize/2;y<=cy+brushSize/2;++y)for(int x=cx-brushSize/2;x<=cx+brushSize/2;++x){
            if(x<0||y<0||x>=(int)w||y>=(int)h)continue;
            float d=std::hypot(x-cx,y-cy);
            if(d>brushSize/2.0f)continue;
            size_t idx=(static_cast<size_t>(y)*w+x)*4;
            float diff=(std::fabs(layer.pixels[idx]-fg.r)+std::fabs(layer.pixels[idx+1]-fg.g)+std::fabs(layer.pixels[idx+2]-fg.b))/3.0f;
            if(diff<=tolerance)layer.pixels[idx+3]=0;
        }
    }
    (void)limits;(void)protectFg;
}

void magic_eraser_click(Image& layer, int x, int y, float tolerance, bool contiguous, bool antiAlias, float strength) {
    uint32_t w=layer.width,h=layer.height;
    if(x<0||y<0||x>=(int)w||y>=(int)h)return;
    size_t start=(static_cast<size_t>(y)*w+x)*4;
    float sr=layer.pixels[start],sg=layer.pixels[start+1],sb=layer.pixels[start+2];
    auto match=[&](size_t i)->bool{return (std::fabs(layer.pixels[i*4]-sr)+std::fabs(layer.pixels[i*4+1]-sg)+std::fabs(layer.pixels[i*4+2]-sb))/3.0f<=tolerance;};
    if(contiguous){
        std::vector<char> visited(w*h,0);
        std::queue<std::pair<int,int>> q;q.push({x,y});visited[y*w+x]=1;
        while(!q.empty()){
            auto[a,b]=q.front();q.pop();
            size_t i=b*w+a;
            if(match(i)){layer.pixels[i*4+3]*=(1.0f-strength);
                int dirs[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
                for(auto&d:dirs){int nx=a+d[0],ny=b+d[1];if(nx<0||ny<0||nx>=(int)w||ny>=(int)h)continue;size_t ni=ny*w+nx;if(!visited[ni]){visited[ni]=1;q.push({nx,ny});}}}
        }
    }else{
        for(uint32_t yy=0;yy<h;++yy)for(uint32_t xx=0;xx<w;++xx){size_t i=yy*w+xx;if(match(i))layer.pixels[i*4+3]*=(1.0f-strength);}
    }
}

}
