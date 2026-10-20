#include "pt_vector.h"
#include <cmath>
#include <sstream>
#include <cstring>
#include <algorithm>

namespace pt {

static void cubicFlatten(const PathPoint& a, const PathPoint& b, std::vector<float>& poly, float tolerance) {
    float x0=a.x,y0=a.y,x3=b.x,y3=b.y;
    float x1=a.outX,y1=a.outY,x2=b.inX,y2=b.inY;
    int steps=std::max(6,(int)(std::hypot(x3-x0,y3-y0)/tolerance));
    for(int i=0;i<=steps;++i){
        float t=i/(float)steps,u=1-t;
        float bx=u*u*u*x0+3*u*u*t*x1+3*u*t*t*x2+t*t*t*x3;
        float by=u*u*u*y0+3*u*u*t*y1+3*u*t*t*y2+t*t*t*y3;
        poly.push_back(bx);poly.push_back(by);
    }
}

void path_flatten(const VectorPath& vp, std::vector<std::vector<float>>& polys, float tolerance) {
    polys.clear();
    for(auto&sp:vp.subpaths){
        if(sp.points.empty())continue;
        std::vector<float> poly;
        if(sp.closed){
            size_t n=sp.points.size();
            for(size_t i=0;i<n;++i)cubicFlatten(sp.points[i],sp.points[(i+1)%n],poly,tolerance);
        }else{
            size_t n=sp.points.size();
            poly.push_back(sp.points[0].x);poly.push_back(sp.points[0].y);
            for(size_t i=0;i+1<n;++i)cubicFlatten(sp.points[i],sp.points[i+1],poly,tolerance);
        }
        polys.push_back(poly);
    }
}

void shape_rect(VectorPath& vp, float x, float y, float w, float h, float radius) {
    SubPath sp;sp.closed=true;
    auto corner=[&](float cx,float cy,float sx,float sy){
        PathPoint p;p.x=cx;p.y=cy;
        p.inX=cx-sx*radius;p.inY=cy-sy*radius;
        p.outX=cx+sx*radius;p.outY=cy+sy*radius;
        return p;};
    if(radius<=0){
        float xs[]={x,x+w,x+w,x},ys[]={y,y,y+h,y+h};
        for(int i=0;i<4;++i){PathPoint p;p.x=xs[i];p.y=ys[i];sp.points.push_back(p);}
    }else{
        sp.points.push_back(corner(x+radius,y,0,-1));
        sp.points.push_back(PathPoint{x+w-radius,y,0,0,0,0,true});
        sp.points.push_back(corner(x+w,y+radius,1,0));
        sp.points.push_back(PathPoint{x+w,y+h-radius,0,0,0,0,true});
        sp.points.push_back(corner(x+w-radius,y+h,0,1));
        sp.points.push_back(PathPoint{x+radius,y+h,0,0,0,0,true});
        sp.points.push_back(corner(x,y+h-radius,-1,0));
        sp.points.push_back(PathPoint{x,y+radius,0,0,0,0,true});
    }
    vp.subpaths.push_back(sp);
}

void shape_ellipse(VectorPath& vp, float cx, float cy, float rx, float ry) {
    SubPath sp;sp.closed=true;
    const float k=0.5522847f;
    struct P{float x,y,ix,iy,ox,oy;};
    P pts[4]={{cx,cy-ry,cx-rx*k,cy-ry,cx+rx*k,cy-ry},{cx+rx,cy,cx+rx,cy-ry*k,cx+rx,cy+ry*k},{cx,cy+ry,cx+rx*k,cy+ry,cx-rx*k,cy+ry},{cx-rx,cy,cx-rx,cy+ry*k,cx-rx,cy-ry*k}};
    for(auto&p:pts){PathPoint pp;pp.x=p.x;pp.y=p.y;pp.inX=p.ix;pp.inY=p.iy;pp.outX=p.ox;pp.outY=p.oy;sp.points.push_back(pp);}
    vp.subpaths.push_back(sp);
}

void shape_polygon(VectorPath& vp, float cx, float cy, float r, int sides, float rotDeg) {
    SubPath sp;sp.closed=true;
    float a0=rotDeg*0.0174533f-M_PI/2;
    for(int i=0;i<sides;++i){
        float a=a0+i*2*M_PI/sides;
        PathPoint p;p.x=cx+std::cos(a)*r;p.y=cy+std::sin(a)*r;
        sp.points.push_back(p);
    }
    vp.subpaths.push_back(sp);
}

void shape_star(VectorPath& vp, float cx, float cy, float r1, float r2, int points, float rotDeg) {
    SubPath sp;sp.closed=true;
    float a0=rotDeg*0.0174533f-M_PI/2;
    for(int i=0;i<points*2;++i){
        float r=(i%2==0)?r1:r2;
        float a=a0+i*M_PI/points;
        PathPoint p;p.x=cx+std::cos(a)*r;p.y=cy+std::sin(a)*r;
        sp.points.push_back(p);
    }
    vp.subpaths.push_back(sp);
}

void shape_line(VectorPath& vp, float x0, float y0, float x1, float y1, float width) {
    SubPath sp;sp.closed=false;
    PathPoint a;a.x=x0;a.y=y0;PathPoint b;b.x=x1;b.y=y1;
    sp.points={a,b};
    vp.subpaths.push_back(sp);
    (void)width;
}

static bool insidePoly(const std::vector<float>& poly,float x,float y){
    bool in=false;size_t n=poly.size()/2;
    for(size_t i=0,j=n-1;i<n;j=i++){
        float xi=poly[i*2],yi=poly[i*2+1],xj=poly[j*2],yj=poly[j*2+1];
        if(((yi>y)!=(yj>y))&&(x<(xj-xi)*(y-yi)/(yj-yi+1e-9f)+xi))in=!in;
    }
    return in;
}

void boolean_paths(const VectorPath& a, const VectorPath& b, VectorPath& out, int op) {
    out.subpaths.clear();
    std::vector<std::vector<float>> pa,pb;
    path_flatten(a,pa,2.0f);path_flatten(b,pb,2.0f);
    float minX=1e9,minY=1e9,maxX=-1e9,maxY=-1e9;
    for(auto&p:pa)for(size_t i=0;i<p.size();i+=2){minX=std::min(minX,p[i]);minY=std::min(minY,p[i+1]);maxX=std::max(maxX,p[i]);maxY=std::max(maxY,p[i+1]);}
    for(auto&p:pb)for(size_t i=0;i<p.size();i+=2){minX=std::min(minX,p[i]);minY=std::min(minY,p[i+1]);maxX=std::max(maxX,p[i]);maxY=std::max(maxY,p[i+1]);}
    if(minX>maxX)return;
    int gridW=256,gridH=256;
    std::vector<char> maskA(gridW*gridH,0),maskB(gridW*gridH,0);
    for(int gy=0;gy<gridH;++gy)for(int gx=0;gx<gridW;++gx){
        float x=minX+(gx+0.5f)/gridW*(maxX-minX),y=minY+(gy+0.5f)/gridH*(maxY-minY);
        bool ina=false,inb=false;
        for(auto&p:pa)if(insidePoly(p,x,y))ina=!ina;
        for(auto&p:pb)if(insidePoly(p,x,y))inb=!inb;
        maskA[gy*gridW+gx]=ina;maskB[gy*gridW+gx]=inb;
    }
    std::vector<char> res(gridW*gridH,0);
    for(int i=0;i<gridW*gridH;++i){
        switch(op){
            case 0:res[i]=maskA[i]||maskB[i];break;
            case 1:res[i]=maskA[i]&&!maskB[i];break;
            case 2:res[i]=maskA[i]&&maskB[i];break;
            default:res[i]=maskA[i]!=maskB[i];break;
        }
    }
    SubPath sp;sp.closed=true;
    for(int gy=0;gy<gridH;++gy)for(int gx=0;gx<gridW;++gx){
        if(res[gy*gridW+gx]){
            bool edge=gx==0||gy==0||gx==gridW-1||gy==gridH-1||!res[gy*gridW+gx-1]||!res[gy*gridW+gx+1]||!res[(gy-1)*gridW+gx]||!res[(gy+1)*gridW+gx];
            if(edge){
                float x=minX+(gx+0.5f)/gridW*(maxX-minX),y=minY+(gy+0.5f)/gridH*(maxY-minY);
                PathPoint p;p.x=x;p.y=y;sp.points.push_back(p);
            }
        }
    }
    if(!sp.points.empty())out.subpaths.push_back(sp);
}

void rasterize_path(const VectorPath& vp, Image& layer, const std::vector<float>* mask, const Color& fill, bool antiAlias, float feather) {
    std::vector<std::vector<float>> polys;
    path_flatten(vp,polys,1.0f);
    uint32_t w=layer.width,h=layer.height;
    std::vector<float> cov(static_cast<size_t>(w)*h,0.0f);
    for(uint32_t y=0;y<h;++y){
        std::vector<float> xs;
        float py=y+0.5f;
        for(auto&p:polys){
            size_t n=p.size()/2;
            for(size_t i=0;i<n;++i){
                float x0=p[i*2],y0=p[i*2+1],x1=p[((i+1)%n)*2],y1=p[((i+1)%n)*2+1];
                if((y0<=py&&y1>py)||(y1<=py&&y0>py)){
                    float t=(py-y0)/(y1-y0);
                    xs.push_back(x0+t*(x1-x0));
                }
            }
        }
        std::sort(xs.begin(),xs.end());
        for(size_t k=0;k+1<xs.size();k+=2){
            int xa=std::max(0,(int)xs[k]),xb=std::min((int)w-1,(int)xs[k+1]);
            for(int x=xa;x<=xb;++x)cov[y*w+x]=1.0f;
        }
    }
    if(antiAlias){
        std::vector<float> tmp=cov;
        for(uint32_t y=1;y+1<h;++y)for(uint32_t x=1;x+1<w;++x){
            size_t i=y*w+x;
            cov[i]=std::max(cov[i],(tmp[i-1]+tmp[i+1]+tmp[i-w]+tmp[i+w])*0.25f*0.8f);
        }
    }
    if(feather>0){
        std::vector<float> tmp=cov;
        int r=(int)feather;
        for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
            float sum=0;int cnt=0;
            for(int dy=-r;dy<=r;++dy)for(int dx=-r;dx<=r;++dx){
                int sx=std::max(0,std::min((int)w-1,x+dx)),sy=std::max(0,std::min((int)h-1,y+dy));
                sum+=tmp[sy*w+sx];cnt++;
            }
            cov[y*w+x]=sum/cnt;
        }
    }
    for(uint32_t y=0;y<h;++y)for(uint32_t x=0;x<w;++x){
        size_t i=y*w+x;
        float m=cov[i]*(mask?(*mask)[i]:1.0f)*fill.a;
        if(m<=0)continue;
        size_t idx=i*4;
        layer.pixels[idx]=layer.pixels[idx]*(1-m)+fill.r*m;
        layer.pixels[idx+1]=layer.pixels[idx+1]*(1-m)+fill.g*m;
        layer.pixels[idx+2]=layer.pixels[idx+2]*(1-m)+fill.b*m;
        layer.pixels[idx+3]=std::min(1.0f,layer.pixels[idx+3]+m);
    }
}

static float cl2(float v){return v<0?0:(v>1?1:v);}

void stroke_path(const VectorPath& vp, Image& layer, const std::vector<float>* mask, const Color& c, float width, bool antiAlias) {
    std::vector<std::vector<float>> polys;
    path_flatten(vp,polys,1.0f);
    uint32_t w=layer.width,h=layer.height;
    float half=width*0.5f;
    for(auto&p:polys){
        size_t n=p.size()/2;
        for(size_t i=0;i+1<n;++i){
            float x0=p[i*2],y0=p[i*2+1],x1=p[(i+1)*2],y1=p[(i+1)*2+1];
            float len=std::hypot(x1-x0,y1-y0);
            int steps=std::max(1,(int)len);
            for(int s=0;s<=steps;++s){
                float t=s/steps,x=x0+(x1-x0)*t,y=y0+(y1-y0)*t;
                int xa=std::max(0,(int)(x-half)),xb=std::min((int)w-1,(int)(x+half));
                int ya=std::max(0,(int)(y-half)),yb=std::min((int)h-1,(int)(y+half));
                for(int yy=ya;yy<=yb;++yy)for(int xx=xa;xx<=xb;++xx){
                    float d=std::hypot(xx-x,yy-y);
                    float cov=d<=half?(antiAlias?cl2((half-d)/std::max(0.5f,half*0.3f)):1.0f):0.0f;
                    if(cov<=0)continue;
                    size_t idx=((static_cast<size_t>(yy)*w+xx))*4;
                    float m=cov*c.a*(mask?(*mask)[yy*w+xx]:1.0f);
                    layer.pixels[idx]=layer.pixels[idx]*(1-m)+c.r*m;
                    layer.pixels[idx+1]=layer.pixels[idx+1]*(1-m)+c.g*m;
                    layer.pixels[idx+2]=layer.pixels[idx+2]*(1-m)+c.b*m;
                    layer.pixels[idx+3]=std::min(1.0f,layer.pixels[idx+3]+m);
                }
            }
        }
    }
}

std::string paths_to_json(const std::vector<VectorPath>& paths) {
    std::ostringstream os;
    os<<"[";
    for(size_t pi=0;pi<paths.size();++pi){
        const auto&vp=paths[pi];
        if(pi)os<<",";
        os<<"{\"subpaths\":[";
        for(size_t si=0;si<vp.subpaths.size();++si){
            const auto&sp=vp.subpaths[si];
            if(si)os<<",";
            os<<"{\"closed\":"<<(sp.closed?"true":"false")<<",\"points\":[";
            for(size_t i=0;i<sp.points.size();++i){
                const auto&p=sp.points[i];
                if(i)os<<",";
                os<<"{"<<p.x<<","<<p.y<<","<<p.inX<<","<<p.inY<<","<<p.outX<<","<<p.outY<<"}";
            }
            os<<"]}";
        }
        os<<"],\"fillRule\":"<<vp.fillRule<<"}";
    }
    os<<"]";
    return os.str();
}

static bool parseDouble(const char*& cur,double& out){
    while(*cur==' '||*cur==',')++cur;
    char* end=nullptr;
    out=strtod(cur,&end);
    if(end==cur)return false;
    cur=end;return true;
}

bool paths_from_json(const std::string& json, std::vector<VectorPath>& paths) {
    paths.clear();
    const char* cur=json.c_str();
    while(*cur&&*cur!='[')++cur;
    if(*cur!='[')return false;
    ++cur;
    while(*cur&&*cur!=']'){
        while(*cur&&*cur!='{')++cur;
        if(*cur!='{')break;
        VectorPath vp;
        const char* spStart=strstr(cur,"\"subpaths\"");
        if(!spStart)break;
        cur=spStart;while(*cur!='[')++cur;++cur;
        while(*cur&&*cur!=']'||(*cur==']'&&false)){
            while(*cur&&*cur!='{')++cur;if(*cur!='{')break;
            SubPath sp;
            const char* pc=strstr(cur,"\"closed\"");
            if(pc)sp.closed=pc[8]=='t';
            const char* pp=strstr(cur,"\"points\"");
            if(!pp)break;
            cur=pp;while(*cur!='[')++cur;++cur;
            while(*cur&&*cur!=']'){
                while(*cur&&*cur!='{')++cur;if(*cur!='{')break;
                ++cur;
                PathPoint p;double vals[6]={0,0,0,0,0,0};
                for(int i=0;i<6;++i)if(!parseDouble(cur,vals[i]))break;
                p.x=vals[0];p.y=vals[1];p.inX=vals[2];p.inY=vals[3];p.outX=vals[4];p.outY=vals[5];
                sp.points.push_back(p);
                while(*cur&&*cur!='}')++cur;if(*cur=='}')++cur;
            }
            if(*cur==']')++cur;
            vp.subpaths.push_back(sp);
            while(*cur&&*cur!='}'&&*cur!=']')++cur;
            if(*cur=='}')++cur;
            if(*cur==']'){++cur;break;}
        }
        paths.push_back(vp);
        while(*cur&&*cur!=','&&*cur!=']')++cur;
        if(*cur==',')++cur;
    }
    return !paths.empty();
}

struct Glyph {
    int codepoint;
    float advance;
    std::vector<std::pair<float,float>> segments;
};

static bool glyphFor(char ch, float& adv, std::vector<std::pair<float,float>>& segs) {
    struct Def { char c; float adv; const char* segs; };
    static const Def defs[] = {
        {'A',0.7f,"0,1 0.35,0 0.7,1 0.15,0.6 0.55,0.6"},
        {'B',0.65f,"0,0 0,1 0.4,1 0.6,0.8 0.4,0.5 0.6,0.2 0.4,0 0,0 0,0.5 0.4,0.5"},
        {'C',0.65f,"0.6,0.85 0.3,1 0,0.7 0,0.3 0.3,0 0.6,0.15"},
        {'D',0.65f,"0,0 0,1 0.35,1 0.65,0.7 0.65,0.3 0.35,0 0,0"},
        {'E',0.6f,"0.6,0 0,0 0,1 0.6,1 0,0.5 0.5,0.5"},
        {'F',0.6f,"0.6,0 0,0 0,1 0.6,1 0,0.5 0.5,0.5"},
        {'G',0.7f,"0.6,0.85 0.3,1 0,0.7 0,0.3 0.3,0 0.6,0.3 0.6,0.5 0.35,0.5"},
        {'H',0.7f,"0,0 0,1 0,0.5 0.7,0.5 0.7,0 0.7,1"},
        {'I',0.3f,"0.15,0 0.15,1 0,0 0.3,0 0,1 0.3,1"},
        {'J',0.55f,"0.5,0 0.5,0.8 0.3,1 0.1,0.8 0.1,0.6"},
        {'K',0.65f,"0,0 0,1 0,0.5 0.6,0 0.2,0.5 0.6,1"},
        {'L',0.6f,"0,0 0,1 0,0 0.6,0"},
        {'M',0.8f,"0,0 0,1 0.4,0.4 0.8,1 0.8,0"},
        {'N',0.7f,"0,0 0,1 0.7,0 0.7,1"},
        {'O',0.7f,"0.35,0 0,0.3 0,0.7 0.35,1 0.7,0.7 0.7,0.3 0.35,0"},
        {'P',0.65f,"0,0 0,1 0.4,1 0.65,0.75 0.4,0.5 0,0.5"},
        {'Q',0.7f,"0.35,0 0,0.3 0,0.7 0.35,1 0.7,0.7 0.7,0.3 0.35,0 0.5,0.4 0.75,0"},
        {'R',0.65f,"0,0 0,1 0.4,1 0.65,0.75 0.4,0.5 0,0.5 0.4,0.5 0.65,0"},
        {'S',0.65f,"0.6,0.15 0.3,0 0.05,0.25 0.3,0.5 0.6,0.75 0.35,1 0.05,0.85"},
        {'T',0.65f,"0,1 0.65,1 0.325,1 0.325,0"},
        {'U',0.7f,"0,1 0,0.3 0.35,0 0.7,0.3 0.7,1"},
        {'V',0.7f,"0,1 0.35,0 0.7,1"},
        {'W',0.9f,"0,1 0.22,0 0.45,0.7 0.68,0 0.9,1"},
        {'X',0.65f,"0,0 0.65,1 0.65,0 0,1"},
        {'Y',0.65f,"0,1 0.325,0.5 0.65,1 0.325,0.5 0.325,0"},
        {'Z',0.6f,"0,1 0.6,1 0,0 0.6,0"},
        {'0',0.65f,"0.325,0 0,0.3 0,0.7 0.325,1 0.65,0.7 0.65,0.3 0.325,0 0.1,0.9 0.55,0.1"},
        {'1',0.65f,"0.2,0.8 0.4,1 0.4,0 0.2,0 0.6,0"},
        {'2',0.65f,"0,0.8 0.3,1 0.6,0.7 0,0 0.6,0"},
        {'3',0.65f,"0,0.9 0.3,1 0.55,0.75 0.3,0.5 0.55,0.25 0.3,0 0,0.1"},
        {'4',0.65f,"0.45,1 0,0.3 0.65,0.3 0.45,1 0.45,0"},
        {'5',0.65f,"0.6,1 0.15,1 0.1,0.5 0.45,0.35 0.6,0.15 0.3,0 0,0.15"},
        {'6',0.65f,"0.5,1 0.2,0.85 0.1,0.5 0.1,0.15 0.4,0 0.6,0.2 0.4,0.5 0.1,0.45"},
        {'7',0.65f,"0,1 0.6,1 0.2,0"},
        {'8',0.65f,"0.325,0.5 0.1,0.75 0.325,1 0.55,0.75 0.325,0.5 0.1,0.25 0.325,0 0.55,0.25 0.325,0.5"},
        {'9',0.65f,"0.2,0 0.5,0.15 0.6,0.5 0.6,0.85 0.3,1 0.1,0.8 0.3,0.5 0.6,0.55"},
        {'.',0.3f,"0.15,0 0.15,0.05"},
        {',',0.3f,"0.2,0.1 0.1,-0.1"},
        {'-',0.5f,"0.1,0.55 0.4,0.55"},
        {'+',0.6f,"0.1,0.5 0.5,0.5 0.3,0.3 0.3,0.7"},
        {'=',0.6f,"0.1,0.4 0.5,0.4 0.1,0.6 0.5,0.6"},
        {'(',0.4f,"0.3,1 0.1,0.7 0.1,0.3 0.3,0"},
        {')',0.4f,"0.1,1 0.3,0.7 0.3,0.3 0.1,0"},
        {'!',0.3f,"0.15,1 0.15,0.4 0.15,0.1 0.15,0.05"},
        {'?',0.6f,"0,0.85 0.3,1 0.55,0.75 0.3,0.5 0.3,0.4 0.3,0.1 0.3,0.05"},
        {':',0.3f,"0.15,0.6 0.15,0.55 0.15,0.15 0.15,0.1"},
        {';',0.3f,"0.15,0.6 0.15,0.55 0.2,0.15 0.1,-0.05"},
        {'/',0.5f,"0,0 0.5,1"},
        {'\\',0.5f,"0,1 0.5,0"},
        {'\'',0.3f,"0.15,1 0.15,0.85"},
        {'"',0.5f,"0.15,1 0.15,0.85 0.35,1 0.35,0.85"},
        {'#',0.7f,"0.15,1 0.25,0 0.45,1 0.55,0 0,0.7 0.7,0.7 0,0.3 0.7,0.3"},
        {'%',0.8f,"0.1,0.9 0.15,1 0.2,0.9 0.15,0.8 0.1,0.9 0.6,0.2 0.65,0.3 0.7,0.2 0.65,0.1 0.6,0.2 0.1,0.1 0.7,0.9"},
        {' ',0.4f,""}
    };
    char up=ch;
    if(ch>='a'&&ch<='z')up=ch-32;
    for(const auto&d:defs){
        if(d.c==up){
            adv=d.adv;segs.clear();
            const char* s=d.segs;
            float px=0,py=0;
            while(*s){
                char* end=nullptr;
                px=strtof(s,&end);s=end;
                if(*s==','){++s;}
                py=strtof(s,&end);s=end;
                segs.push_back({px,py});
                while(*s==' ')++s;
            }
            return true;
        }
    }
    adv=0.5f;segs.clear();
    segs.push_back({0.1f,0.1f});segs.push_back({0.4f,0.9f});
    return true;
}

void text_to_layer(Image& layer, const std::string& utf8, float pxSize, const Color& color, float x, float y, bool vertical) {
    std::string ascii;
    for(unsigned char c:utf8){
        if(c>=32&&c<127)ascii.push_back((char)c);
        else if(c=='\n')ascii.push_back('\n');
    }
    float scale=pxSize;
    float penX=x,penY=y;
    float lineH=pxSize*1.2f;
    for(char ch:ascii){
        if(ch=='\n'){penX=x;penY-=lineH;continue;}
        float adv;std::vector<std::pair<float,float>> segs;
        glyphFor(ch,adv,segs);
        for(size_t i=0;i+1<segs.size();++i){
            float x0=penX+segs[i].first*scale,y0=penY-segs[i].second*scale;
            float x1=penX+segs[i+1].first*scale,y1=penY-segs[i+1].second*scale;
            if(vertical){std::swap(x0,y0);std::swap(x1,y1);}
            VectorPath vp;shape_line(vp,x0,y0,x1,y1,pxSize*0.12f);
            stroke_path(vp,layer,nullptr,color,pxSize*0.12f,true);
        }
        penX+=adv*scale;
    }
}

float measure_text(const std::string& utf8, float pxSize, float& outWidth, float& outHeight) {
    float maxLine=0,line=0,height=pxSize*1.2f;
    for(unsigned char c:utf8){
        if(c=='\n'){maxLine=std::max(maxLine,line);line=0;height+=pxSize*1.2f;continue;}
        if(c<32||c>=127)continue;
        float adv;std::vector<std::pair<float,float>> segs;
        glyphFor((char)c,adv,segs);
        line+=adv*pxSize;
    }
    maxLine=std::max(maxLine,line);
    outWidth=maxLine;outHeight=height;
    return maxLine;
}

}
