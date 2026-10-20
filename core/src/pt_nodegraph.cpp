#include "pt_nodegraph.h"
#include "pt_adjust.h"
#include "pt_filters.h"
#include "pt_math.h"
#include "pt_blend.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <cstring>
#include <unordered_map>
#include <functional>

namespace pt {

static inline float cl(float v){return v<0?0:(v>1?1:v);}

struct NodeInput {
    Image img;
    bool valid=false;
};

static Image runNode(const NodeGraph::Node& node, const std::vector<Image>& inputs, uint32_t W, uint32_t H) {
    if (node.type=="input") return !inputs.empty()?inputs[0]:create_image(W,H,ChannelKind::RGB,0);
    if (node.type=="constColor") {
        Color c{node.params.size()>0?node.params[0]:1,node.params.size()>1?node.params[1]:0,node.params.size()>2?node.params[2]:0,node.params.size()>3?node.params[3]:1};
        Image out=create_image(W,H,ChannelKind::RGB,0);
        for(size_t i=0;i<out.pixelCount();++i){out.pixels[i*4]=c.r;out.pixels[i*4+1]=c.g;out.pixels[i*4+2]=c.b;out.pixels[i*4+3]=c.a;}
        return out;
    }
    if (node.type=="blend") {
        if(inputs.size()<2)return inputs.empty()?create_image(W,H,ChannelKind::RGB,0):inputs[0];
        Image out=clone_image(inputs[0]);
        BlendMode mode=(BlendMode)(int)(node.params.empty()?0:node.params[0]);
        float op=node.params.size()>1?node.params[1]:1.0f;
        blend_images(out.data(),inputs[1].data(),out.pixelCount(),mode,op);
        return out;
    }
    if (node.type=="levels") {
        Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);
        float a=node.params.size()>0?node.params[0]:0,b=node.params.size()>1?node.params[1]:1,g=node.params.size()>2?node.params[2]:1,c=node.params.size()>3?node.params[3]:0,d=node.params.size()>4?node.params[4]:1;
        adjust_levels(out,a,b,g,c,d);
        return out;
    }
    if (node.type=="curves") {
        Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);
        std::vector<float> lut(256);
        float sh=node.params.size()>0?node.params[0]:0,mid=node.params.size()>1?node.params[1]:0.5f,hi=node.params.size()>2?node.params[2]:1;
        for(int i=0;i<256;++i){float t=i/255.0f;lut[i]=cl(sh+(hi-sh)*t+mid*sin(t*M_PI));}
        adjust_curves(out,lut,lut,lut);
        return out;
    }
    if (node.type=="invert") {Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);adjust_invert(out);return out;}
    if (node.type=="hueSat") {
        Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);
        adjust_hue_saturation(out,node.params.size()>0?node.params[0]:0,node.params.size()>1?node.params[1]:0,node.params.size()>2?node.params[2]:0,6);
        return out;
    }
    if (node.type=="blur") {Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);gaussian_blur(out,node.params.empty()?4.0f:node.params[0]);return out;}
    if (node.type=="sharpen") {Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);unsharp_mask(out,node.params.size()>0?node.params[0]:1.0f,node.params.size()>1?node.params[1]:2.0f,0);return out;}
    if (node.type=="noise") {Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);add_noise(out,node.params.size()>0?node.params[0]:0.1f,false,false,node.params.size()>1?node.params[1]:0);return out;}
    if (node.type=="perlinTex") {
        Image out;
        generate_procedural_texture(out,W,H,0,node.params.size()>0?node.params[0]:64.0f,node.params.size()>1?node.params[1]:4,2.0f,0.5f,node.params.size()>2?node.params[2]:0,(int)(node.params.size()>3?node.params[3]:0),{0,0,0,1},{1,1,1,1});
        return out;
    }
    if (node.type=="simplexTex") {
        Image out;
        generate_procedural_texture(out,W,H,1,node.params.size()>0?node.params[0]:64.0f,node.params.size()>1?node.params[1]:4,2.0f,0.5f,node.params.size()>2?node.params[2]:0,(int)(node.params.size()>3?node.params[3]:0),{0,0,0,1},{1,1,1,1});
        return out;
    }
    if (node.type=="worleyTex") {
        Image out;
        generate_procedural_texture(out,W,H,2,node.params.size()>0?node.params[0]:32.0f,1,2.0f,0.5f,0,(int)(node.params.size()>3?node.params[3]:0),{0,0,0,1},{1,1,1,1});
        return out;
    }
    if (node.type=="transform") {
        Image out;
        if(inputs.empty())return create_image(W,H,ChannelKind::RGB,0);
        float a=node.params.size()>0?node.params[0]:0,sx=node.params.size()>1?node.params[1]:1,sy=node.params.size()>2?node.params[2]:1,tx=node.params.size()>3?node.params[3]:0,ty=node.params.size()>4?node.params[4]:0;
        float rad=a*0.0174533f;
        transform_affine(inputs[0],out,cos(rad)*sx,-sin(rad)*sy,tx,sin(rad)*sx,cos(rad)*sy,ty);
        return out;
    }
    if (node.type=="threshold") {Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);adjust_threshold(out,node.params.empty()?0.5f:node.params[0]);return out;}
    if (node.type=="posterize") {Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);adjust_posterize(out,(int)(node.params.empty()?4.0f:node.params[0]));return out;}
    if (node.type=="grayscale") {Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);desaturate(out);return out;}
    if (node.type=="sepia") {
        Image out=inputs.empty()?create_image(W,H,ChannelKind::RGB,0):clone_image(inputs[0]);
        for(size_t i=0;i<out.pixelCount();++i){
            float*r=&out.pixels[i*4];
            float nr=cl(r[0]*0.393+r[1]*0.769+r[2]*0.189);
            float ng=cl(r[0]*0.349+r[1]*0.686+r[2]*0.168);
            float nb=cl(r[0]*0.272+r[1]*0.534+r[2]*0.131);
            r[0]=nr;r[1]=ng;r[2]=nb;
        }
        return out;
    }
    if (node.type=="output") return inputs.empty()?create_image(W,H,ChannelKind::RGB,0):inputs[0];
    return inputs.empty()?create_image(W,H,ChannelKind::RGB,0):inputs[0];
}

bool node_graph_run(const NodeGraph& g, const Image& input, Image& output) {
    std::unordered_map<int,std::vector<int>> incoming;
    for(size_t i=0;i<g.edges.size();++i){incoming[g.edges[i].toNode].push_back((int)i);}
    std::unordered_map<int,Image> cache;
    std::function<Image(int)> eval=[&](int nodeId)->Image{
        auto it=cache.find(nodeId);
        if(it!=cache.end())return it->second;
        const NodeGraph::Node* node=nullptr;
        for(auto&n:g.nodes)if(n.id==nodeId){node=&n;break;}
        if(!node)return create_image(input.width,input.height,ChannelKind::RGB,0);
        std::vector<Image> ins;
        auto inIt=incoming.find(nodeId);
        if(inIt!=incoming.end()){
            for(int ei:inIt->second){
                const auto&e=g.edges[ei];
                ins.push_back(eval(e.fromNode));
            }
        }
        if(ins.empty()&&node->type!="input"&&node->type!="constColor"&&node->type!="perlinTex"&&node->type!="simplexTex"&&node->type!="worleyTex")ins.push_back(input);
        else if(node->type=="input")ins={input};
        Image res=runNode(*node,ins,input.width,input.height);
        cache[nodeId]=res;
        return res;
    };
    int outId=-1;
    for(auto&n:g.nodes)if(n.type=="output"){outId=n.id;break;}
    if(outId<0){for(auto it=g.nodes.rbegin();it!=g.nodes.rend();++it){outId=it->id;break;}}
    if(outId<0)return false;
    output=eval(outId);
    return true;
}

std::string node_graph_to_json(const NodeGraph& g) {
    std::ostringstream os;
    os<<"{\"nodes\":[";
    for(size_t i=0;i<g.nodes.size();++i){
        const auto&n=g.nodes[i];
        if(i)os<<",";
        os<<"{\"id\":"<<n.id<<",\"type\":\""<<n.type<<"\",\"params\":[";
        for(size_t k=0;k<n.params.size();++k){if(k)os<<",";char b[32];snprintf(b,sizeof b,"%.6g",n.params[k]);os<<b;}
        os<<"],\"str\":\"";
        for(char c:n.strParam){if(c=='"'||c=='\\')os<<'\\';os<<c;}
        os<<"\"}";
    }
    os<<"],\"edges\":[";
    for(size_t i=0;i<g.edges.size();++i){
        const auto&e=g.edges[i];
        if(i)os<<",";
        os<<"{"<<e.fromNode<<","<<e.fromSocket<<","<<e.toNode<<","<<e.toSocket<<"}";
    }
    os<<"]}";
    return os.str();
}

bool node_graph_from_json(const std::string& json, NodeGraph& g) {
    g.nodes.clear();g.edges.clear();
    const char* cur=json.c_str();
    const char* ns=strstr(cur,"\"nodes\"");
    if(!ns)return false;
    cur=ns;while(*cur!='[')++cur;++cur;
    while(*cur&&*cur!=']'){
        while(*cur&&*cur!='{')++cur;if(*cur!='{')break;
        NodeGraph::Node n;
        ++cur;
        while(*cur&&*cur!='}'&&*cur){
            while(*cur&&*cur!='"'&&*cur!='}')++cur;
            if(*cur!='\"')break;
            std::string key;
            ++cur;while(*cur&&*cur!='\"'){key.push_back(*cur);++cur;}if(*cur=='"')++cur;
            while(*cur&&*cur!=':')++cur;if(*cur==':')++cur;
            while(*cur==' ')++cur;
            if(key=="id"){char*e;n.id=(int)strtol(cur,&e,10);cur=e;}
            else if(key=="type"){++cur;while(*cur&&*cur!='\"'){n.type.push_back(*cur);++cur;}if(*cur=='"')++cur;}
            else if(key=="params"){++cur;while(*cur&&*cur!=']'){char*e;double v=strtod(cur,&e);if(e==cur)break;n.params.push_back((float)v);cur=e;while(*cur&&*cur!=','&&*cur!=']')++cur;if(*cur==',')++cur;}if(*cur==']')++cur;}
            else if(key=="str"){++cur;while(*cur&&*cur!='\"'){if(*cur=='\\')++cur;n.strParam.push_back(*cur);++cur;}if(*cur=='"')++cur;}
            else{if(*cur=='"'){++cur;while(*cur&&*cur!='\"')++cur;if(*cur=='"')++cur;}else if(*cur=='['){int d=0;do{if(*cur=='[')d++;else if(*cur==']')d--;++cur;}while(d>0&&*cur);}else{char*e;strtof(cur,&e);cur=e;}}
            while(*cur&&*cur!=','&&*cur!='}')++cur;if(*cur==',')++cur;
        }
        if(*cur=='}')++cur;
        g.nodes.push_back(n);
        while(*cur&&*cur!=','&&*cur!=']')++cur;if(*cur==',')++cur;
    }
    if(*cur==']')++cur;
    const char* es=strstr(json.c_str(),"\"edges\"");
    if(es){
        cur=es;while(*cur!='[')++cur;++cur;
        while(*cur&&*cur!=']'){
            while(*cur&&*cur!='{')++cur;if(*cur!='{')break;
            ++cur;
            NodeGraph::Edge e;
            char*ep;
            e.fromNode=(int)strtol(cur,&ep,10);cur=ep;
            while(*cur&&*cur!=',')++cur;if(*cur==',')++cur;
            e.fromSocket=(int)strtol(cur,&ep,10);cur=ep;
            while(*cur&&*cur!=',')++cur;if(*cur==',')++cur;
            e.toNode=(int)strtol(cur,&ep,10);cur=ep;
            while(*cur&&*cur!=',')++cur;if(*cur==',')++cur;
            e.toSocket=(int)strtol(cur,&ep,10);cur=ep;
            while(*cur&&*cur!='}')++cur;if(*cur=='}')++cur;
            g.edges.push_back(e);
            while(*cur&&*cur!=','&&*cur!=']')++cur;if(*cur==',')++cur;
        }
    }
    return !g.nodes.empty();
}

void HistoryStack::push(const std::string& label, const std::string& docJson) {
    Snapshot s;s.label=label;s.compressedDoc.assign(docJson.begin(),docJson.end());
    if(index_+1<(int)entries_.size())entries_.resize(index_+1);
    entries_.push_back(std::move(s));
    index_=(int)entries_.size()-1;
    if(entries_.size()>64){entries_.erase(entries_.begin());index_--;}
}

std::string HistoryStack::undo() {
    if(!canUndo())return "";
    index_--;
    return entries_[index_].compressedDoc.empty()?"":std::string(entries_[index_].compressedDoc.begin(),entries_[index_].compressedDoc.end());
}

std::string HistoryStack::redo() {
    if(!canRedo())return "";
    index_++;
    return entries_[index_].compressedDoc.empty()?"":std::string(entries_[index_].compressedDoc.begin(),entries_[index_].compressedDoc.end());
}

std::string HistoryStack::current() const {
    if(index_<0||index_>=(int)entries_.size())return "";
    return std::string(entries_[index_].compressedDoc.begin(),entries_[index_].compressedDoc.end());
}

void HistoryStack::clear() { entries_.clear(); index_=-1; }

static uint32_t crcTable[256];
static bool crcInit=false;
static void initCrc(){if(crcInit)return;for(uint32_t i=0;i<256;++i){uint32_t c=i;for(int k=0;k<8;++k)c=(c&1)?0xEDB88320u^(c>>1):c>>1;crcTable[i]=c;}crcInit=true;}

uint32_t crc32_compute(const uint8_t* data,size_t len){initCrc();uint32_t c=0xFFFFFFFF;for(size_t i=0;i<len;++i)c=crcTable[(c^data[i])&0xFF]^(c>>8);return c^0xFFFFFFFF;}

bool zlib_compress(const std::vector<uint8_t>& in,std::vector<uint8_t>& out){
    out.clear();
    out.push_back(0x78);out.push_back(0x01);
    size_t pos=0;
    do{
        size_t block=std::min<size_t>(in.size()-pos,65535);
        bool last=pos+block>=in.size();
        out.push_back(last?1:0);
        uint16_t len=(uint16_t)block,nlen=~len;
        out.push_back(len&0xFF);out.push_back(len>>8);out.push_back(nlen&0xFF);out.push_back(nlen>>8);
        out.insert(out.end(),in.begin()+pos,in.begin()+pos+block);
        pos+=block;
    }while(pos<in.size());
    uint32_t a=1,b=0;
    for(uint8_t v:in){a=(a+v)%65521;b=(b+a)%65521;}
    uint32_t ad=(b<<16)|a;
    out.push_back((ad>>24)&0xFF);out.push_back((ad>>16)&0xFF);out.push_back((ad>>8)&0xFF);out.push_back(ad&0xFF);
    return true;
}

bool zlib_decompress(const std::vector<uint8_t>& in,std::vector<uint8_t>& out){
    out.clear();
    if(in.size()<6)return false;
    size_t pos=2;
    while(pos<in.size()){
        uint8_t hdr=in[pos++];
        bool final=hdr&1;int type=(hdr>>1)&3;
        if(type==0){
            pos+=2;
            uint16_t len=in[pos]|(in[pos+1]<<8);pos+=4;
            out.insert(out.end(),in.begin()+pos,in.begin()+pos+len);
            pos+=len;
        }else return false;
        if(final)break;
    }
    return true;
}

}
