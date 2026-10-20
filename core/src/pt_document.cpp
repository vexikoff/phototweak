#include "pt_document.h"
#include "pt_blend.h"
#include "pt_selection.h"
#include <sstream>
#include <fstream>
#include <cstring>
#include <random>
#include <chrono>
#include <functional>
#include <algorithm>

namespace pt {

std::string new_layer_id() {
    static std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::ostringstream os;
    os << std::hex << rng();
    return os.str();
}

Document::Document(uint32_t w, uint32_t h, const std::string& name, ChannelKind ch)
    : width(w), height(h), title(name), channelKind(ch) {}

std::shared_ptr<LayerNode> Document::addRasterLayer(const std::string& name) {
    auto l = std::make_shared<LayerNode>();
    l->id = new_layer_id();
    l->name = name;
    l->kind = 0;
    l->pixels = create_image(width, height, ChannelKind::RGB, 0.0f);
    for (size_t i = 0; i < l->pixels.pixelCount(); ++i) l->pixels.pixels[i*4+3] = 0.0f;
    roots.push_back(l);
    return l;
}

std::shared_ptr<LayerNode> Document::addGroup(const std::string& name) {
    auto g = std::make_shared<LayerNode>();
    g->id = new_layer_id();
    g->name = name;
    g->kind = 1;
    roots.push_back(g);
    return g;
}

std::shared_ptr<LayerNode> Document::addAdjustmentLayer(const std::string& name, int adjType) {
    auto l = std::make_shared<LayerNode>();
    l->id = new_layer_id();
    l->name = name;
    l->kind = 2;
    l->adjType = adjType;
    roots.push_back(l);
    return l;
}

std::shared_ptr<LayerNode> Document::addTextLayer(const std::string& name) {
    auto l = std::make_shared<LayerNode>();
    l->id = new_layer_id();
    l->name = name;
    l->kind = 3;
    l->pixels = create_image(width, height, ChannelKind::RGB, 0.0f);
    for (size_t i = 0; i < l->pixels.pixelCount(); ++i) l->pixels.pixels[i*4+3] = 0.0f;
    roots.push_back(l);
    return l;
}

std::shared_ptr<LayerNode> Document::addSmartObjectLayer(const std::string& name, const std::string& sourcePath) {
    auto l = std::make_shared<LayerNode>();
    l->id = new_layer_id();
    l->name = name;
    l->kind = 4;
    l->sourcePath = sourcePath;
    Image src;
    if (load_image_any(sourcePath, src)) {
        l->pixels = clone_image(src);
    } else {
        l->pixels = create_image(width, height, ChannelKind::RGB, 0.0f);
        for (size_t i = 0; i < l->pixels.pixelCount(); ++i) l->pixels.pixels[i*4+3] = 0.0f;
    }
    roots.push_back(l);
    return l;
}

static std::shared_ptr<LayerNode> findInTree(const std::vector<std::shared_ptr<LayerNode>>& nodes, const std::string& id) {
    for (auto& n : nodes) {
        if (n->id == id) return n;
        if (!n->children.empty()) {
            auto r = findInTree(n->children, id);
            if (r) return r;
        }
    }
    return nullptr;
}

std::shared_ptr<LayerNode> Document::findLayer(const std::string& id) {
    return findInTree(roots, id);
}

bool Document::removeLayer(const std::string& id) {
    std::function<bool(std::vector<std::shared_ptr<LayerNode>>&)> rec = [&](std::vector<std::shared_ptr<LayerNode>>& v)->bool{
        for (auto it=v.begin();it!=v.end();++it) {
            if ((*it)->id==id) { v.erase(it); return true; }
            if (!(*it)->children.empty() && rec((*it)->children)) return true;
        }
        return false;
    };
    return rec(roots);
}

bool Document::moveLayer(const std::string& id, int newIndex) {
    std::shared_ptr<LayerNode> node;
    std::vector<std::shared_ptr<LayerNode>>* list=nullptr;
    std::function<bool(std::vector<std::shared_ptr<LayerNode>>&)> rec=[&](std::vector<std::shared_ptr<LayerNode>>& v)->bool{
        for(size_t i=0;i<v.size();++i){
            if(v[i]->id==id){node=v[i];list=&v;v.erase(v.begin()+i);return true;}
            if(rec(v[i]->children))return true;
        }
        return false;
    };
    if(!rec(roots)||!node||!list)return false;
    if(newIndex<0)newIndex=0;
    if(newIndex>(int)list->size())newIndex=list->size();
    list->insert(list->begin()+newIndex,node);
    return true;
}

static void applyAdjustment(Image& base, const LayerNode& adj) {
    Image work = clone_image(base);
    switch (adj.adjType) {
        case 0: if(adj.adjParams.size()>=5)adjust_levels(work,adj.adjParams[0],adj.adjParams[1],adj.adjParams[2],adj.adjParams[3],adj.adjParams[4]); break;
        case 1: if(!adj.curveLutR.empty())adjust_curves(work,adj.curveLutR,adj.curveLutG,adj.curveLutB); break;
        case 2: if(adj.adjParams.size()>=3)adjust_exposure(work,adj.adjParams[0],adj.adjParams[1],adj.adjParams[2]); break;
        case 3: if(adj.adjParams.size()>=3)adjust_hue_saturation(work,adj.adjParams[0],adj.adjParams[1],adj.adjParams[2],(int)adj.adjParams[3]); break;
        case 4: adjust_invert(work); break;
        case 5: if(adj.adjParams.size()>=2)adjust_posterize(work,(int)adj.adjParams[0]); break;
        case 6: if(adj.adjParams.size()>=1)adjust_threshold(work,adj.adjParams[0]); break;
        case 7: if(adj.adjParams.size()>=2)adjust_brightness_contrast(work,adj.adjParams[0],adj.adjParams[1]); break;
        case 8: if(adj.adjParams.size()>=2)adjust_vibrance(work,adj.adjParams[0],adj.adjParams[1]); break;
        default: break;
    }
    blend_images(base.data(), work.data(), base.pixelCount(), BlendMode::Normal, adj.opacity);
}

void compositeList(const std::vector<std::shared_ptr<LayerNode>>& layers, Image& target) {
    for (auto& layer : layers) {
        if (!layer->visible) continue;
        if (layer->isGroup()) {
            Image group = create_image(target.width, target.height, ChannelKind::RGB, 0.0f);
            for (size_t i = 0; i < group.pixelCount(); ++i) group.pixels[i*4+3] = 0.0f;
            compositeList(layer->children, group);
            if (layer->mask.enabled && layer->mask.data.width == target.width)
                blend_with_mask(target.data(), group.data(), layer->mask.data.data(), target.pixelCount(), layer->blend, layer->opacity);
            else
                blend_images(target.data(), group.data(), target.pixelCount(), layer->blend, layer->opacity);
            continue;
        }
        if (layer->kind == 2) { applyAdjustment(target, *layer); continue; }
        if (layer->pixels.width != target.width || layer->pixels.height != target.height) continue;
        const float* srcPixels = layer->pixels.data();
        std::vector<float> masked;
        if (layer->mask.enabled && layer->mask.data.width == target.width) {
            masked = layer->mask.data.pixels;
            srcPixels = layer->pixels.data();
            blend_with_mask(target.data(), srcPixels, masked.data(), target.pixelCount(), layer->blend, layer->opacity);
        } else {
            blend_images(target.data(), srcPixels, target.pixelCount(), layer->blend, layer->opacity);
        }
    }
}

void Document::flattenTo(Image& out) const {
    out = create_image(width, height, ChannelKind::RGB, 1.0f);
    std::vector<std::shared_ptr<LayerNode>> reversed = roots;
    std::reverse(reversed.begin(), reversed.end());
    compositeList(reversed, const_cast<Image&>(out));
}

void Document::compositeRegion(Image& out, int x0, int y0, int w, int h) const {
    flattenTo(out);
    (void)x0;(void)y0;(void)w;(void)h;
}

static std::string escJson(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c=='"'||c=='\\') { o.push_back('\\'); o.push_back(c); }
        else if (c=='\n') o += "\\n";
        else if (c=='\t') o += "\\t";
        else if ((unsigned char)c < 32) { char buf[8]; snprintf(buf,sizeof buf,"\\u%04x",(unsigned char)c); o+=buf; }
        else o.push_back(c);
    }
    return o;
}

static std::string num(float v) {
    char buf[32]; snprintf(buf,sizeof buf,"%.7g",v); return buf;
}

static void serializeLayer(const std::shared_ptr<LayerNode>& l, std::ostringstream& os) {
    os<<"{\"id\":\""<<l->id<<"\",\"name\":\""<<escJson(l->name)<<"\","
      <<"\"kind\":"<<l->kind<<",\"visible\":"<<(l->visible?"true":"false")<<","
      <<"\"opacity\":"<<num(l->opacity)<<",\"blend\":"<<(int)l->blend<<","
      <<"\"clipping\":"<<(l->clipping?"true":"false")<<","
      <<"\"x\":"<<l->x<<",\"y\":"<<l->y<<","
      <<"\"adjType\":"<<l->adjType<<","
      <<"\"text\":["<<num(l->textSize)<<","<<num(l->textColor.r)<<","<<num(l->textColor.g)<<","<<num(l->textColor.b)<<","<<num(l->textColor.a)<<","<<escJson(l->textContent)<<","<<escJson(l->sourcePath)<<","<<escJson(l->vectorPathsJson)<<"]";
    if (!l->curveLutR.empty()) {
        os<<",\"curves\":[";
        for(size_t i=0;i<l->curveLutR.size();++i){if(i)os<<",";os<<num(l->curveLutR[i]);}
        os<<",[";
        for(size_t i=0;i<l->curveLutG.size();++i){if(i)os<<",";os<<num(l->curveLutG[i]);}
        os<<",[";
        for(size_t i=0;i<l->curveLutB.size();++i){if(i)os<<",";os<<num(l->curveLutB[i]);}
        os<<"]]";
    }
    if (!l->adjParams.empty()) {
        os<<",\"adj\":[";
        for(size_t i=0;i<l->adjParams.size();++i){if(i)os<<",";os<<num(l->adjParams[i]);}
        os<<"]";
    }
    if (l->mask.enabled && l->mask.data.width>0) {
        os<<",\"maskW\":"<<l->mask.data.width<<",\"maskH\":"<<l->mask.data.width*0+l->mask.data.height<<",\"mask\":[";
        size_t step=std::max<size_t>(1,l->mask.data.pixelCount()/65536);
        for(size_t i=0;i<l->mask.data.pixelCount();i+=step){if(i)os<<",";os<<num(l->mask.data.pixels[i]);}
        os<<"]";
    }
    if (l->pixels.width>0) {
        os<<",\"pxW\":"<<l->pixels.width<<",\"pxH\":"<<l->pixels.height<<",\"px\":[";
        size_t total=l->pixels.pixels.size();
        for(size_t i=0;i<total;++i){if(i)os<<",";os<<num(l->pixels.pixels[i]);}
        os<<"]";
    }
    if (!l->children.empty()) {
        os<<",\"children\":[";
        for(size_t i=0;i<l->children.size();++i){if(i)os<<",";serializeLayer(l->children[i],os);}
        os<<"]";
    }
    os<<"}";
}

std::string Document::serialize() const {
    std::ostringstream os;
    os<<"{\"width\":"<<width<<",\"height\":"<<height<<",\"dpi\":"<<num(dpi)<<",\"channels\":"<<(int)channelKind
      <<",\"title\":\""<<escJson(title)<<"\","
      <<"\"layers\":[";
    for(size_t i=0;i<roots.size();++i){if(i)os<<",";serializeLayer(roots[i],os);}
    os<<"]}";
    return os.str();
}

struct JsonCursor { const char* cur; bool ok=true; };
static double jnum(JsonCursor& jc){char*e;double v=strtod(jc.cur,&e);if(e==jc.cur)jc.ok=false;jc.cur=e;return v;}
static void jskip(JsonCursor& jc){while(*jc.cur&&(*jc.cur==' '||*jc.cur==','||*jc.cur==':'||*jc.cur=='\n'||*jc.cur=='\t'))++jc.cur;}
static std::string jstr(JsonCursor& jc){jskip(jc);if(*jc.cur!='"'){jc.ok=false;return "";}++jc.cur;std::string s;while(*jc.cur&&*jc.cur!='"'){if(*jc.cur=='\\'){++jc.cur;if(*jc.cur=='n'){s.push_back('\n');++jc.cur;}else{s.push_back(*jc.cur);++jc.cur;}}else{s.push_back(*jc.cur);++jc.cur;}}if(*jc.cur=='"')++jc.cur;return s;}

static std::shared_ptr<LayerNode> parseLayer(JsonCursor& jc) {
    jskip(jc);
    if (*jc.cur!='{'){jc.ok=false;return nullptr;}
    ++jc.cur;
    auto l=std::make_shared<LayerNode>();
    while(*jc.cur&&*jc.cur!='}'&&jc.ok){
        jskip(jc);
        if(*jc.cur==','){++jc.cur;continue;}
        if(*jc.cur=='}')break;
        std::string key=jstr(jc);
        jskip(jc);
        if(*jc.cur!=':'){jc.ok=false;break;}
        ++jc.cur;jskip(jc);
        if(key=="id")l->id=jstr(jc);
        else if(key=="name")l->name=jstr(jc);
        else if(key=="kind")l->kind=(int)jnum(jc);
        else if(key=="visible")l->visible=(*jc.cur=='t')?(jc.cur+=4,true):(jc.cur+=5,false);
        else if(key=="opacity")l->opacity=(float)jnum(jc);
        else if(key=="blend")l->blend=(BlendMode)(int)jnum(jc);
        else if(key=="clipping")l->clipping=(*jc.cur=='t')?(jc.cur+=4,true):(jc.cur+=5,false);
        else if(key=="x")l->x=(int)jnum(jc);
        else if(key=="y")l->y=(int)jnum(jc);
        else if(key=="adjType")l->adjType=(int)jnum(jc);
        else if(key=="text"){
            ++jc.cur;
            l->textSize=(float)jnum(jc);jnum(jc);jnum(jc);jnum(jc);jnum(jc);
            jskip(jc);l->textContent=jstr(jc);jskip(jc);l->sourcePath=jstr(jc);jskip(jc);l->vectorPathsJson=jstr(jc);
            if(*jc.cur==']')++jc.cur;
        }
        else if(key=="curves"){
            ++jc.cur;l->curveLutR.clear();
            while(*jc.cur&&*jc.cur!=']'&&jc.ok)l->curveLutR.push_back((float)jnum(jc));
            ++jc.cur;jskip(jc);++jc.cur;
            l->curveLutG.clear();
            while(*jc.cur&&*jc.cur!=']'&&jc.ok)l->curveLutG.push_back((float)jnum(jc));
            ++jc.cur;jskip(jc);++jc.cur;
            l->curveLutB.clear();
            while(*jc.cur&&*jc.cur!=']'&&jc.ok)l->curveLutB.push_back((float)jnum(jc));
            if(*jc.cur==']')++jc.cur;
            if(*jc.cur==']')++jc.cur;
        }
        else if(key=="adj"){
            ++jc.cur;l->adjParams.clear();
            while(*jc.cur&&*jc.cur!=']'&&jc.ok)l->adjParams.push_back((float)jnum(jc));
            if(*jc.cur==']')++jc.cur;
        }
        else if(key=="maskW"){l->mask.data.width=(uint32_t)jnum(jc);}
        else if(key=="maskH"){l->mask.data.height=(uint32_t)jnum(jc);}
        else if(key=="mask"){
            ++jc.cur;
            l->mask.data.channels=ChannelKind::RGB;
            l->mask.data.pixels.clear();
            while(*jc.cur&&*jc.cur!=']'&&jc.ok)l->mask.data.pixels.push_back((float)jnum(jc));
            if(*jc.cur==']')++jc.cur;
        }
        else if(key=="pxW"){l->pixels.width=(uint32_t)jnum(jc);}
        else if(key=="pxH"){l->pixels.height=(uint32_t)jnum(jc);}
        else if(key=="px"){
            ++jc.cur;
            l->pixels.channels=ChannelKind::RGB;
            l->pixels.pixels.clear();
            while(*jc.cur&&*jc.cur!=']'&&jc.ok)l->pixels.pixels.push_back((float)jnum(jc));
            if(*jc.cur==']')++jc.cur;
        }
        else if(key=="children"){
            ++jc.cur;
            while(*jc.cur&&*jc.cur!=']'&&jc.ok){
                auto child=parseLayer(jc);
                if(child){child->parent=l;l->children.push_back(child);}
                jskip(jc);
                if(*jc.cur==',')++jc.cur;
            }
            if(*jc.cur==']')++jc.cur;
        }
        else {
            if(*jc.cur=='"')jstr(jc);
            else if(*jc.cur=='['){int depth=0;do{if(*jc.cur=='[')depth++;else if(*jc.cur==']')depth--;++jc.cur;}while(depth>0&&*jc.cur);}
            else{jnum(jc);}
        }
        jskip(jc);
    }
    if(*jc.cur=='}')++jc.cur;
    return l;
}

std::shared_ptr<Document> Document::deserialize(const std::string& json) {
    auto doc=std::make_shared<Document>();
    JsonCursor jc{json.c_str(),true};
    jskip(jc);
    if(*jc.cur!='{')return doc;
    ++jc.cur;
    while(*jc.cur&&*jc.cur!='}'&&jc.ok){
        jskip(jc);
        if(*jc.cur==','){++jc.cur;continue;}
        std::string key=jstr(jc);
        jskip(jc);
        if(*jc.cur!=':')break;
        ++jc.cur;jskip(jc);
        if(key=="width")doc->width=(uint32_t)jnum(jc);
        else if(key=="height")doc->height=(uint32_t)jnum(jc);
        else if(key=="dpi")doc->dpi=(float)jnum(jc);
        else if(key=="channels")doc->channelKind=(ChannelKind)(int)jnum(jc);
        else if(key=="title")doc->title=jstr(jc);
        else if(key=="layers"){
            ++jc.cur;
            while(*jc.cur&&*jc.cur!=']'&&jc.ok){
                auto l=parseLayer(jc);
                if(l)doc->roots.push_back(l);
                jskip(jc);
                if(*jc.cur==',')++jc.cur;
            }
            if(*jc.cur==']')++jc.cur;
        }
        else {
            if(*jc.cur=='"')jstr(jc);
            else if(*jc.cur=='['){int depth=0;do{if(*jc.cur=='[')depth++;else if(*jc.cur==']')depth--;++jc.cur;}while(depth>0&&*jc.cur);}
            else jnum(jc);
        }
        jskip(jc);
    }
    return doc;
}

std::shared_ptr<Document> Document::open(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return nullptr;
    std::stringstream ss;
    ss << f.rdbuf();
    std::string content = ss.str();
    if (!content.empty() && content[0]=='{') {
        return deserialize(content);
    }
    Image img;
    if (!load_image_any(path, img)) return nullptr;
    auto doc = std::make_shared<Document>(img.width, img.height, path, ChannelKind::RGB);
    auto bg = doc->addRasterLayer("Background");
    bg->pixels = clone_image(img);
    doc->path = path;
    return doc;
}

bool Document::save(const std::string& p) {
    if (p.size()>5 && p.substr(p.size()-5)==".png") {
        Image flat;
        flattenTo(flat);
        return save_png(p, flat);
    }
    std::ofstream f(p, std::ios::binary);
    if (!f) return false;
    f << serialize();
    return true;
}

bool Document::saveCache(const std::string& cachePath) {
    std::ofstream f(cachePath, std::ios::binary);
    if (!f) return false;
    f << serialize();
    return true;
}

}
