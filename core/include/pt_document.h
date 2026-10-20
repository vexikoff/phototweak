#pragma once

#include "pt_image.h"
#include <string>
#include <vector>
#include <memory>

namespace pt {

struct LayerMaskData {
    bool enabled = true;
    Image data;
};

struct LayerNode {
    std::string id;
    std::string name;
    int kind = 0;
    bool visible = true;
    float opacity = 1.0f;
    BlendMode blend = BlendMode::Normal;
    bool clipping = false;
    int x = 0;
    int y = 0;
    Image pixels;
    LayerMaskData mask;
    std::string sourcePath;
    std::vector<std::shared_ptr<LayerNode>> children;
    std::weak_ptr<LayerNode> parent;
    std::vector<float> curveLutR;
    std::vector<float> curveLutG;
    std::vector<float> curveLutB;
    std::vector<float> adjParams;
    int adjType = -1;
    std::string textContent;
    float textSize = 24.0f;
    Color textColor{0,0,0,1};
    std::string vectorPathsJson;

    bool isGroup() const { return kind == 1; }
    bool isEmptyRaster() const { return kind == 0 && pixels.width == 0; }
};

class Document {
public:
    uint32_t width = 0;
    uint32_t height = 0;
    float dpi = 72.0f;
    ChannelKind channelKind = ChannelKind::RGB;
    std::string title;
    std::string path;
    std::vector<std::shared_ptr<LayerNode>> roots;
    std::vector<std::string> savedSelections;

    Document() = default;
    Document(uint32_t w, uint32_t h, const std::string& name, ChannelKind ch);

    std::shared_ptr<LayerNode> addRasterLayer(const std::string& name);
    std::shared_ptr<LayerNode> addGroup(const std::string& name);
    std::shared_ptr<LayerNode> addAdjustmentLayer(const std::string& name, int adjType);
    std::shared_ptr<LayerNode> addTextLayer(const std::string& name);
    std::shared_ptr<LayerNode> addSmartObjectLayer(const std::string& name, const std::string& sourcePath);
    std::shared_ptr<LayerNode> findLayer(const std::string& id);
    bool removeLayer(const std::string& id);
    bool moveLayer(const std::string& id, int newIndex);
    void flattenTo(Image& out) const;
    void compositeRegion(Image& out, int x0, int y0, int w, int h) const;
    std::string serialize() const;
    static std::shared_ptr<Document> deserialize(const std::string& json);
    static std::shared_ptr<Document> open(const std::string& path);
    bool save(const std::string& path);
    bool saveCache(const std::string& cachePath);
};

std::string new_layer_id();

}
