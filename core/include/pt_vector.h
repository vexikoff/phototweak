#pragma once

#include "pt_image.h"
#include <vector>
#include <string>

namespace pt {

struct PathPoint {
    float x = 0, y = 0;
    float inX = 0, inY = 0, outX = 0, outY = 0;
    bool smooth = true;
};

struct SubPath {
    std::vector<PathPoint> points;
    bool closed = false;
};

struct VectorPath {
    std::vector<SubPath> subpaths;
    int fillRule = 0;
};

void path_flatten(const VectorPath& vp, std::vector<std::vector<float>>& polys, float tolerance);
void shape_rect(VectorPath& vp, float x, float y, float w, float h, float radius);
void shape_ellipse(VectorPath& vp, float cx, float cy, float rx, float ry);
void shape_polygon(VectorPath& vp, float cx, float cy, float r, int sides, float rotDeg);
void shape_star(VectorPath& vp, float cx, float cy, float r1, float r2, int points, float rotDeg);
void shape_line(VectorPath& vp, float x0, float y0, float x1, float y1, float width);
void boolean_paths(const VectorPath& a, const VectorPath& b, VectorPath& out, int op);
void rasterize_path(const VectorPath& vp, Image& layer, const std::vector<float>* mask, const Color& fill, bool antiAlias, float feather);
void stroke_path(const VectorPath& vp, Image& layer, const std::vector<float>* mask, const Color& c, float width, bool antiAlias);
std::string paths_to_json(const std::vector<VectorPath>& paths);
bool paths_from_json(const std::string& json, std::vector<VectorPath>& paths);
void text_to_layer(Image& layer, const std::string& utf8, float pxSize, const Color& color, float x, float y, bool vertical);
float measure_text(const std::string& utf8, float pxSize, float& outWidth, float& outHeight);

}
