#pragma once

#include "pt_image.h"

namespace pt {

struct BrushSettings {
    float size = 30.0f;
    float hardness = 0.8f;
    float spacing = 0.1f;
    float opacity = 1.0f;
    float flow = 1.0f;
    float angleDeg = 0.0f;
    float roundness = 1.0f;
    float jitter = 0.0f;
    int shapeType = 0;
    Color color{1, 0, 0, 1};
    bool airbrush = false;
};

void brush_stamp(Image& layer, const BrushSettings& bs, float cx, float cy);
void brush_stroke(Image& layer, const BrushSettings& bs, const std::vector<float>& pointsXY, const std::vector<float>* pressure);
void pencil_stroke(Image& layer, const Color& c, float size, const std::vector<float>& pointsXY, const std::vector<float>* pressure);
void eraser_stroke(Image& layer, const BrushSettings& bs, const std::vector<float>& pointsXY, const std::vector<float>* pressure, bool mode);
void gradient_linear(Image& layer, const std::vector<float>* mask, float x0, float y0, float x1, float y1, const std::vector<float>& stopsRGBA, int stopCount, int interp);
void gradient_radial(Image& layer, const std::vector<float>* mask, float cx, float cy, float r0, float r1, const std::vector<float>& stopsRGBA, int stopCount);
void paint_bucket_fill(Image& layer, const std::vector<float>* mask, int x, int y, const Color& c, float tolerance, bool contiguous);
void clone_stamp(Image& dst, const Image& srcLayer, const std::vector<float>* mask, int dx, int dy, int sx, int sy, int w, int h, float offsetOX, float offsetOY, float opacity);
void pattern_stamp(Image& dst, const Image& pattern, const std::vector<float>* mask, int x0, int y0, int w, int h, int ox, int oy, float opacity);
void healing_brush(Image& layer, const Image& srcLayer, const std::vector<float>* mask, int tx, int ty, int sx, int sy, int radius, float sampleAlpha);
void spot_healing(Image& layer, const std::vector<float>* mask, int cx, int cy, int radius, int type, float tolerance);
void patch_repair(Image& layer, const std::vector<float>* selMask, const std::vector<float>& patchMask, int method);
void content_aware_move(Image& layer, const std::vector<float>* mask, const std::vector<float>& regionMask, int dx, int dy);
void red_eye_removal(Image& layer, const std::vector<float>* mask, int cx, int cy, int radius, float pupilSize, float darkening);
void dodge_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float exposure, float range, const std::vector<float>* pressure);
void burn_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float exposure, float range, const std::vector<float>* pressure);
void sponge_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float flow, int mode, const std::vector<float>* pressure);
void blur_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float strength, const std::vector<float>* pressure);
void sharpen_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float strength, const std::vector<float>* pressure);
void smudge_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float strength, const std::vector<float>* pressure);
void mixer_brush_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, float wet, float mix, float flow, const Color& loadColor, const std::vector<float>* pressure);
void color_replace_stroke(Image& layer, const std::vector<float>* mask, const std::vector<float>& points, int radius, const Color& replacement, float tolerance, int sampleMode, bool locks);
void art_history_stroke(Image& srcHist, Image& dst, const std::vector<float>* mask, const std::vector<float>& points, int radius, float tolerance, int style);
void background_eraser_stroke(Image& layer, const std::vector<float>& points, int brushSize, float tolerance, int limits, bool protectFg);
void magic_eraser_click(Image& layer, int x, int y, float tolerance, bool contiguous, bool antiAlias, float strength);

}
