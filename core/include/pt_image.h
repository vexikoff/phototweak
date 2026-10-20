#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>

namespace pt {

enum class BlendMode : int {
    Normal = 0, PassThrough, Dissolve, Darken, Multiply, ColorBurn, LinearBurn,
    Lighter, Screen, ColorDodge, LinearDodge, HardLight, SoftLight, VividLight,
    LinearLight, PinLight, Difference, Exclusion, Hue, Saturation, Color, Luminosity
};

enum class ChannelKind : int { RGB = 0, CMYK, Lab, Gray };

struct Image {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<float> pixels;
    ChannelKind channels = ChannelKind::RGB;

    size_t pixelCount() const { return static_cast<size_t>(width) * height; }
    float* data() { return pixels.data(); }
    const float* data() const { return pixels.data(); }

    Image() = default;
    Image(uint32_t w, uint32_t h, ChannelKind ch, float fill = 0.0f);
};

Image create_image(uint32_t w, uint32_t h, ChannelKind ch, float fill);
Image clone_image(const Image& src);
bool load_png(const std::string& path, Image& out);
bool save_png(const std::string& path, const Image& img);
bool load_image_any(const std::string& path, Image& out);
bool save_image_any(const std::string& path, const Image& img);

}
