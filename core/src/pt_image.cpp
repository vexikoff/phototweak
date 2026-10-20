#include "pt_image.h"
#include <algorithm>
#include <cmath>

namespace pt {

static int channelsOf(ChannelKind ch) {
    switch (ch) {
        case ChannelKind::RGB: return 4;
        case ChannelKind::CMYK: return 5;
        case ChannelKind::Lab: return 4;
        case ChannelKind::Gray: return 2;
    }
    return 4;
}

Image::Image(uint32_t w, uint32_t h, ChannelKind ch, float fill) : width(w), height(h), channels(ch) {
    int c = channelsOf(ch);
    pixels.assign(static_cast<size_t>(w) * h * c, fill);
    if (c == 4 && ch == ChannelKind::RGB) {
        for (size_t i = 3; i < pixels.size(); i += 4) pixels[i] = 1.0f;
        if (fill != 0.0f) for (size_t i = 3; i < pixels.size(); i += 4) pixels[i] = fill;
    }
}

Image create_image(uint32_t w, uint32_t h, ChannelKind ch, float fill) {
    return Image(w, h, ch, fill);
}

Image clone_image(const Image& src) {
    Image out;
    out.width = src.width;
    out.height = src.height;
    out.channels = src.channels;
    out.pixels = src.pixels;
    return out;
}

}
