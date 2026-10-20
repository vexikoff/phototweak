#include "pt_image.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>

#ifdef PT_HAVE_STB_IMAGE
#include "stb_image.h"
#include "stb_image_write.h"
#endif

namespace pt {

static bool writeChunk(FILE* f, const char* type, const uint8_t* data, uint32_t len) {
    uint32_t beLen = __builtin_bswap32(len);
    if (fwrite(&beLen, 4, 1, f) != 1) return false;
    if (fwrite(type, 4, 1, f) != 1) return false;
    if (len && fwrite(data, 1, len, f) != len) return false;
    uint32_t crc = 0xFFFFFFFFu;
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        init = true;
    }
    std::vector<uint8_t> buf(4 + len);
    memcpy(buf.data(), type, 4);
    if (len) memcpy(buf.data() + 4, data, len);
    for (uint8_t b : buf) crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8);
    crc ^= 0xFFFFFFFFu;
    uint32_t beCrc = __builtin_bswap32(crc);
    if (fwrite(&beCrc, 4, 1, f) != 1) return false;
    return true;
}

static uint32_t adler32(const uint8_t* data, size_t len) {
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < len; ++i) { a = (a + data[i]) % 65521; b = (b + a) % 65521; }
    return (b << 16) | a;
}

static bool deflateStore(const uint8_t* in, size_t inLen, std::vector<uint8_t>& out) {
    out.push_back(0x78); out.push_back(0x01);
    size_t pos = 0;
    do {
        size_t block = std::min<size_t>(inLen - pos, 65535);
        bool last = (pos + block >= inLen);
        out.push_back(last ? 1 : 0);
        uint16_t len = static_cast<uint16_t>(block);
        uint16_t nlen = ~len;
        out.push_back(len & 0xFF); out.push_back(len >> 8);
        out.push_back(nlen & 0xFF); out.push_back(nlen >> 8);
        out.insert(out.end(), in + pos, in + pos + block);
        pos += block;
    } while (pos < inLen);
    uint32_t ad = adler32(in, inLen);
    out.push_back((ad >> 24) & 0xFF); out.push_back((ad >> 16) & 0xFF);
    out.push_back((ad >> 8) & 0xFF); out.push_back(ad & 0xFF);
    return true;
}

bool save_png(const std::string& path, const Image& img) {
    if (img.width == 0 || img.height == 0) return false;
    std::vector<uint8_t> raw(static_cast<size_t>(img.width) * img.height * 4);
    for (size_t i = 0; i < img.pixelCount(); ++i) {
        float r = img.pixels[i * 4 + 0], g = img.pixels[i * 4 + 1], b = img.pixels[i * 4 + 2], a = img.pixels[i * 4 + 3];
        raw[i * 4 + 0] = static_cast<uint8_t>(std::lround(std::max(0.0f, std::min(1.0f, r)) * 255.0f));
        raw[i * 4 + 1] = static_cast<uint8_t>(std::lround(std::max(0.0f, std::min(1.0f, g)) * 255.0f));
        raw[i * 4 + 2] = static_cast<uint8_t>(std::lround(std::max(0.0f, std::min(1.0f, b)) * 255.0f));
        raw[i * 4 + 3] = static_cast<uint8_t>(std::lround(std::max(0.0f, std::min(1.0f, a)) * 255.0f));
    }
    std::vector<uint8_t> filtered(raw.size() + img.height);
    size_t srcPos = 0;
    for (uint32_t y = 0; y < img.height; ++y) {
        filtered[y * (1 + img.width * 4)] = 0;
        memcpy(&filtered[y * (1 + img.width * 4) + 1], &raw[srcPos], img.width * 4);
        srcPos += img.width * 4;
    }
    std::vector<uint8_t> z;
    if (!deflateStore(filtered.data(), filtered.size(), z)) return false;
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    fwrite("\x89PNG\r\n\x1a\n", 1, 8, f);
    uint8_t ihdr[13];
    uint32_t wBe = __builtin_bswap32(img.width), hBe = __builtin_bswap32(img.height);
    memcpy(ihdr, &wBe, 4); memcpy(ihdr + 4, &hBe, 4);
    ihdr[8] = 8; ihdr[9] = 6; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
    bool ok = writeChunk(f, "IHDR", ihdr, 13) && writeChunk(f, "IDAT", z.data(), static_cast<uint32_t>(z.size())) && writeChunk(f, "IEND", nullptr, 0);
    fclose(f);
    return ok;
}

static int readU32BE(const uint8_t* p) { return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }

static bool inflateRaw(const std::vector<uint8_t>& in, std::vector<uint8_t>& out) {
    size_t pos = 2;
    while (pos < in.size()) {
        uint8_t hdr = in[pos++];
        bool final = hdr & 1;
        int type = (hdr >> 1) & 3;
        if (type == 0) {
            pos += 4;
            if (pos + 0 >= in.size()) break;
            size_t start = pos;
            uint16_t len = in[start] | (in[start + 1] << 8);
            out.insert(out.end(), in.begin() + pos + 2, in.begin() + pos + 2 + len);
            pos += 2 + len;
        } else {
            return false;
        }
        if (final) break;
    }
    return !out.empty();
}

bool load_png(const std::string& path, Image& out) {
#ifdef PT_HAVE_STB_IMAGE
    int w, h, ch;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &ch, 4);
    if (!data) return false;
    out = create_image(w, h, ChannelKind::RGB, 0.0f);
    for (int i = 0; i < w * h; ++i) {
        out.pixels[i * 4 + 0] = data[i * 4 + 0] / 255.0f;
        out.pixels[i * 4 + 1] = data[i * 4 + 1] / 255.0f;
        out.pixels[i * 4 + 2] = data[i * 4 + 2] / 255.0f;
        out.pixels[i * 4 + 3] = data[i * 4 + 3] / 255.0f;
    }
    stbi_image_free(data);
    return true;
#else
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(sz);
    fread(buf.data(), 1, sz, f); fclose(f);
    if (sz < 8 || memcmp(buf.data(), "\x89PNG\r\n\x1a\n", 8) != 0) return false;
    size_t pos = 8;
    uint32_t w = 0, h = 0;
    std::vector<uint8_t> idat;
    while (pos + 8 <= sz) {
        uint32_t len = readU32BE(&buf[pos]);
        const uint8_t* type = &buf[pos + 4];
        const uint8_t* data = &buf[pos + 8];
        if (memcmp(type, "IHDR", 4) == 0) { w = readU32BE(data); h = readU32BE(data + 4); if (data[8] != 8 || data[9] != 6) { fclose(f); return false; } }
        else if (memcmp(type, "IDAT", 4) == 0) idat.insert(idat.end(), data, data + len);
        else if (memcmp(type, "IEND", 4) == 0) break;
        pos += 12 + len;
    }
    std::vector<uint8_t> dec;
    if (!inflateRaw(idat, dec)) return false;
    out = create_image(w, h, ChannelKind::RGB, 0.0f);
    size_t stride = 1 + w * 4;
    if (dec.size() < stride * h) return false;
    for (uint32_t y = 0; y < h; ++y) {
        const uint8_t* row = &dec[y * stride + 1];
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx = (static_cast<size_t>(y) * w + x) * 4;
            out.pixels[idx + 0] = row[x * 4 + 0] / 255.0f;
            out.pixels[idx + 1] = row[x * 4 + 1] / 255.0f;
            out.pixels[idx + 2] = row[x * 4 + 2] / 255.0f;
            out.pixels[idx + 3] = row[x * 4 + 3] / 255.0f;
        }
    }
    return true;
#endif
}

bool load_image_any(const std::string& path, Image& out) { return load_png(path, out); }
bool save_image_any(const std::string& path, const Image& img) { return save_png(path, img); }

}
