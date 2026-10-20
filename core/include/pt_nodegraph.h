#pragma once

#include "pt_image.h"
#include <vector>
#include <string>
#include <cstdint>

namespace pt {

struct NodeGraph {
    struct Node {
        int id = 0;
        std::string type;
        std::vector<float> params;
        std::string strParam;
    };
    struct Edge {
        int fromNode = 0;
        int fromSocket = 0;
        int toNode = 0;
        int toSocket = 0;
    };
    std::vector<Node> nodes;
    std::vector<Edge> edges;
};

bool node_graph_run(const NodeGraph& g, const Image& input, Image& output);
std::string node_graph_to_json(const NodeGraph& g);
bool node_graph_from_json(const std::string& json, NodeGraph& g);

struct ActionStep {
    std::string command;
    std::string argsJson;
};

struct RecordedAction {
    std::string name;
    std::vector<ActionStep> steps;
};

class HistoryStack {
public:
    struct Snapshot {
        std::string label;
        std::vector<uint8_t> compressedDoc;
    };
    void push(const std::string& label, const std::string& docJson);
    bool canUndo() const { return index_ > 0; }
    bool canRedo() const { return index_ < static_cast<int>(entries_.size()) - 1; }
    std::string undo();
    std::string redo();
    std::string current() const;
    size_t size() const { return entries_.size(); }
    void clear();
private:
    std::vector<Snapshot> entries_;
    int index_ = -1;
};

uint32_t crc32_compute(const uint8_t* data, size_t len);
bool zlib_compress(const std::vector<uint8_t>& in, std::vector<uint8_t>& out);
bool zlib_decompress(const std::vector<uint8_t>& in, std::vector<uint8_t>& out);

}
