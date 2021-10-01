#pragma once

#include <rrt-star/graph.h>

namespace rrt_star {

using Path = std::vector<size_t>;

class Dijkstra {
public:
    static Path compute(const Graph& graph, const Point& startPos, const Point& endPos);
};

}  // namespace rrt_star