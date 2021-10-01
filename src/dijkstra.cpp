#include <rrt-star/dijkstra.h>

#include <algorithm>
#include <iostream>
#include <unordered_map>

namespace rrt_star {

using MyPairType = std::pair<size_t, double>;
struct CompareSecond {
    bool operator()(const MyPairType& left, const MyPairType& right) const {
        return left.second < right.second;
    }
};

// TODO: replace dist by a heap?
Path Dijkstra::compute(const Graph& graph, const Point& startPos, const Point& endPos) {
    size_t src_idx = graph.getIndex(startPos);
    size_t end_idx = graph.getIndex(endPos);

    std::vector<size_t> nodes = graph.getAllNeighborKeys();
    std::unordered_map<size_t, double> dist;
    std::unordered_map<size_t, size_t> prev;

    size_t NONE = -1;  // big number actually

    for (const auto& i : nodes) {
        dist[i] = std::numeric_limits<double>::infinity();
        prev[i] = NONE;
    }

    dist[src_idx] = 0.;

    while (nodes.size() > 0) {
        auto min_node_it = std::min_element(nodes.begin(), nodes.end(), [&dist](const size_t& left, const size_t& right) { return dist[left] < dist[right]; });
        size_t currNode = *min_node_it;
        nodes.erase(min_node_it);

        if (dist[currNode] == std::numeric_limits<double>::infinity())
            break;

        auto neighbors = graph.getNeighbors(currNode);
        for (const auto& np : neighbors) {
            double newCost = dist[currNode] + np.second;
            if (newCost < dist[np.first]) {
                dist[np.first] = newCost;
                prev[np.first] = currNode;
            }
        }
    }

    // retrieve path
    size_t currNode = end_idx;
    Path path_reversed;
    while (prev[currNode] != NONE) {
        path_reversed.push_back(currNode);
        currNode = prev[currNode];
    }
    path_reversed.push_back(currNode);

    std::reverse(path_reversed.begin(), path_reversed.end());
    return path_reversed;
}

}  // namespace rrt_star