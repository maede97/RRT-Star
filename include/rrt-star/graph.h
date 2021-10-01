#pragma once

#include <rrt-star/obstacle.h>

#include <vector>

namespace rrt_star {

using Path = std::vector<size_t>;

class Graph {
    // TODO: change data structure of both vertices and edges to allow faster access!
public:
    Graph();

    void clear();
    void reserve(size_t num_vertices, size_t num_edges);

    void dump(std::ostream& stream) const;

    size_t addVertex(const Point& vertex);
    // add an edge, but does NOT update the distances.
    void addEdge(size_t idx_parent, size_t idx_child);

    bool getClosestNeighbor(const Point& point, const std::vector<ObstacleSPtr>& obstacles, size_t& vertex_idx) const;

    // Vertices
    const size_t getNumVertices() const;
    const Point getVertex(size_t index) const;
    const size_t getIndex(const Point& p) const;
    const size_t getParent(size_t index) const;

    // Distance
    const double distance(size_t index) const;
    double& distance(size_t index);
    void propagateCost(size_t parent_idx, const double& cost_change);

    // Backtrack
    Path backtrack(const Point& start_pos, const Point& end_pos) const;

private:
    using Edge = std::pair<size_t, size_t>;
    using Node = std::pair<size_t, Point>;  // parent, Vertex

    std::vector<Node> m_vertices;
    std::vector<Edge> m_edges;
    std::vector<double> m_distances;
};

}  // namespace rrt_star