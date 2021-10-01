#pragma once

#include <rrt-star/obstacle.h>

#include <vector>

namespace rrt_star {

class Graph {
    // TODO: change data structure of both vertices and edges to allow faster access!
public:
    Graph();

    void clear();
    void reserve(size_t num_vertices, size_t num_edges);

    // TODO: improve this!
    void dump(std::ostream& stream) const;

    size_t addVertex(const Point& vertex);
    void addEdge(size_t idx_1, size_t idx_2, double distance);

    bool getClosestNeighbor(const Point& point, const std::vector<ObstacleSPtr>& obstacles, size_t& vertex_idx) const;

    // Vertices
    const size_t getNumVertices() const;
    const Point getVertex(size_t index) const;
    const size_t getIndex(const Point& p) const;

    // Distance
    const double distance(size_t index) const;
    double& distance(size_t index);

    // Neighbors
    const std::vector<size_t> getAllNeighborKeys() const;
    std::vector<std::pair<size_t, double>> getNeighbors(size_t index) const;

private:
    using Edge = std::pair<size_t, size_t>;
    std::vector<Point> m_vertices;
    std::vector<Edge> m_edges;
    std::vector<double> m_distances;
    // m_neighbors[idx][...]{neighbor_idx, cost}
    std::vector<std::vector<std::pair<size_t, double>>> m_neighbors;
};

}  // namespace rrt_star