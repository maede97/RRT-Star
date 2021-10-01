#include <rrt-star/graph.h>

#include <iostream>
#include <limits>

namespace rrt_star {
Graph::Graph() {}
void Graph::clear() {
    m_edges.clear();
    m_vertices.clear();
    m_distances.clear();
}

void Graph::reserve(size_t num_vertices, size_t num_edges) {
    m_vertices.reserve(num_vertices);
    m_edges.reserve(num_edges);
    m_distances.reserve(num_vertices);
    m_neighbors.reserve(num_vertices);
}

size_t Graph::addVertex(const Point& vertex) {
    // TODO: m_vertices should be a set, such that every vertex is only once added. Otherwise return it's index.
    // TODO: Therefore we need a hash function on the Point.
    m_vertices.push_back(vertex);
    m_neighbors.push_back({});
    m_distances.push_back(0.);
    return m_vertices.size() - 1;
}

void Graph::addEdge(size_t idx_1, size_t idx_2, double distance) {
    m_edges.push_back({idx_1, idx_2});
    m_neighbors[idx_1].push_back(std::make_pair(idx_2, distance));
    m_neighbors[idx_2].push_back(std::make_pair(idx_1, distance));
}

bool Graph::getClosestNeighbor(const Point& point, const std::vector<ObstacleSPtr>& obstacles, size_t& vertex_idx) const {
    double min_dist = std::numeric_limits<double>::infinity();

    bool has_at_least_one = false;
    for (size_t i = 0; i < m_vertices.size(); i++) {
        Line line = {point, m_vertices[i]};
        bool isInAny = false;
        for (const auto& obstacle : obstacles) {
            if (obstacle->goesThrough(line)) {
                isInAny = true;
                break;
            }
        }
        if (isInAny)
            continue;

        double dist = Vector(point, m_vertices[i]).norm();
        if (dist < min_dist) {
            vertex_idx = i;
            min_dist = dist;
            has_at_least_one = true;
        }
    }

    return has_at_least_one;
    ;
}

const size_t Graph::getNumVertices() const {
    return m_vertices.size();
}

const Point Graph::getVertex(size_t index) const {
    return m_vertices.at(index);
}

const size_t Graph::getIndex(const Point& p) const {
    for (size_t i = 0; i < m_vertices.size(); i++) {
        if (Vector(m_vertices.at(i), p).norm() < std::numeric_limits<double>::epsilon()) {
            return i;
        }
    }
    return m_vertices.size();
}

const double Graph::distance(size_t index) const {
    return m_distances.at(index);
}

double& Graph::distance(size_t index) {
    return m_distances.at(index);
}

const std::vector<size_t> Graph::getAllNeighborKeys() const {
    std::vector<size_t> keys;
    for (size_t i = 0; i < m_neighbors.size(); i++)
        keys.push_back(i);

    return keys;
}

std::vector<std::pair<size_t, double>> Graph::getNeighbors(size_t index) const {
    return m_neighbors.at(index);
}

void Graph::dump(std::ostream& stream) const {
    stream << "\"edges\": [" << std::endl;
    size_t idx = 0;
    for (const auto& e : m_edges) {
        Point p1 = m_vertices.at(e.first);
        Point p2 = m_vertices.at(e.second);
        stream << "    [" << p1.x() << ", " << p1.y() << ", " << p2.x() << ", " << p2.y() << "]";
        if (idx++ != m_edges.size() - 1)
            stream << ",";

        stream << std::endl;
    }

    stream << "  ]," << std::endl;
    stream << "\"vertices\" : [" << std::endl;
    idx = 0;
    for (const auto& v : m_vertices) {
        stream << "    [" << v.x() << ", " << v.y() << "]";
        if (idx++ != m_vertices.size() - 1)
            stream << ",";
        stream << std::endl;
    }
    stream << "  ]";
}

}  // namespace rrt_star