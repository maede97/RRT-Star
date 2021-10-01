#include <rrt-star/graph.h>

#include <algorithm>
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
}

size_t Graph::addVertex(const Point& vertex) {
    // TODO: m_vertices should be a set, such that every vertex is only once added. Otherwise return it's index.
    // TODO: Therefore we need a hash function on the Point.
    m_vertices.push_back({-1, vertex});
    m_distances.push_back(0.);
    return m_vertices.size() - 1;
}

void Graph::addEdge(size_t idx_parent, size_t idx_child) {
    m_edges.push_back({idx_parent, idx_child});
    m_vertices[idx_child].first = idx_parent;
}

bool Graph::getClosestNeighbor(const Point& point, const std::vector<ObstacleSPtr>& obstacles, size_t& vertex_idx) const {
    double min_dist = std::numeric_limits<double>::infinity();

    bool has_at_least_one = false;
    for (size_t i = 0; i < m_vertices.size(); i++) {
        Line line = {point, m_vertices[i].second};
        bool isInAny = false;
        for (const auto& obstacle : obstacles) {
            if (obstacle->goesThrough(line)) {
                isInAny = true;
                break;
            }
        }
        if (isInAny)
            continue;

        double dist = Vector(point, m_vertices[i].second).norm();
        if (dist < min_dist) {
            vertex_idx = i;
            min_dist = dist;
            has_at_least_one = true;
        }
    }

    return has_at_least_one;
}

const size_t Graph::getNumVertices() const {
    return m_vertices.size();
}

const Point Graph::getVertex(size_t index) const {
    return m_vertices.at(index).second;
}

const size_t Graph::getIndex(const Point& p) const {
    for (size_t i = 0; i < m_vertices.size(); i++) {
        if (Vector(m_vertices.at(i).second, p).norm() < std::numeric_limits<double>::epsilon()) {
            return i;
        }
    }
    return m_vertices.size();
}

const size_t Graph::getParent(size_t index) const {
    return m_vertices.at(index).first;
}

const double Graph::distance(size_t index) const {
    return m_distances.at(index);
}

double& Graph::distance(size_t index) {
    return m_distances.at(index);
}

void Graph::propagateCost(size_t parent_idx, const double& cost_change) {
    size_t idx = 0;
    for (const auto& vertex : m_vertices) {
        if (vertex.first == parent_idx) {
            m_distances.at(idx) += cost_change;
            propagateCost(idx, cost_change);
        }

        idx++;
    }
}

Path Graph::backtrack(const Point& start_pos, const Point& end_pos) const {
    size_t start_idx = getIndex(start_pos);
    size_t end_idx = getIndex(end_pos);

    Path ret = {end_idx};

    size_t current = end_idx;
    while (m_vertices.at(current).first != -1) {
        ret.push_back(m_vertices.at(current).first);
        current = m_vertices.at(current).first;
    }

    std::reverse(ret.begin(), ret.end());
    return ret;
}

void Graph::dump(std::ostream& stream) const {
    stream << "\"edges\": [" << std::endl;
    size_t idx = 0;
    for (const auto& e : m_edges) {
        Point p1 = m_vertices.at(e.first).second;
        Point p2 = m_vertices.at(e.second).second;
        stream << "    [" << p1.x() << ", " << p1.y() << ", " << p2.x() << ", " << p2.y() << "]";
        if (idx++ != m_edges.size() - 1)
            stream << ",";

        stream << std::endl;
    }

    stream << "  ]," << std::endl;
    stream << "\"vertices\" : [" << std::endl;
    idx = 0;
    for (const auto& v : m_vertices) {
        stream << "    [" << v.second.x() << ", " << v.second.y() << "]";
        if (idx++ != m_vertices.size() - 1)
            stream << ",";
        stream << std::endl;
    }
    stream << "  ]";
}

}  // namespace rrt_star