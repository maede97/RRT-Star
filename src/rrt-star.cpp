#include <rrt-star/rrt-star.h>

#include <iostream>

namespace rrt_star {
RRTStar::RRTStar(const Point& startPos, const Point& endPos, const std::vector<ObstacleSPtr>& obstacles)
    : m_startPos(startPos), m_endPos(endPos), m_obstacles(obstacles) {
    // Let the domain be spanned by both start and end point.
    m_domain.first = Point(std::min(startPos.x(), endPos.x()), std::min(startPos.y(), endPos.y()));
    m_domain.second = Point(std::max(startPos.x(), endPos.x()), std::max(startPos.y(), endPos.y()));

    m_gen.seed((size_t)time(0));
}

RRTStar::RRTStar(const Domain& domain, const Point& startPos, const Point& endPos, const std::vector<ObstacleSPtr>& obstacles)
    : m_domain(domain), m_startPos(startPos), m_endPos(endPos), m_obstacles(obstacles) {
    m_gen.seed((size_t)time(0));
}

bool RRTStar::compute(const size_t& iterations, const double& maxStepSize) {
    m_graph.clear();

    // We have at most n new vertices and around 2*n edges
    m_graph.reserve(iterations, iterations * 2);

    m_success = false;

    // Initially: push back the start position
    m_graph.addVertex(m_startPos);

    for (size_t i = 0; i < iterations; i++) {
        // 1. create a random position
        Point randomVertex = createRandomVertex();

        // 2. check if in obstacle
        if (isInAnyObstacle(randomVertex))
            continue;

        // 3. check for neighbor
        size_t closest_vertex_idx = 0;
        if (!m_graph.getClosestNeighbor(randomVertex, m_obstacles, closest_vertex_idx)) {
            // error, no neighbor found
            continue;
        }

        // 4. Create the vertex based on the random vertex and the closest vertex
        // TODO: put this code inside a function with a good name
        Point closestVertex = m_graph.getVertex(closest_vertex_idx);
        Vector dir = Vector(closestVertex, randomVertex);
        double length = dir.norm();
        double fac = std::min(length, maxStepSize) / length;
        Vector dir_n = Vector(dir.x() * fac, dir.y() * fac);

        Point newVertex = Point(closestVertex.x() + dir_n.x(), closestVertex.y() + dir_n.y());

        // 5. Add edge to graph to all neighbors
        size_t new_idx = m_graph.addVertex(newVertex);
        double dist = dir_n.norm();
        m_graph.addEdge(closest_vertex_idx, new_idx, dist);
        m_graph.distance(new_idx) = m_graph.distance(closest_vertex_idx) + dist;

        // 6. Update nearby vertices
        size_t num_vertices = m_graph.getNumVertices();
        for (size_t i = 0; i < num_vertices; i++) {
            if (i == new_idx)
                continue;

            double dist = Vector(m_graph.getVertex(i), newVertex).norm();
            if (dist > maxStepSize)
                continue;

            Line line = {m_graph.getVertex(i), newVertex};
            bool isInAny = false;
            for (const ObstacleSPtr obstacle : m_obstacles) {
                if (obstacle->goesThrough(line)) {
                    isInAny = true;
                    break;
                }
            }
            if (isInAny) {
                continue;
            }

            if (m_graph.distance(new_idx) + dist < m_graph.distance(i)) {
                m_graph.addEdge(i, new_idx, dist);
                m_graph.distance(i) = m_graph.distance(new_idx) + dist;
            }
        }

        // 6. check if goal position reached
        double distance_to_end = Vector(newVertex, m_endPos).norm();
        if (distance_to_end < 2 * maxStepSize) {
            size_t end_idx = m_graph.addVertex(m_endPos);
            m_graph.addEdge(new_idx, end_idx, distance_to_end);

            // try to update the distance matrix
            try {
                m_graph.distance(end_idx) = std::min(m_graph.distance(end_idx), m_graph.distance(new_idx) + distance_to_end);
            } catch (const std::exception&) {
                m_graph.distance(end_idx) = m_graph.distance(new_idx) + distance_to_end;
            }

            m_success = true;
        }
    }

    if (m_success) {
        m_path = Dijkstra::compute(m_graph, m_startPos, m_endPos);
    }

    return m_success;
}

const Graph& RRTStar::getGraph() const {
    return m_graph;
}

void RRTStar::dump(std::ostream& stream) const {
    stream << "{" << std::endl;

    if (m_success) {
        m_graph.dump(stream);
        stream << "," << std::endl;
    }
    stream << "\"obstacles\" : [" << std::endl;
    size_t idx = 0;
    for (const auto& o : m_obstacles) {
        stream << "    [";
        o->dump(stream);
        stream << "]";
        if (idx++ != m_obstacles.size() - 1)
            stream << ",";
        stream << std::endl;
    }
    stream << "  ]," << std::endl;

    if (m_success) {
        stream << "\"path\" : [" << std::endl;
        idx = 0;
        size_t prev_idx = -1;
        for (const auto& curr_idx : m_path) {
            if (prev_idx == -1) {
                // first iteration, just store it.
                prev_idx = curr_idx;
                idx++;
                continue;
            }
            Point p1 = m_graph.getVertex(prev_idx);
            Point p2 = m_graph.getVertex(curr_idx);
            stream << "    [" << p1.x() << ", " << p1.y() << ", " << p2.x() << ", " << p2.y() << "]";
            if (idx++ != m_path.size() - 1)
                stream << ",";

            stream << std::endl;

            prev_idx = curr_idx;
        }
        stream << "  ]";
    }
    stream << std::endl << "}" << std::endl;
}

Point RRTStar::createRandomVertex() const {
    // Wheight the random point to be close to the end
    Vector s = Vector(m_startPos, m_endPos);

    double rx = m_unif(m_gen);
    double ry = m_unif(m_gen);
    double x = m_startPos.x() - (s.x() / 2.) + rx * s.x() * 2.;
    double y = m_startPos.y() - (s.y() / 2.) + ry * s.y() * 2.;
    return Point(x, y);
}

bool RRTStar::isInAnyObstacle(const Point& point) const {
    bool isInAny = false;
    for (const auto& obstacle : m_obstacles) {
        if (obstacle->contains(point)) {
            return true;
        }
    }
    return false;
}

}  // namespace rrt_star