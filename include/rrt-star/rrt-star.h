#pragma once

#include <rrt-star/graph.h>

#include <random>

namespace rrt_star {

class RRTStar {
public:
    RRTStar(const Point& startPos, const Point& endPos, const std::vector<ObstacleSPtr>& obstacles);

    // true if successful
    bool compute(const size_t& iterations, const double& maxStepSize);

    // dump the final data to a stream, JSON-formatted
    void dump(std::ostream& stream) const;

private:
    Point createRandomVertex() const;

    bool goesThroughAnyObstacle(const Line& line) const;
    bool isInAnyObstacle(const Point& point) const;

    Point createVertexBasedOnRandomAndClosest(const Point& randomVertex, size_t closestVertexIdx, const double& maxStepSize, double& distance) const;

private:
    Point m_startPos = Point();
    Point m_endPos = Point();
    const std::vector<ObstacleSPtr>& m_obstacles;

    bool m_success = false;
    Path m_path;  // final path if success

    Graph m_graph;

    // random number generation
    mutable std::mt19937 m_gen;
    mutable std::uniform_real_distribution<double> m_unif = std::uniform_real_distribution<double>(0, 1);
};

}  // namespace rrt_star
