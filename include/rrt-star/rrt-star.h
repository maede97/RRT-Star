#pragma once

#include <rrt-star/dijkstra.h>
#include <rrt-star/graph.h>
#include <rrt-star/obstacle.h>

#include <random>
#include <vector>

namespace rrt_star {

using Domain = std::pair<Point, Point>;

// TODO: currently implemented is only RRT, not RRT-Star.
// https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80
class RRTStar {
public:
    RRTStar(const Point& startPos, const Point& endPos, const std::vector<ObstacleSPtr>& obstacles);
    RRTStar(const Domain& domain, const Point& startPos, const Point& endPos, const std::vector<ObstacleSPtr>& obstacles);

    // true if successful
    // TODO: keep going as flag after goal found to improve it
    bool compute(const size_t& iterations, const double& maxStepSize);

    // get data, a tree for example, to perform dijkstra on it
    const Graph& getGraph() const;

    // dump the data to a stream
    void dump(std::ostream& stream) const;

private:
    Point createRandomVertex() const;
    bool isInAnyObstacle(const Point& point) const;

private:
    Point m_startPos = Point();
    Point m_endPos = Point();
    const std::vector<ObstacleSPtr>& m_obstacles;

    bool m_success = false;
    Path m_path;  // final path if success

    Domain m_domain = {};

    Graph m_graph;

    // random number generation
    mutable std::mt19937 m_gen;
    mutable std::uniform_real_distribution<double> m_unif = std::uniform_real_distribution<double>(0, 1);
};

}  // namespace rrt_star
