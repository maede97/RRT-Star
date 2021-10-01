#include <rrt-star/obstacle.h>

namespace rrt_star {

Circle::Circle(Point p, double radius) : Obstacle(), m_pos(p), m_radius(radius) {}

bool Circle::contains(const Point& point) const {
    return Vector(m_pos, point).squaredNorm() < m_radius * m_radius;
}

bool Circle::goesThrough(const Line& line) const {
    // after https://mathworld.wolfram.com/Circle-LineIntersection.html

    // move line such that the circle would be at 0/0
    Line movedLine = {line.first - m_pos, line.second - m_pos};

    Vector d = Vector(movedLine.first, movedLine.second);
    double dr2 = d.squaredNorm();
    double D = movedLine.first.x() * movedLine.second.y() - movedLine.second.x() * movedLine.first.y();

    return (m_radius * m_radius * dr2 - D * D) >= 0;
}

void Obstacle::dump(std::ostream&) const {}

void Circle::dump(std::ostream& stream) const {
    stream << m_pos.x() << ", " << m_pos.y() << ", " << m_radius;
}

}  // namespace rrt_star