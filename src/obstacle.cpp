#include <rrt-star/obstacle.h>

#include <cmath>
#include <iostream>

namespace rrt_star {

Circle::Circle(Point p, double radius) : Obstacle(), m_pos(p), m_radius(radius) {}

bool Circle::contains(const Point& point) const {
    return Vector(point.x() - m_pos.x(), point.y() - m_pos.y()).squaredNorm() < m_radius * m_radius;
}

bool Circle::goesThrough(const Line& line) const {
    // after https://mathworld.wolfram.com/Circle-LineIntersection.html

    // move line such that the circle would be at 0/0
    Line movedLine = {line.first - m_pos, line.second - m_pos};

    double dx = movedLine.second.x() - movedLine.first.x();
    double dy = movedLine.second.y() - movedLine.first.y();
    double dr = std::sqrt(dx * dx + dy * dy);
    double D = movedLine.first.x() * movedLine.second.y() - movedLine.second.x() * movedLine.first.y();

    return (m_radius * m_radius * dr * dr - D * D) >= 0;
}

void Obstacle::dump(std::ostream& stream) const {
    stream << "0" << std::endl;
}

void Circle::dump(std::ostream& stream) const {
    stream << m_pos.x() << ", " << m_pos.y() << ", " << m_radius;
}

}  // namespace rrt_star