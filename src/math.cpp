#include <rrt-star/math.h>

#include <cmath>

namespace rrt_star {

Point::Point() {}

Point::Point(double x, double y) : m_x(x), m_y(y) {}

double Point::x() const {
    return m_x;
}

double Point::y() const {
    return m_y;
}

double& Point::x() {
    return m_x;
}

double& Point::y() {
    return m_y;
}

Point Point::operator+(const Point& other) const {
    return Point(m_x + other.m_x, m_y + other.m_y);
}

Point Point::operator-(const Point& other) const {
    return Point(m_x - other.m_x, m_y - other.m_y);
}

Point Point::operator+(const Vector& other) const {
    return Point(m_x + other.x(), m_y + other.y());
}

Point Point::operator-(const Vector& other) const {
    return Point(m_x - other.x(), m_y - other.y());
}

Vector::Vector() {}

Vector::Vector(double x, double y) : m_x(x), m_y(y) {}

Vector::Vector(const Point& p1, const Point& p2) {
    m_x = p2.x() - p1.x();
    m_y = p2.y() - p1.y();
}

double Vector::x() const {
    return m_x;
}

double Vector::y() const {
    return m_y;
}

double& Vector::x() {
    return m_x;
}

double& Vector::y() {
    return m_y;
}

Vector Vector::operator+(const Vector& other) const {
    return Vector(m_x + other.m_x, m_y + other.m_y);
}

Vector Vector::operator-(const Vector& other) const {
    return Vector(m_x - other.m_x, m_y - other.m_y);
}

double Vector::norm() const {
    return std::sqrt(squaredNorm());
}

double Vector::squaredNorm() const {
    return m_x * m_x + m_y * m_y;
    ;
}

std::ostream& operator<<(std::ostream& out, const Point& point) {
    out << "P(" << point.x() << ", " << point.y() << ")";
    return out;
}

std::ostream& operator<<(std::ostream& out, const Vector& vector) {
    out << "V(" << vector.x() << ", " << vector.y() << ")";
    return out;
}

}  // namespace rrt_star