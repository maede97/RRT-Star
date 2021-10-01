#pragma once

#include <ostream>

namespace rrt_star {

// Forward declarations
class Point;
class Vector;

// Point 2D class
class Point {
public:
    Point();
    Point(double x, double y);
    ~Point() = default;

    double x() const;
    double y() const;
    double& x();
    double& y();

    Point operator+(const Point& other) const;
    Point operator-(const Point& other) const;

    Point operator+(const Vector& other) const;
    Point operator-(const Vector& other) const;

private:
    double m_x = 0;
    double m_y = 0;
};

std::ostream& operator<<(std::ostream& out, const Point& point);

class Vector {
public:
    Vector();
    Vector(double x, double y);
    Vector(const Point& p1, const Point& p2);
    ~Vector() = default;

    double x() const;
    double y() const;
    double& x();
    double& y();

    Vector operator+(const Vector& other) const;
    Vector operator-(const Vector& other) const;

    double norm() const;
    double squaredNorm() const;

private:
    double m_x = 0;
    double m_y = 0;
};

std::ostream& operator<<(std::ostream& out, const Vector& vector);

using Line = std::pair<Point, Point>;

}  // namespace rrt_star