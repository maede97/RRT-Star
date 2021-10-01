#pragma once

#include <rrt-star/math.h>

namespace rrt_star {
class Obstacle {
public:
    Obstacle() {}

    virtual bool contains(const Point& point) const {
        return false;
    };
    virtual bool goesThrough(const Line& line) const {
        return false;
    };

    virtual void dump(std::ostream& stream) const;
};
using ObstacleSPtr = std::shared_ptr<Obstacle>;

class Circle : public Obstacle {
public:
    Circle(Point p, double radius);

    virtual bool contains(const Point& point) const override;
    virtual bool goesThrough(const Line& line) const override;

    virtual void dump(std::ostream& stream) const override;

private:
    Point m_pos = Point();
    double m_radius = 0.;
};

}  // namespace rrt_star