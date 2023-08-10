#pragma once

#include "Point2.hpp"

class Circle
{
public:
    explicit Circle(Point2 c = Point2(), double r = 0.0) : mC(c), mR(r) {}
    Point2 c() const { return mC; }
    double r() const { return mR; }

private:
    Point2 mC;
    double mR;
};
