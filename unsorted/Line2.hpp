#pragma once

#include "Point2.hpp"

class Line2
{
public:
    explicit Line2(Point2 a = Point2(), Point2 b = Point2()) : mA(a), mB(b) {}
    Point2 a() const { return mA; }
    Point2 b() const { return mB; }

private:
    Point2 mA;
    Point2 mB;
};
