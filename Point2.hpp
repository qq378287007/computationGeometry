#pragma once

class Point2
{
public:
    explicit Point2(double x = 0.0, double y = 0.0) : mX(x), mY(y) {}
    double x() const { return mX; }
    double y() const { return mY; }

private:
    double mX;
    double mY;
};


