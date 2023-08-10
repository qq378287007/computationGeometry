
#pragma once
#include <cmath>

class Vector2
{
public:
    explicit Vector2(double dx = 0.0, double dy = 0.0) : mDx(dx), mDy(dy) {}
    double dx() const { return mDx; }
    double dy() const { return mDy; }

    double &dx() { return mDx; }
    double &dy() { return mDy; }

    Vector2 operator+(const Vector2 &v) const
    {
        return Vector2(mDx + v.dx(), mDy + v.dy());
    }
    Vector2 operator-(const Vector2 &v) const
    {
        return Vector2(mDx - v.dx(), mDy - v.dy());
    }
    Vector2 operator*(double k) const
    {
        return Vector2(mDx * k, mDy * k);
    }
    Vector2 operator/(double k) const
    {
        return Vector2(mDx / k, mDy / k);
    }
    double norm() const
    {
        return mDx * mDx + mDy * mDy;
    }
    double length() const
    {
        return std::sqrt(norm());
    }

    Vector2 normalized() const
    {
        return *this / length();
    }

    double dot(const Vector2 &v) const
    {
        return mDx * v.mDx + mDy * v.mDy;
    }
    double cross(const Vector2 &v) const
    {
        return mDx * v.mDy - mDy * v.mDx;
    }

private:
    double mDx;
    double mDy;
};