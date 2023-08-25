#pragma once

#include <vector>
using namespace std;

#include "Point2.hpp"

class Polygon2
{
public:
    explicit Polygon2(const vector<Point2>& pts = vector<Point2>()) : mPts(pts) {}
    vector<Point2> pts() const { return mPts; }

private:
    vector<Point2> mPts;
};


