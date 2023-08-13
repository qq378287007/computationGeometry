#include <iostream>
#include <cmath>
#include <tuple>
#include <utility>
#include <vector>
using namespace std;

inline bool between(double x, double x1, double x2)
{
    return (x <= x1 && x >= x2) || (x <= x2 && x >= x1);
}

inline bool between(double x1, double x2, double x3, double x4)
{
    return between(x1, x3, x4) || between(x2, x3, x4);
}

inline bool pointInRect(double x, double y, double x1, double y1, double x2, double y2)
{
    return between(x, x1, x2) && between(y, y1, y2);
}

inline bool pointInRect2(double x, double y, double up, double down, double left, double right)
{
    return x >= left && x <= right && y >= down && y <= up;
}

inline double p2p(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

inline bool pointInCircle(double x1, double y1, double x2, double y2, double r)
{
    return p2p(x1, y1, x2, y2) <= r;
}

// 叉乘
inline double cross(double x1, double y1, double x2, double y2)
{
    return x1 * y2 - x2 * y1;
}

// 点乘
inline double dot(double x1, double y1, double x2, double y2)
{
    return x1 * x2 + y1 * y2;
}

inline bool pointOnLine(double x, double y, double x1, double y1, double x2, double y2)
{
    return cross(x - x1, y - y1, x2 - x1, y2 - y1) == 0.0;
}

inline bool pointOnLineSegment(double x, double y, double x1, double y1, double x2, double y2)
{
    return pointInRect(x, y, x1, y1, x2, y2) && pointOnLine(x, y, x1, y1, x2, y2);
}

inline bool segmentsIntersect(double x1, double y1, double x2, double y2,
                              double x3, double y3, double x4, double y4)
{
    // 检查 x 范围重叠
    bool overlapX = between(x1, x2, x3, x4);

    // 检查 y 范围重叠
    bool overlapY = between(y1, y2, y3, y4);

    // 检查是否共线
    bool collinear = pointOnLine(x1, y1, x3, y3, x4, y4);

    // 向量叉乘，跨立试验
    double p12xp13 = cross(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
    double p12xp14 = cross(x2 - x1, y2 - y1, x4 - x1, y4 - y1);

    double p34xp32 = cross(x4 - x3, y4 - y3, x2 - x3, y2 - y3);
    double p34xp31 = cross(x4 - x3, y4 - y3, x1 - x3, y1 - y3);

    // 如果存在重叠且不共线,或者有一个端点在另一线段上,则相交
    return (overlapX && overlapY && !collinear) && p12xp13 * p12xp14 < 0.0 && p34xp32 * p34xp31 < 0.0;
}

inline bool pointInPolygon(double x, double y, const vector<tuple<double, double, double, double>> &py)
{
    double x1, y1, x2, y2;
    double tmp;
    int count = 0;
    const int n = py.size();
    for (int i = 0; i < n; i++)
    {
        x1 = get<0>(py[i]);
        y1 = get<1>(py[i]);
        x2 = get<2>(py[i]);
        y2 = get<3>(py[i]);
        if (pointOnLineSegment(x, y, x1, y1, x2, y2))
            return true;

        if (between(y, y1, y2) && y1 != y2)
        {
            if (y1 > y2)
            {
                tmp = y1;
                y1 = y2;
                y2 = tmp;
                tmp = x1;
                x1 = x2;
                x2 = tmp;
            }
            if (cross(x2 - x1, y2 - y1, x - x1, y - y1) < 0.0 && y != y2)
                count++;
        }
    }
    return count % 2 == 1;
}

bool collision(const vector<pair<double, double>> &pa, const vector<pair<double, double>> &pb)
{
    int n;
    double x1, y1, x2, y2;
    double dx, dy;
    vector<pair<double, double>> axes;

    n = pa.size();
    for (int i = 0; i < n; i++)
    {
        x1 = pa[i].first;
        y1 = pa[i].second;

        x2 = pa[(i + 1) % n].first;
        y2 = pa[(i + 1) % n].second;

        dx = x2 - x1;
        dy = y2 - y1;
        axes.emplace_back(make_pair(dy, -dx));
    }

    n = pb.size();
    for (int i = 0; i < n; i++)
    {
        x1 = pb[i].first;
        y1 = pb[i].second;

        x2 = pb[(i + 1) % n].first;
        y2 = pb[(i + 1) % n].second;

        dx = x2 - x1;
        dy = y2 - y1;
        axes.emplace_back(make_pair(dy, -dx));
    }

    double prj;
    double min1, max1;
    double min2, max2;
    for (const auto &ax : axes)
    {
        x1 = ax.first;
        y1 = ax.second;

        min1 = 999.9;
        max1 = -999.9;
        for (int i = 0; i < pa.size(); i++)
        {
            x2 = pa[i].first;
            y2 = pa[i].second;

            prj = x1 * x2 + y1 * y2;
            if (prj < min1)
                min1 = prj;
            else if (prj > max1)
                max1 = prj;
        }

        min2 = 999.9;
        max2 = -999.9;
        for (int i = 0; i < pb.size(); i++)
        {
            x2 = pb[i].first;
            y2 = pb[i].second;

            prj = x1 * x2 + y1 * y2;
            if (prj < min2)
                min2 = prj;
            else if (prj > max2)
                max2 = prj;
        }

        if (max1 < min2 || min1 > max2)
            return false;
    }

    return true;
}

void insertLineSegment(vector<pair<double, double>> &pa, vector<pair<double, double>> &pb)
{
    double x1, y1, x2, y2, x3, y3, x4, y4;

    x1 = pa.cbegin()->first;
    y1 = pa.cbegin()->second;

    x2 = pa.cend()->first;
    y2 = pa.cend()->second;

    x3 = pb.cbegin()->first;
    y3 = pb.cbegin()->second;

    x4 = pb.cend()->first;
    y4 = pb.cend()->second;

    if (!between(x1, x2, x3, x4) || !between(y1, y2, y3, y4))
        return;

    double dx1, dy1, dx2, dy2;
    dx1 = x2 - x1;
    dy1 = y2 - y1;
    dx2 = x4 - x3;
    dy2 = y4 - y3;

    if (dx1 * dy2 - dx2 * dy1 == 0.0)
    {
        return;
    }
}

int main()
{
    tuple<double, double, double, double> p(0.0, 1.0, 2.0, 2.5);
    get<0>(p);
    make_tuple(0.0, 1.0, 2.0, 2.5);
    return 0;
}