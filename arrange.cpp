#include <iostream>
#include <cmath>
#include <tuple>
#include <utility>
#include <vector>
#include <memory>
using namespace std;

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

// 三点共线
inline bool pointOnLine(double x, double y, double x1, double y1, double x2, double y2)
{
    return cross(x - x1, y - y1, x2 - x1, y2 - y1) == 0.0;
}

// 点位于线段上
inline bool pointOnLineSegment(double x, double y, double x1, double y1, double x2, double y2)
{
    int min_x = x1;
    int max_x = x2;
    if (min_x > max_x)
    {
        min_x = x2;
        max_x = x1;
    }
    if (x < min_x || x > max_x)
        return false;

    int min_y = y1;
    int max_y = y2;
    if (min_y > max_y)
    {
        min_y = y2;
        max_y = y1;
    }
    if (y < min_y || y > max_y)
        return false;

    return pointOnLine(x, y, x1, y1, x2, y2);
}

// 凸多边形的碰撞检测
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

// 获取多边形的一个内部点
shared_ptr<pair<double, double>> pointInItem(vector<pair<double, double>> &item)
{
    shared_ptr<pair<double, double>> p;
    const int n = item.size();
    if (n >= 3)
    {
        // 凸顶点和索引
        int cur_index = 0;
        double cur_x = item[0].first;
        double cur_y = item[0].second;
        // 寻找一个凸顶点
        for (int i = 1; i < n; ++i)
        {
            if (cur_y < item[i].second)
            {
                cur_index = i;
                cur_x = item[i].first;
                cur_y = item[i].second;
            }
        }

        int pre_index = (cur_index - 1 + n) % n;
        int next_index = (cur_index + 1 + n) % n;

        double distance;
        double min_d = double(INT_MAX);
        for (int i = 1; i < n; ++i)
        {
            if (i == cur_index)
                continue;
            distance = pow(item[i].first - cur_x, 2) + pow(item[i].second - cur_y, 2);
            if (distance < min_d)
                min_d = distance;

            double dx1 = item[pre_index].first - cur_x;
            double dy1 = item[pre_index].second - cur_y;
            double len1 = sqrt(dx1 * dx1 + dy1 * dy1);
            double dx2 = item[next_index].first - cur_x;
            double dy2 = item[next_index].second - cur_y;
            double len2 = sqrt(dx2 * dx2 + dy2 * dy2);

            double x = cur_x + distance * 0.5 * (dx1 / len1 + dx2 / len2);
            double y = cur_y + distance * 0.5 * (dy1 / len1 + dy2 / len2);
            p = make_shared<pair<double, double>>(x, y);
        }
    }
    return p;
}

int main()
{

    return 0;
}