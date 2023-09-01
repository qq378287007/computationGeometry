#include <vector>
#include <utility>
#include <tuple>
#include <cmath>
using namespace std;

// 相等
inline bool equal(double x1, double x2, double error = 0.00001)
{
    return abs(x1 - x2) <= error;
}

// 叉乘
inline double cross(double x1, double y1, double x2, double y2)
{
    return x1 * y2 - x2 * y1;
}

inline bool overlap(double x1, double x2, double x3, double x4)
{
    double tmp;
    if (x1 > x2)
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;
    }
    if (x3 > x4)
    {
        tmp = x3;
        x3 = x4;
        x4 = tmp;
    }
    return x1 <= x4 && x2 >= x3;
}

vector<pair<double, double>> intersectPoint(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
    vector<pair<double, double>> points;

    if (!overlap(x1, x2, x3, x4) || !overlap(y1, y2, y3, y4))
        return points;

    if (equal(cross(x1 - x3, y1 - y3, x2 - x3, y2 - y3), 0.0)) // 共线
    {
        double tmp;
        if (x1 > x2)
        {
            tmp = x1;
            x1 = x2;
            x2 = tmp;
            tmp = y1;
            y1 = y2;
            y2 = tmp;
        }

        if (x3 > x4)
        {
            tmp = x3;
            x3 = x4;
            x4 = tmp;
            tmp = y3;
            y3 = y4;
            y4 = tmp;
        }

        double x0[4] = {x1, x2, x3, x4};
        double y0[4] = {y1, y2, y3, y4};

        int left_ind = x1 > x3 ? 0 : 2;
        double x = x0[left_ind];
        double y = y0[left_ind];
        points.emplace_back(x, y);

        int right_ind = x2 < x4 ? 1 : 3;
        if (!equal(x0[left_ind], x0[right_ind]))
        {
            x = x0[right_ind];
            y = y0[right_ind];
            points.emplace_back(x, y);
        }
    }
    else
    {
        // 向量叉乘，跨立试验
        double p12xp13 = cross(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
        double p12xp14 = cross(x2 - x1, y2 - y1, x4 - x1, y4 - y1);

        double p34xp32 = cross(x4 - x3, y4 - y3, x2 - x3, y2 - y3);
        double p34xp31 = cross(x4 - x3, y4 - y3, x1 - x3, y1 - y3);

        if (p12xp13 * p12xp14 <= 0.0 && p34xp32 * p34xp31 <= 0.0)
        {
            double s = cross(x1 - x3, y1 - y3, x2 - x1, y2 - y1) / cross(x4 - x3, y4 - y3, x2 - x1, y2 - y1);
            double x = x1 + s * (x2 - x1);
            double y = y1 + s * (y2 - y1);
            points.emplace_back(x, y);
        }
    }

    return points;
}

enum PointPosition
{
    UNKNOW,
    IN,
    ON,
    OUT,
};

struct Point
{
    double x;
    double y;
    bool flag{false};
    PointPosition pp{UNKNOW};
};

bool equalPoint(Point a, Point b, double error = 0.00001)
{
    return equal(a.x, b.x, error) && equal(a.y, b.y, error);
}

double distancePoint(Point a, Point b)
{
    return pow(a.x - b.x, 2) + pow(a.y - b.y, 2);
}

struct Segment
{
    Point sp;
    Point ep; // 不包含
    vector<Point> ip;

    void insertPoint(double x, double y)
    {
        Point p;
        p.x = x;
        p.y = y;
        p.flag = true;
        p.pp = ON;
        if (equalPoint(p, sp))
        {
            sp.flag = true;
            sp.pp == ON;
        }
        else if (equalPoint(p, ep)) // 不会执行
        {
            ep.flag = true;
            ep.pp == ON;
        }
        else
        {
            const int n = ip.size();
            for (int i = 0; i < n; i++)
                if (equalPoint(p, ip[i]))
                    return;

            const double disatnce = distancePoint(p, sp);
            int index = 0;
            while (index < n && distancePoint(ip[index], sp) < disatnce)
                index++;
            ip.insert(ip.cbegin() + index, p);
        }
    }

    bool containPoint(Point p) const
    {
        double x1 = sp.x;
        double x2 = ep.x;
        double x = p.x;
        if ((x - x1) * (x - x2) > 0.0)
            return false;

        double y1 = sp.y;
        double y2 = ep.y;
        double y = p.y;
        if ((y - y1) * (y - y2) > 0.0)
            return false;

        return cross(x - x1, y - y1, x - x2, y - y2) == 0.0;
    }

    int crossPoint(double x, double y) const
    {
        double x1 = sp.x;
        double x2 = ep.x;
        if (x < min(x1, x2))
            return 0;

        double y1 = sp.y;
        double y2 = ep.y;
        if ((y - y1) * (y - y2) > 0.0)
            return 0;

        if (y1 == y2)
            return 0;

        if (y1 > y2)
        {
            double tmp = y1;
            y1 = y2;
            y2 = tmp;
            tmp = x1;
            x1 = x2;
            x2 = tmp;
        }
        if (cross(x2 - x1, y2 - y1, x - x1, y - y1) < 0.0 && y != y2)
            return 1;

        return 0;
    }

    inline int getSize() const { return 1 + ip.size(); };

    Point &getPoint(int index)
    {
        if (index == 0)
            return sp;
        else
            return ip[index - 1];
    }

    int getIndex(double x, double y)
    {
        for (int index = 0; index < getSize(); index++)
        {
            const Point &p = getPoint(index);
            if (equal(p.x, x) && equal(p.y, y))
                return index;
        }
        return -1;
    }
};

PointPosition pointInPolygon(Point p, const vector<Segment> &vs)
{
    const int n = vs.size();
    const double x = p.x;
    const double y = p.y;
    int count = 0;
    for (int i = 0; i < n; i++)
    {
        if (vs[i].containPoint(p))
            return ON;
        count += vs[i].crossPoint(x, y);
    }
    if (count % 2 == 1)
        return IN;
    else
        return OUT;
}

void setPointPosition(vector<Segment> &left, vector<Segment> &right)
{
    for (int i = 0; i < left.size(); i++)
    {
        for (int j = 0; j < left[i].getSize(); j++)
        {
            Point &p = left[i].getPoint(j);
            if (p.pp == UNKNOW)
                p.pp = pointInPolygon(p, right);
        }
    }

    for (int i = 0; i < right.size(); i++)
    {
        for (int j = 0; j < right[i].getSize(); j++)
        {
            Point &p = right[i].getPoint(j);
            if (p.pp == UNKNOW)
                p.pp = pointInPolygon(p, left);
        }
    }
}

Point nextPoint(vector<Segment> &vs, int &i, int &j)
{
    j++;
    if (j == vs[i].getSize())
    {
        j = 0;
        i = (i + 1) % vs.size();
    }
    return vs[i].getPoint(j);
}

vector<Segment> getSegment(vector<Segment> &in_vs, double x, double y, int &i)
{
    int j = in_vs[i].getIndex(x, y);

    vector<Segment> out_vs;
    Point sp;
    Point ep;
    Segment seg;

    while (true)
    {
        ep = nextPoint(in_vs, i, j);
        if (ep.pp == OUT)
            break;

        seg.sp = sp;
        seg.ep = ep;
        out_vs.emplace_back(seg);
        sp = ep;
    }

    return out_vs;
}

vector<vector<Segment>> intersectSegment(vector<Segment> &left, vector<Segment> &right)
{
    const int n_left = left.size();
    const int n_right = right.size();
    vector<tuple<double, double, int, int>> points; // 交点及对应片段序号
    for (int i = 0; i < n_left; i++)
    {
        double x1 = left[i].sp.x;
        double y1 = left[i].sp.y;
        double x2 = left[i].ep.x;
        double y2 = left[i].ep.y;

        for (int j = 0; j < n_right; j++)
        {
            double x3 = right[j].sp.x;
            double y3 = right[j].sp.y;
            double x4 = right[j].ep.x;
            double y4 = right[j].ep.y;

            for (const auto &point : intersectPoint(x1, y1, x2, y2, x3, y3, x4, y4))
            {
                double x = point.first;
                double y = point.second;
                if (!(equal(x2, x) && equal(y2, y) || equal(x4, x) && equal(y4, y)))
                {
                    points.emplace_back(x, y, i, j);
                    left[i].insertPoint(x, y);
                    right[j].insertPoint(x, y);
                }
            }
        }
    }

    // 所有点分为内部点、外部点、交点
    setPointPosition(left, right);

    vector<vector<Segment>> vvs;
    while (!points.empty())
    {
        tuple<double, double, int, int> first = points.back();
        points.pop_back();

        double x = get<0>(first);
        double y = get<1>(first);
        int left_i = get<2>(first);
        int right_i = get<3>(first);

        double cur_x = x;
        double cur_y = y;
        bool left_flag = true;

        vector<Segment> vs;
        vector<Segment> out_vs;
        while (true)
        {
            out_vs = getSegment(left_flag ? left : right, cur_x, cur_y, left_flag ? left_i : right_i);
            left_flag = !left_flag;

            if (out_vs.empty())
            {
                out_vs = getSegment(left_flag ? left : right, cur_x, cur_y, left_flag ? left_i : right_i);
                left_flag = !left_flag;
            }

            if (out_vs.empty())
                break;

            cur_x = out_vs.back().ep.x;
            cur_y = out_vs.back().ep.y;
            vs.insert(vs.cend(), out_vs.cbegin(), out_vs.cend());
            if (equal(x, cur_x) && equal(y, cur_y))
                break;
        }
        if (!vs.empty())
            vvs.emplace_back(vs);
    }
    return vvs;
}

int main()
{
    return 0;
}