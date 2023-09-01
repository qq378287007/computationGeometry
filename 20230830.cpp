#include <vector>
#include <cmath>
#include <iostream>
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

// 区间[x1, x2]，[x3, x4]是否重叠
inline bool overlap(double x1, double x2, double x3, double x4)
{
    double tmp;
    if (x1 > x2)
    {
        tmp = x1, x1 = x2, x2 = tmp;
    }
    if (x3 > x4)
    {
        tmp = x3, x3 = x4, x4 = tmp;
    }
    return x1 <= x4 && x2 >= x3;
}

// 线段[(x1, y1), (x2, y2)], [(x3, y3), (x4, y4)]交点
vector<pair<double, double>> intersectPoint(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
    vector<pair<double, double>> points;

    if (!overlap(x1, x2, x3, x4)) // x轴投影不重叠
        return points;

    if (!overlap(y1, y2, y3, y4)) // y轴投影不重叠
        return points;

    if (equal(cross(x2 - x1, y2 - y1, x4 - x3, y4 - y3), 0.0)) // 直线平行
    {
        if (equal(cross(x1 - x3, y1 - y3, x2 - x3, y2 - y3), 0.0)) // 三点共线
        {
            double tmp;
            if (x1 > x2)
            {
                tmp = x1, x1 = x2, x2 = tmp;
                tmp = y1, y1 = y2, y2 = tmp;
            }

            if (x3 > x4)
            {
                tmp = x3, x3 = x4, x4 = tmp;
                tmp = y3, y3 = y4, y4 = tmp;
            }

            double x0[4] = {x1, x2, x3, x4};
            double y0[4] = {y1, y2, y3, y4};

            int left_ind = x1 > x3 ? 0 : 2;
            points.emplace_back(x0[left_ind], y0[left_ind]);

            int right_ind = x2 < x4 ? 1 : 3;
            if (!equal(x0[left_ind], x0[right_ind]))
                points.emplace_back(x0[right_ind], y0[right_ind]);
        }
    }
    else // 直线相交
    {
        // 向量叉乘，跨立试验
        double p12xp13 = cross(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
        double p12xp14 = cross(x2 - x1, y2 - y1, x4 - x1, y4 - y1);
        if (p12xp13 * p12xp14 > 0.0) // (x3, y3), (x4, y4)位于[(x1, y1), (x2, y2)]同侧
            return points;

        double p34xp32 = cross(x4 - x3, y4 - y3, x2 - x3, y2 - y3);
        double p34xp31 = cross(x4 - x3, y4 - y3, x1 - x3, y1 - y3);

        if (p34xp32 * p34xp31 > 0.0) //(x1, y1), (x2, y2)位于[(x3, y3), (x4, y4)]同侧
            return points;

        // 线段相交
        double s = cross(x1 - x3, y1 - y3, x2 - x1, y2 - y1) / cross(x4 - x3, y4 - y3, x2 - x1, y2 - y1);
        double x = x1 + s * (x2 - x1);
        double y = y1 + s * (y2 - y1);
        points.emplace_back(x, y);
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
    PointPosition pp;
    bool visited;
    int left_i;
    int right_i;
    Point(double mX = 0.0, double mY = 0.0, PointPosition mPp = UNKNOW, bool mVisited = false, int mLeft_i = -1, int mRight_i = -1)
        : x(mX), y(mY), pp(mPp), visited(mVisited), left_i(mLeft_i), right_i(mRight_i) {}
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
    Point ep;
    vector<Point> ip;
    Segment(Point mSp = {}, Point mEp = {}) : sp(mSp), ep(mEp) {}

    inline int getSize() const { return 1 + ip.size(); };

    void insertPoint(double x, double y, int left_i, int right_i)
    {
        Point p{x, y, ON, false, left_i, right_i};
        if (equalPoint(p, sp))
        {
            sp = p;
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

    Point &getPoint(int index)
    {
        if (index == 0)
            return sp;
        else
            return ip[index - 1];
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

        return equal(cross(x - x1, y - y1, x - x2, y - y2), 0.0);
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

        if (equal(y1, y2))
            return 0;

        if (y1 > y2)
        {
            double tmp = y1, y1 = y2, y2 = tmp;
            tmp = x1, x1 = x2, x2 = tmp;
        }
        if (cross(x2 - x1, y2 - y1, x - x1, y - y1) < 0.0 && y != y2)
            return 1;

        return 0;
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

bool findFirstIntersectPoint(vector<Segment> &vs, int &i, int &j)
{
    for (i = 0; i < vs.size(); i++)
    {
        for (j = 0; j < vs[i].getSize(); j++)
        {
            Point &p = vs[i].getPoint(j);
            if (p.pp == ON && p.visited == false)
            {
                p.visited = true;
                return true;
            }
        }
    }
    return false;
}

void nextPoint(vector<Segment> &vs, int &i, int &j, Point &p)
{
    j++;
    if (j == vs[i].getSize())
    {
        j = 0;
        i = (i + 1) % vs.size();
    }
    p = vs[i].getPoint(j);
}

bool getIndex(vector<Segment> &vs, double x, double y, int &i, int &j)
{
    int start_i = i;
    int start_j = j;

    do
    {
        Point p = vs[i].getPoint(j);
        if (equal(p.x, x) && equal(p.y, y))
            return true;
        nextPoint(vs, i, j, p);
    } while (!(i == start_i && j == start_j));
    return false;
}

vector<Segment> getSegment(vector<Segment> &left, int &i, int &j)
{
    int start_i = i;
    int start_j = j;

    Point sp;
    Point ep;
    vector<Segment> vs;

    ep = left[i].getPoint(j);
    do
    {
        sp = ep;
        nextPoint(left, i, j, ep);
        if (ep.pp == OUT)
            break;

        vs.emplace_back(sp, ep);

    } while (!(i == start_i && j == start_j));

    return vs;
}

vector<vector<Segment>> intersectSegment(vector<Segment> &left, vector<Segment> &right)
{
    const int n_left = left.size();
    const int n_right = right.size();
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

                if ((equal(x2, x) && equal(y2, y)) || (equal(x4, x) && equal(y4, y)))
                    continue;

                left[i].insertPoint(x, y, i, j);
                right[j].insertPoint(x, y, i, j);
            }
        }
    }

    // 所有点分为内部点、外部点、交点
    setPointPosition(left, right);

    vector<vector<Segment>> vvs;
    int i;
    int j;
    while (findFirstIntersectPoint(left, i, j))
    {
        bool flag = true;
        Point first_p = left[i].getPoint(j);
        Point last_p = first_p;

        vector<Segment> vs;
        vector<Segment> tmp;
        double x;
        double y;
        do
        {
            x = last_p.x;
            y = last_p.y;
            j = 0;

            if (flag)
            {
                i = last_p.left_i;
                getIndex(left, x, y, i, j);
                tmp = getSegment(left, i, j);
            }
            else
            {
                i = last_p.right_i;
                getIndex(right, x, y, i, j);
                tmp = getSegment(right, i, j);
            }
            flag = !flag;
            if (tmp.empty())
                break;

            vs.insert(vs.cend(), tmp.cbegin(), tmp.cend());
            last_p = vs.back().ep;
        } while (!equalPoint(first_p, last_p));
        if (!vs.empty())
            vvs.emplace_back(vs);
    }
    return vvs;
}

vector<vector<Segment>> differenceSegment(vector<Segment> &left, vector<Segment> &right)
{
    vector<vector<Segment>> vvs_intersect = intersectSegment(left, right);
    vector<vector<Segment>> vvs;
    if (vvs_intersect.empty())
        return vvs;

    return vvs;
}

vector<vector<Segment>> intersectSegment2(vector<Segment> &left, vector<Segment> &right)
{
    const int n_left = left.size();
    const int n_right = right.size();
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

                if ((equal(x2, x) && equal(y2, y)) || (equal(x4, x) && equal(y4, y)))
                    continue;

                left[i].insertPoint(x, y, i, j);
                right[j].insertPoint(x, y, i, j);
            }
        }
    }

    // 所有点分为内部点、外部点、交点
    setPointPosition(left, right);

    vector<vector<Segment>> vvs;
    return vvs;
}

vector<vector<Segment>> unionSegment(vector<Segment> &left, vector<Segment> &right)
{
    const int n_left = left.size();
    const int n_right = right.size();
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

                if ((equal(x2, x) && equal(y2, y)) || (equal(x4, x) && equal(y4, y)))
                    continue;

                left[i].insertPoint(x, y, i, j);
                right[j].insertPoint(x, y, i, j);
            }
        }
    }

    // 所有点分为内部点、外部点、交点
    setPointPosition(left, right);

    vector<vector<Segment>> vvs;
    int i;
    int j;
    while (findFirstIntersectPoint(left, i, j))
    {
        bool flag = true;
        Point first_p = left[i].getPoint(j);
        Point last_p = first_p;

        vector<Segment> vs;
        vector<Segment> tmp;
        double x;
        double y;
        do
        {
            x = last_p.x;
            y = last_p.y;
            j = 0;

            if (flag)
            {
                i = last_p.left_i;
                getIndex(left, x, y, i, j);
                tmp = getSegment(left, i, j);
            }
            else
            {
                i = last_p.right_i;
                getIndex(right, x, y, i, j);
                tmp = getSegment(right, i, j);
            }
            flag = !flag;
            if (tmp.empty())
                break;

            vs.insert(vs.cend(), tmp.cbegin(), tmp.cend());
            last_p = vs.back().ep;
        } while (!equalPoint(first_p, last_p));
        if (!vs.empty())
            vvs.emplace_back(vs);
    }
    return vvs;
}

vector<Segment> createSegment(const vector<Point> &points)
{
    const int n = points.size();
    vector<Segment> vs;
    for (int i = 0; i < n; i++)
        vs.emplace_back(points[i], points[(i + 1) % n]);
    return vs;
}

int main()
{
    vector<Segment> left = createSegment({{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}});
    vector<Segment> right = createSegment({{2.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}, {2.0, 0.0}});

    vector<vector<Segment>> vvs = intersectSegment(left, right);

    return 0;
}