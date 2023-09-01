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

enum Position
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
    Position pp;
    Point(double mX = 0.0, double mY = 0.0, Position mPp = UNKNOW)
        : x(mX), y(mY), pp(mPp) {}
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
    Position pp;

    Segment(Point mSp = {}, Point mEp = {}, Position mPp = UNKNOW) : sp(mSp), ep(mEp), pp(mPp) {}

    inline int getSize() const { return 1 + ip.size(); };

    void insertPoint(double x, double y)
    {
        Point p{x, y, ON};
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
            double tmp;
            tmp = y1, y1 = y2, y2 = tmp;
            tmp = x1, x1 = x2, x2 = tmp;
        }
        if (cross(x2 - x1, y2 - y1, x - x1, y - y1) < 0.0 && y != y2)
            return 1;

        return 0;
    }

    vector<Segment> toSegment() const
    {
        const int n = getSize();
        vector<Point> vp;
        vp.reserve(n + 1);
        vp.emplace_back(sp);
        vp.insert(vp.cend(), ip.cbegin(), ip.cend());
        vp.emplace_back(ep);

        vector<Segment> vs;
        for (int i = 0; i < n; i++)
            vs.emplace_back(vp[i], vp[i + 1]);
        return vs;
    }
};

Position pointInPolygon(Point p, const vector<Segment> &vs)
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

vector<vector<Segment>> cover(const vector<Segment> &vs_left, const vector<Segment> &vs_right)
{
    vector<Segment> vs;
    vs.insert(vs.cend(), vs_left.cbegin(), vs_left.cend());
    vs.insert(vs.cend(), vs_right.cbegin(), vs_right.cend());

    vector<vector<Segment>> vvs;
    while (!vs.empty())
    {
        vector<Segment> tmp;
        tmp.push_back(vs.back());
        vs.pop_back();

        vector<Segment>::const_iterator iter;
        do
        {
            iter = vs.cbegin();
            if (distancePoint(tmp.back().ep, iter->sp))
            {
                tmp.push_back(*iter);
                vs.erase(iter);
                continue;
            }
            iter++;
        } while (iter != vs.cend());

        if (distancePoint(tmp.front().sp, tmp.back().ep))
            vvs.emplace_back(tmp);
    }

    return vvs;
}

vector<vector<Segment>> intersectSegment(vector<Segment> &left, vector<Segment> &right)
{
    int n = 0;
    for (int i = 0; i < left.size(); i++)
    {
        double x1 = left[i].sp.x;
        double y1 = left[i].sp.y;
        double x2 = left[i].ep.x;
        double y2 = left[i].ep.y;

        for (int j = 0; j < right.size(); j++)
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

                left[i].insertPoint(x, y);
                right[j].insertPoint(x, y);
                n++;
            }
        }
    }

    vector<Segment> vs_left;
    vs_left.reserve(left.size() + n);
    for (const Segment &seg : left)
    {
        vector<Segment> vs = seg.toSegment();
        vs_left.insert(vs_left.cend(), vs.cbegin(), vs.cend());
        // cout << "vs_size: " << vs.size() << endl;
    }
    const int n_left = vs_left.size();
    // cout << "n_left: " << n_left << endl;
    for (int i = 0; i < n_left; i++)
    {
        if (vs_left[i].sp.pp == UNKNOW)
        {
            int last = 0;
            while (vs_left[(i + last - 1 + n_left) % n_left].sp.pp == UNKNOW)
                last--;
            int next = 0;
            while (vs_left[(i + next + 1) % n_left].sp.pp == UNKNOW)
                next++;
            Position pp = pointInPolygon(vs_left[i].sp, right);

            for (int j = last; j <= next; j++)
                vs_left[(i + j + n_left) % n_left].sp.pp = pp;
            // cout << "i:" << i << ", pp: " << pp << ", last: " << last << ", next: " << next << endl;
        }
    }
    for (int i = 0; i < n_left; i++)
    {
        Point cur_p = vs_left[i].sp;
        if (cur_p.pp == IN || cur_p.pp == OUT)
        {
            vs_left[i].pp = cur_p.pp;
            continue;
        }

        Point next_p = vs_left[(i + 1) % n_left].sp;
        if (next_p.pp == IN || next_p.pp == OUT)
        {
            vs_left[i].pp = next_p.pp;
            continue;
        }

        Point mid_p{cur_p.x + next_p.x, cur_p.y + next_p.y};
        Position pp = pointInPolygon(mid_p, right);
        vs_left[i].pp = pp;

        cout << vs_left[i].sp.pp << endl;
    }

    vector<Segment> vs_right;
    vs_right.reserve(right.size() + n);
    for (Segment seg : right)
    {
        vector<Segment> vs = seg.toSegment();
        vs_right.insert(vs_right.cend(), vs.cbegin(), vs.cend());
    }
    const int n_right = vs_right.size();
    cout << "n_right: " << n_right << endl;
    for (int i = 0; i < n_right; i++)
    {
        if (vs_right[i].sp.pp == UNKNOW)
        {
            int last = 0;
            while (vs_right[(i + last - 1 + n_right) % n_right].sp.pp == UNKNOW)
                last--;
            int next = 0;
            while (vs_right[(i + next + 1) % n_right].sp.pp == UNKNOW)
                next++;
            Position pp = pointInPolygon(vs_right[i].sp, left);

            for (int j = last; j <= next; j++)
                vs_right[(i + j + n_right) % n_right].sp.pp = pp;
        }
    }
    for (int i = 0; i < n_left; i++)
    {
        Position pp;

        Point cur_p = vs_right[i].sp;
        pp = cur_p.pp;
        if (pp == IN || pp == OUT)
        {
            vs_right[i].pp = pp;
            continue;
        }

        Point next_p = vs_right[(i + 1) % n_left].sp;
        pp = next_p.pp;
        if (pp == IN || pp == OUT)
        {
            vs_right[i].pp = pp;
            continue;
        }

        Point mid_p{cur_p.x + next_p.x, cur_p.y + next_p.y};
        pp = pointInPolygon(mid_p, right);
        vs_right[i].pp = pp;

        cout << vs_right[i].sp.pp << endl;
    }

    return cover(vs_left, vs_right);
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
    for (auto vs : vvs)
    {
        for (auto seg : vs)
        {
            for (int i = 0; i < seg.getSize(); i++)
            {
                auto &p = seg.sp;
                cout << p.x << ", " << p.y << ", " << p.pp << endl;
            }
        }
    }

    return 0;
}