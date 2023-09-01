#include <vector>
#include <cmath>
#include <iostream>
#include <memory>
using namespace std;

// 相等
inline bool equal(double x1, double x2, double error = 1.0e-6)
{
    return abs(x1 - x2) <= error;
}

struct Point
{
    double x;
    double y;
    Point(double mX = 0.0, double mY = 0.0) : x(mX), y(mY) {}
};

bool equalPoint(Point a, Point b, double error = 0.00001)
{
    return equal(a.x, b.x, error) && equal(a.y, b.y, error);
}

double distancePoint(Point a, Point b)
{
    return pow(b.x - a.x, 2) + pow(b.y - a.y, 2);
}

double anglePoint(Point a, Point b)
{
    return atan2(b.y - a.y, b.x - a.x);
}

void swapPoint(Point &a, Point &b)
{
    Point tmp;
    tmp = a, a = b, b = tmp;
}

struct SegmentLine
{
    Point sp;
    Point ep;
    vector<Point> ip;
    SegmentLine(Point mSp = {}, Point mEp = {}) : sp(mSp), ep(mEp) {}
    virtual ~SegmentLine() = default;

    virtual void reverse()
    {
        swapPoint(sp, ep);
        for (int i = 0, j = ip.size() - 1; i < j; i++, j--)
            swapPoint(ip[i], ip[j]);
    }
    virtual Point centerPoint() const
    {
        return Point((sp.x + ep.x) * 0.5, (sp.y + ep.y) * 0.5);
    }
};

struct ArcLine : SegmentLine
{
    Point cp;
    double r;
    bool anticlockwise;
    ArcLine(Point mSp = {}, Point mEp = {}, Point mCp = {}, double mR = 0.0, bool mAnticlockwise = true)
        : SegmentLine(mSp, mEp), cp(mCp), r(mR), anticlockwise(mAnticlockwise) {}

    virtual void reverse() override
    {
        SegmentLine::reverse();
        anticlockwise = !anticlockwise;
    }
    virtual Point centerPoint() const override
    {
        double start_angle = anglePoint(cp, sp);
        double end_angle = anglePoint(cp, ep);

        if (anticlockwise)
        {
            while (end_angle > start_angle)
                end_angle -= 2 * M_PI;
            while (end_angle < start_angle)
                end_angle += 2 * M_PI;
        }
        else
        {
            while (end_angle < start_angle)
                end_angle += 2 * M_PI;
            while (end_angle > start_angle)
                end_angle -= 2 * M_PI;
        }
        double center_angle = (start_angle + end_angle) * 0.5;
        double x = cp.x + r * cos(center_angle);
        double y = cp.y + r * sin(center_angle);

        return Point(x, y);
    }
};

enum Shape
{
    CLOSEPATH,
    CIRCLE,
    POLYGON,
};

struct ClosePath
{
    vector<shared_ptr<ClosePath>> inClosePath;
    Shape shape{CLOSEPATH};
    virtual ~ClosePath() = default;
};

struct Circle : ClosePath
{
    Point cp;
    double r;
    bool anticlockwise;
    vector<Point> ip;
    Circle(Point mCp = {}, double mR = 0.0, bool mAnticlockwise = true)
        : cp(mCp), r(mR), anticlockwise(mAnticlockwise)
    {
        shape = CIRCLE;
    }
    vector<ArcLine> toArc() const
    {
        const int n = ip.size();
        vector<ArcLine> arcs;
        arcs.reserve(n);
        for (int i = 0; i < n; i++)
            arcs.emplace_back(ip[i], ip[(i + 1) % n], cp, r);
        return arcs;
    }
    void addPoint(double x, double y)
    {
        Point p{x, y};
        const int n = ip.size();
        for (int i = 0; i < n; i++)
            if (equalPoint(p, ip[i]))
                return;

        const double angle = anglePoint(cp, p);
        int index = 0;
        while (index < n && anglePoint(cp, ip[index]) < angle)
            index++;
        ip.insert(ip.cbegin() + index, p);
    }
};

void swapCirlce(Circle &c1, Circle &c2)
{
    Circle tmp;
    tmp = c1, c1 = c2, c2 = tmp;
}

struct Polygon : ClosePath
{
    vector<shared_ptr<SegmentLine>> lines;
    Polygon()
    {
        shape = POLYGON;
    }
    void addLine(shared_ptr<SegmentLine> line)
    {
        lines.emplace_back(line);
    }
};

// 求相交圆的交点（两个交点）
// 假定两圆会相交，且r1<=r2
// 交点（x1，y1）位于矢量p1p2左侧
// 交点（x2，y3）位于矢量p1p2右侧
void insertCirclePoint(double p1_x, double p1_y, double r1, double p2_x, double p2_y, double r2,
                       double &x1, double &y1, double &x2, double &y2)
{
    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));
    double sr = sqrt(r2 * r2 - r1 * r1);
    if (d < sr) // 圆心靠近
    {
        double sx = (sr * sr - d * d) / (2 * d);
        double h = sqrt(r1 * r1 - sx * sx);

        // 向量p2p1
        double p2p1_x = p1_x - p2_x;
        double p2p1_y = p1_y - p2_y;
        // 单位向量p2p1
        double length = sqrt(p2p1_x * p2p1_x + p2p1_y * p2p1_y);
        p2p1_x /= length;
        p2p1_y /= length;

        // 向量p2p1垂直向量的垂直向量，且位于右侧
        double d_p2p1_x = p2p1_y;
        double d_p2p1_y = -p2p1_x;

        // 交点
        x1 = p1_x + sx * p2p1_x + h * d_p2p1_x;
        y1 = p1_y + sx * p2p1_y + h * d_p2p1_y;

        x2 = p1_x + sx * p2p1_x - h * d_p2p1_x;
        y2 = p1_y + sx * p2p1_y - h * d_p2p1_y;
    }
    else if (d > sr) // 圆心较远
    {
        double sx = (d * d - sr * sr) / (2 * d);
        double h = sqrt(r1 * r1 - sx * sx);

        // 向量p1p2
        double p1p2_x = p2_x - p1_x;
        double p1p2_y = p2_y - p1_y;
        // 单位向量p1p2
        double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
        p1p2_x /= length;
        p1p2_y /= length;

        // 垂直向量
        double d_p1p2_x = -p1p2_y;
        double d_p1p2_y = p1p2_x;

        // 交点
        x1 = p1_x + sx * p1p2_x + h * d_p1p2_x;
        y1 = p1_y + sx * p1p2_y + h * d_p1p2_y;

        x2 = p1_x + sx * p1p2_x - h * d_p1p2_x;
        y2 = p1_y + sx * p1p2_y - h * d_p1p2_y;
    }
    else // 圆心连线垂直
    {
        // 向量p1p2
        double p1p2_x = p2_x - p1_x;
        double p1p2_y = p2_y - p1_y;
        // 单位向量p1p2
        double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
        p1p2_x /= length;
        p1p2_y /= length;

        // 向量p1p2的垂直向量，且位于向量p1p2左侧
        double d_p1p2_x = -p1p2_y;
        double d_p1p2_y = p1p2_x;

        // 交点
        x1 = p1_x + r1 * d_p1p2_x;
        y1 = p1_y + r1 * d_p1p2_y;

        x2 = p1_x - r1 * d_p1p2_x;
        y2 = p1_y - r1 * d_p1p2_y;
    }
}

vector<shared_ptr<ClosePath>> insertCircle(Circle &left, Circle &right)
{
    if (left.r > right.r)
        swapCirlce(left, right);

    double p1_x = left.cp.x;
    double p1_y = left.cp.y;
    double r1 = left.r;
    double p2_x = right.cp.x;
    double p2_y = right.cp.y;
    double r2 = right.r;
    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));
    double min_d = r2 - r1;
    double max_d = r2 + r1;

    vector<shared_ptr<ClosePath>> vp;
    if (min_d == 0.0 && d == 0.0) // 重叠
    {
        vp.emplace_back(make_shared<Circle>(left.cp, left.r));
    }
    else if (d > min_d && d < max_d) // 相交
    {
        double x1, y1;
        double x2, y2;
        insertCirclePoint(p1_x, p1_y, r1, p2_x, p2_y, r2, x1, y1, x2, y2);

        Point sp{x1, y1};
        Point ep{x2, y2};
        auto poly = make_shared<Polygon>();

        poly->addLine(make_shared<ArcLine>(sp, ep, right.cp, right.r));
        swapPoint(sp, ep);
        poly->addLine(make_shared<ArcLine>(sp, ep, left.cp, left.r));
        vp.emplace_back(poly);
    }
    return vp;
}

vector<shared_ptr<ClosePath>> unionCircle(Circle &left, Circle &right)
{
    if (left.r > right.r)
        swapCirlce(left, right);

    double p1_x = left.cp.x;
    double p1_y = left.cp.y;
    double r1 = left.r;
    double p2_x = right.cp.x;
    double p2_y = right.cp.y;
    double r2 = right.r;
    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));
    double min_d = r2 - r1;
    double max_d = r2 + r1;

    vector<shared_ptr<ClosePath>> vp;
    if (min_d == 0.0 && d == 0.0) // 重叠
    {
        vp.emplace_back(make_shared<Circle>(left.cp, left.r));
    }
    else if (d <= min_d) // 内含或内切
    {
        vp.emplace_back(make_shared<Circle>(right.cp, right.r));
    }
    else if (d >= max_d) // 相离或外切
    {
        vp.emplace_back(make_shared<Circle>(left.cp, left.r));
        vp.emplace_back(make_shared<Circle>(right.cp, right.r));
    }
    else if (d > min_d && d < max_d) // 相交
    {
        double x1, y1;
        double x2, y2;
        insertCirclePoint(p1_x, p1_y, r1, p2_x, p2_y, r2, x1, y1, x2, y2);

        Point sp{x1, y1};
        Point ep{x2, y2};
        auto poly = make_shared<Polygon>();

        poly->addLine(make_shared<ArcLine>(sp, ep, left.cp, left.r));
        swapPoint(sp, ep);
        poly->addLine(make_shared<ArcLine>(sp, ep, right.cp, right.r));
        vp.emplace_back(poly);
    }
    return vp;
}

vector<shared_ptr<ClosePath>> difference1Circle(Circle &left, Circle &right)
{
    double p1_x = left.cp.x;
    double p1_y = left.cp.y;
    double r1 = left.r;
    double p2_x = right.cp.x;
    double p2_y = right.cp.y;
    double r2 = right.r;
    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));
    double min_d = abs(r2 - r1);
    double max_d = r2 + r1;

    vector<shared_ptr<ClosePath>> vp;
    if (min_d == 0.0 && d == 0.0) // 重叠
    {
    }
    else if (d <= min_d) // 内含或内切
    {
        if (r1 > r2) // 生成环
        {
            auto c1{make_shared<Circle>(left.cp, left.r)};
            c1->inClosePath.emplace_back(make_shared<Circle>(right.cp, right.r, false));
            vp.emplace_back(c1);
        }
    }
    else if (d >= max_d) // 相离或外切
    {
        vp.emplace_back(make_shared<Circle>(left.cp, left.r));
    }
    else if (d > min_d && d < max_d) // 相交
    {
        bool flag = r1 <= r2;
        if (!flag)
            swapCirlce(left, right);

        double x1, y1;
        double x2, y2;
        insertCirclePoint(p1_x, p1_y, r1, p2_x, p2_y, r2, x1, y1, x2, y2);

        Point sp{x1, y1};
        Point ep{x2, y2};
        auto poly = make_shared<Polygon>();

        poly->addLine(make_shared<ArcLine>(sp, ep, left.cp, left.r, flag));
        swapPoint(sp, ep);
        poly->addLine(make_shared<ArcLine>(sp, ep, right.cp, right.r, !flag));
        vp.emplace_back(poly);
    }
    return vp;
}

vector<shared_ptr<ClosePath>> difference2Circle(Circle &left, Circle &right)
{
    double p1_x = left.cp.x;
    double p1_y = left.cp.y;
    double r1 = left.r;
    double p2_x = right.cp.x;
    double p2_y = right.cp.y;
    double r2 = right.r;
    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));
    double min_d = abs(r2 - r1);
    double max_d = r2 + r1;

    vector<shared_ptr<ClosePath>> vp;
    if (min_d == 0.0 && d == 0.0) // 重叠
    {
    }
    else if (d <= min_d) // 内含或内切
    {
        if (r1 > r2) // 生成环
        {
            auto c1{make_shared<Circle>(left.cp, left.r)};
            c1->inClosePath.emplace_back(make_shared<Circle>(right.cp, right.r, false));
            vp.emplace_back(c1);
        }
        else if (r1 < r2)
        {
            auto c1{make_shared<Circle>(right.cp, right.r)};
            c1->inClosePath.emplace_back(make_shared<Circle>(left.cp, left.r, false));
            vp.emplace_back(c1);
        }
    }
    else if (d >= max_d) // 相离或外切
    {
        vp.emplace_back(make_shared<Circle>(left.cp, left.r));
        vp.emplace_back(make_shared<Circle>(right.cp, right.r));
    }
    else if (d > min_d && d < max_d) // 相交
    {
        if (r1 > r2)
            swapCirlce(left, right);

        double x1, y1;
        double x2, y2;
        insertCirclePoint(p1_x, p1_y, r1, p2_x, p2_y, r2, x1, y1, x2, y2);

        Point sp1{x1, y1};
        Point ep1{x2, y2};
        auto poly1 = make_shared<Polygon>();
        poly1->addLine(make_shared<ArcLine>(sp1, ep1, left.cp, left.r, true));
        swapPoint(sp1, ep1);
        poly1->addLine(make_shared<ArcLine>(sp1, ep1, right.cp, right.r, false));
        vp.emplace_back(poly1);

        Point sp2{x1, y1};
        Point ep2{x2, y2};
        auto poly2 = make_shared<Polygon>();
        poly2->addLine(make_shared<ArcLine>(sp2, ep2, left.cp, left.r, false));
        swapPoint(sp2, ep2);
        poly2->addLine(make_shared<ArcLine>(sp2, ep2, right.cp, right.r, true));
        vp.emplace_back(poly2);
    }
    return vp;
}

int main()
{
    Circle left{{}, 2.0};
    Circle right{{2.0, 0.0}, 2.9};
    vector<shared_ptr<ClosePath>> vp = insertCircle(left, right);
    if (!vp.empty())
    {
        shared_ptr<Circle> c = dynamic_pointer_cast<Circle>(vp[0]);
        if (c != nullptr)
        {
            cout << c->cp.x << endl;
            cout << c->cp.y << endl;
            cout << c->r << endl;
        }
        else
        {
            shared_ptr<Polygon> poly = dynamic_pointer_cast<Polygon>(vp[0]);
            if (poly != nullptr)
            {
                for (auto line : poly->lines)
                {
                    shared_ptr<ArcLine> arc = dynamic_pointer_cast<ArcLine>(line);
                    if (arc != nullptr)
                    {
                        cout << "sx: " << arc->sp.x << ", sy: " << arc->sp.y << ", ex: " << arc->ep.x << ", ey: " << arc->ep.y
                             << ", cx: " << arc->cp.x << ", cy: " << arc->cp.y << ", r: " << arc->r << ", direction: " << arc->anticlockwise << endl;
                    }
                }
            }
        }
    }
    else
    {
        cout << "empty" << endl;
    }

    return 0;
}