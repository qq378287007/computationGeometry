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
    Position pos;
    Point(double mX = 0.0, double mY = 0.0, Position mPos = UNKNOW)
        : x(mX), y(mY), pos(mPos) {}
};

bool equalPoint(Point a, Point b, double error = 1.0e-6)
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
    Position pos;
    vector<Point> ip;
    SegmentLine(Point mSp = {}, Point mEp = {}, Position mPos = UNKNOW)
        : sp(mSp), ep(mEp), pos(mPos) {}
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

    virtual vector<shared_ptr<SegmentLine>> toLine() const
    {
        const int n = ip.size();
        vector<Point> vp;
        vp.reserve(n + 1);
        vp.emplace_back(sp);
        vp.insert(vp.cend(), ip.cbegin(), ip.cend());
        vp.emplace_back(ep);

        vector<shared_ptr<SegmentLine>> lines;
        lines.reserve(n);
        for (int i = 0; i < n; i++)
            lines.emplace_back(make_shared<SegmentLine>(vp[i], vp[i + 1]));
        return lines;
    }

    virtual void addPoint(double x, double y)
    {
        Point p{x, y, ON};
        if (equalPoint(p, sp))
        {
            sp = ep;
            return;
        }

        const int n = ip.size();
        const double distance = distancePoint(sp, p);
        int index = 0;
        while (index < n && anglePoint(sp, ip[index]) < distance)
            index++;
        ip.insert(ip.cbegin() + index, p);
    }

    virtual bool containPoint(double x, double y) const
    {
        double x1 = sp.x;
        double x2 = ep.x;
        if ((x - x1) * (x - x2) > 0.0)
            return false;

        double y1 = sp.y;
        double y2 = ep.y;
        if ((y - y1) * (y - y2) > 0.0)
            return false;

        return equal(cross(x - x1, y - y1, x - x2, y - y2), 0.0);
    }

    virtual double getArea() const
    {
        return cross(sp.x, sp.y, ep.x, ep.y) * 0.5;
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

    bool link(shared_ptr<SegmentLine> next) const
    {
        return anglePoint(ep, sp) == anglePoint(next->ep, next->sp);
    }
    shared_ptr<SegmentLine> linkLine(shared_ptr<SegmentLine> next) const
    {
        return make_shared<SegmentLine>(sp, next->ep);
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

    virtual vector<shared_ptr<SegmentLine>> toLine() const override
    {
        const int n = ip.size();
        vector<Point> vp;
        vp.reserve(n + 1);
        vp.emplace_back(sp);
        vp.insert(vp.cend(), ip.cbegin(), ip.cend());
        vp.emplace_back(ep);

        vector<shared_ptr<SegmentLine>> lines;
        lines.reserve(n);
        for (int i = 0; i < n; i++)
            lines.emplace_back(make_shared<ArcLine>(vp[i], vp[i + 1], cp, r, anticlockwise));
        return lines;
    }

    virtual void addPoint(double x, double y) override
    {
        Point p{x, y, ON};
        if (equalPoint(p, sp))
        {
            sp = ep;
            return;
        }

        const int n = ip.size();
        int index = 0;
        double angle = anglePoint(cp, p);
        if (anticlockwise)
        {
            while (angle < 0.0)
                angle += M_PI;
        }
        else
        {
            while (angle > 0.0)
                angle -= M_PI;
        }

        while (index < n)
        {
            double cur_angle = anglePoint(cp, ip[index]);

            if (anticlockwise)
            {
                while (cur_angle < 0.0)
                    cur_angle += M_PI;
                if (cur_angle < angle)
                    index++;
                else
                    break;
            }
            else
            {
                while (cur_angle > 0.0)
                    cur_angle -= M_PI;
                if (cur_angle > angle)
                    index++;
                else
                    break;
            }
        }

        ip.insert(ip.cbegin() + index, p);
    }

    virtual bool containPoint(double x, double y) const
    {
        double distance = pow(x - cp.x, 2) + pow(y - cp.y, 2);
        if (!equal(distance, r * r))
            return false;

        double start_angle = atan2(sp.y - cp.y, sp.x - cp.x);
        double end_angle = atan2(ep.y - cp.y, ep.x - cp.x);
        double angle = atan2(y - cp.y, x - cp.x);
        if (anticlockwise)
        {
            if (start_angle < 0.0)
                start_angle += 2 * M_PI;
            while (end_angle < start_angle)
                end_angle += 2 * M_PI;

            while (angle < start_angle)
                angle += 2 * M_PI;
            return angle <= end_angle;
        }
        else
        {
            if (start_angle > 0.0)
                start_angle -= 2 * M_PI;
            while (end_angle > start_angle)
                end_angle -= 2 * M_PI;

            while (angle > start_angle)
                angle -= 2 * M_PI;
            return angle >= end_angle;
        }
    }

    Position pointPosition(Point p) const
    {
        if (SegmentLine::containPoint(p.x, p.y) || containPoint(p.x, p.y))
            return ON;

        if (distancePoint(p, cp) >= r * r)
            return OUT;

        if (anticlockwise)
        {
            if (cross(ep.x - sp.x, ep.y - sp.y, p.x - sp.x, p.y - sp.y) >= 0.0)
                return OUT;
        }
        else
        {
            if (cross(ep.x - sp.x, ep.y - sp.y, p.x - sp.x, p.y - sp.y) <= 0.0)
                return OUT;
        }

        return IN;
    }

    virtual double getArea() const
    {
        double triangle1 = SegmentLine::getArea();
        double triangle2 = 0.5 * cross(ep.y - cp.y, ep.x - cp.x, sp.y - cp.y, sp.x - cp.x);

        double start_angle = atan2(sp.y - cp.y, sp.x - cp.x);
        double end_angle = atan2(ep.y - cp.y, ep.x - cp.x);
        if (anticlockwise)
        {
            if (start_angle < 0.0)
                start_angle += 2 * M_PI;
            while (end_angle < start_angle)
                end_angle += 2 * M_PI;
        }
        else
        {
            if (start_angle > 0.0)
                start_angle -= 2 * M_PI;
            while (end_angle > start_angle)
                end_angle -= 2 * M_PI;
        }
        double span_angle = end_angle - start_angle;
        double arc_area = 0.5 * span_angle * r * r;

        if (anticlockwise)
            return triangle1 + arc_area + triangle2;
        else
            return triangle1 - arc_area - triangle2;
    }

    bool link(shared_ptr<ArcLine> next)
    {
        return equalPoint(cp, next->cp) && equal(r, next->r) && anticlockwise == next->anticlockwise;
    }
    shared_ptr<ArcLine> linkLine(shared_ptr<ArcLine> next) const
    {
        return make_shared<ArcLine>(sp, next->ep, cp, r, anticlockwise);
    }
};

vector<pair<double, double>> intersectPoint(shared_ptr<SegmentLine> left, shared_ptr<ArcLine> right)
{
    double x1 = left->sp.x;
    double y1 = left->sp.y;
    double x2 = left->ep.x;
    double y2 = left->ep.y;

    double x = right->cp.x;
    double y = right->cp.y;
    double r = right->r;

    // 向量p1p2
    double p1p2_x = x2 - x1;
    double p1p2_y = y2 - y1;
    // 单位向量p1p2
    double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
    p1p2_x /= length;
    p1p2_y /= length;

    // 向量p1p
    double p1p_x = x - x1;
    double p1p_y = y - y1;

    // b为p在直线p1p2上投影
    // p1b长度(有方向)，为p1p和p1p2的点乘
    double p1b_length = p1p_x * p1p2_x + p1p_y * p1p2_y;

    // 向量p1b, 向量p1p2缩放
    double p1b_x = p1p2_x * p1b_length;
    double p1b_y = p1p2_y * p1b_length;

    // 投影点b坐标, p1点坐标+向量p1b
    double b_x = p1b_x + p1b_x;
    double b_y = p1b_y + p1b_y;

    // 向量pb
    double pb_x = b_x - x;
    double pb_y = b_y - y;

    // 点p到直线p1p2距离，pb_length
    double pb_length = sqrt(pb_x * pb_x + pb_y * pb_y);

    vector<pair<double, double>> points;
    if (r == pb_length) // 圆与直线相切，一个交点，交点为b
    {
        // 交点b位于线段p1p2间
        // 交点b位于圆弧区间
        if ((b_x - x1) * (b_x - x2) <= 0.0 && right->containPoint(b_x, b_y))
            points.emplace_back(b_x, b_y);
    }
    else if (r > pb_length) // 圆与直线相交，两个交点
    {
        // b与交点的距离
        double b_d = sqrt(r * r - pb_length * pb_length);

        // 交点bl坐标
        double bl_x = b_x + p1p2_x * b_d;
        double bl_y = b_y + p1p2_y * b_d;
        if ((bl_x - x1) * (bl_x - x2) <= 0.0 && right->containPoint(bl_x, bl_y))
            points.emplace_back(bl_x, bl_y);

        // 交点br坐标
        double br_x = b_x - p1p2_x * b_d;
        double br_y = b_y - p1p2_y * b_d;
        if ((br_x - x1) * (br_x - x2) <= 0.0 && right->containPoint(br_x, br_y))
            points.emplace_back(br_x, br_y);
    }
    return points;
}

vector<pair<double, double>> intersectPoint(shared_ptr<ArcLine> left, shared_ptr<SegmentLine> right)
{
    return intersectPoint(right, left);
}

vector<pair<double, double>> intersectPoint(shared_ptr<ArcLine> left, shared_ptr<ArcLine> right)
{
    if (left->r > right->r)
    {
        shared_ptr<ArcLine> tmp;
        tmp = left, left = right, right = tmp;
    }

    double p1_x = left->cp.x;
    double p1_y = left->cp.y;
    double r1 = left->r;
    double p2_x = right->cp.x;
    double p2_y = right->cp.y;
    double r2 = right->r;

    double min_d = r2 - r1;
    double max_d = r1 + r2;
    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));

    vector<pair<double, double>> points;

    // r1<=r2
    if (d == max_d) // 外切
    {
        // 向量p1p2
        double p1p2_x = p2_x - p1_x;
        double p1p2_y = p2_y - p1_y;
        // 单位向量p1p2
        double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
        p1p2_x /= length;
        p1p2_y /= length;

        // 切点
        double x = p1_x + r1 * p1p2_x;
        double y = p1_y + r1 * p1p2_y;

        // 切点位于圆弧范围内
        if (left->containPoint(x, y) && right->containPoint(x, y))
            points.emplace_back(x, y);
    }
    else if (d < max_d && d > min_d) // 相交
    {
        double x1, y1;
        double x2, y2;
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

        // 切点位于圆弧范围内
        if (left->containPoint(x1, y1) && right->containPoint(x1, y1))
            points.emplace_back(x1, y1);
        if (left->containPoint(x2, y2) && right->containPoint(x2, y2))
            points.emplace_back(x2, y2);
    }
    else if (d == min_d) // 内切或重叠
    {
        if (r1 < r2)
        {
            // 向量p2p1
            double p2p1_x = p1_x - p2_x;
            double p2p1_y = p1_y - p2_y;
            // 单位向量p2p1
            double length = sqrt(p2p1_x * p2p1_x + p2p1_y * p2p1_y);
            p2p1_x /= length;
            p2p1_y /= length;

            // 切点
            double x = p1_x + r1 * p2p1_x;
            double y = p1_y + r1 * p2p1_y;

            // 切点位于圆弧范围内
            if (left->containPoint(x, y) && right->containPoint(x, y))
                points.emplace_back(x, y);
        }
        else // 重叠
        {
            double x, y;

            x = right->sp.x;
            y = right->sp.y;
            if (left->containPoint(x, y))
                points.emplace_back(x, y);

            x = right->ep.x;
            y = right->ep.y;
            if (left->containPoint(x, y))
                points.emplace_back(x, y);

            x = left->sp.x;
            y = left->sp.y;
            if (right->containPoint(x, y))
                points.emplace_back(x, y);

            x = left->ep.x;
            y = left->ep.y;
            if (right->containPoint(x, y))
                points.emplace_back(x, y);
        }
    }

    return points;
}

vector<pair<double, double>> intersectPoint(shared_ptr<SegmentLine> left, shared_ptr<SegmentLine> right)
{
    shared_ptr<ArcLine> arc_left = dynamic_pointer_cast<ArcLine>(left);
    shared_ptr<ArcLine> arc_right = dynamic_pointer_cast<ArcLine>(right);
    if (arc_left != nullptr && arc_right != nullptr)
        return intersectPoint(arc_left, arc_right);
    else if (arc_left != nullptr)
        return intersectPoint(arc_left, right);
    else if (arc_right != nullptr)
        return intersectPoint(left, arc_right);

    vector<pair<double, double>> points;

    double x1 = left->sp.x;
    double x2 = left->ep.x;
    double x3 = right->sp.x;
    double x4 = right->ep.x;
    if (!overlap(x1, x2, x3, x4)) // x轴投影不重叠
        return points;

    double y1 = left->sp.y;
    double y2 = left->ep.y;
    double y3 = right->sp.y;
    double y4 = right->ep.y;
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

struct Polygon
{
    vector<shared_ptr<SegmentLine>> lines;
    vector<shared_ptr<Polygon>> inClosePath;

    Polygon(const vector<shared_ptr<SegmentLine>> &mLines = {}) : lines(mLines) {}

    void addLine(shared_ptr<SegmentLine> line)
    {
        lines.emplace_back(line);
    }
    void addLine(vector<shared_ptr<SegmentLine>> vs)
    {
        lines.insert(lines.cend(), vs.cbegin(), vs.cend());
    }
    void addPath(shared_ptr<Polygon> poly)
    {
        inClosePath.emplace_back(poly);
    }
    bool isClosed() const
    {
        if (lines.empty())
            return false;
        return equalPoint(lines.front()->sp, lines.back()->ep);
    }
    int size() const
    {
        return lines.size();
    }

    Polygon toPoly() const
    {
        Polygon poly;
        for (const auto &line : lines)
            poly.addLine(line->toLine());
        return poly;
    }

    Position pointPosition(Point p) const
    {
        const double x = p.x;
        const double y = p.y;
        int count = 0;

        for (const auto &line : lines)
        {
            if (line->containPoint(x, y))
                return ON;

            auto arc = dynamic_pointer_cast<ArcLine>(line);
            if (arc != nullptr)
            {
                SegmentLine seg{arc->sp, arc->ep};
                if (arc->anticlockwise)
                {
                    if (seg.containPoint(x, y))
                        return IN;

                    if (arc->pointPosition(p) == IN)
                        return IN;
                }
                else
                {
                    if (seg.containPoint(x, y))
                        return OUT;

                    if (arc->pointPosition(p) == IN)
                        return OUT;
                }
            }

            count += line->crossPoint(x, y);
        }

        if (count % 2 == 1)
            return IN;
        else
            return OUT;
    }

    vector<vector<shared_ptr<SegmentLine>>> getSegmentLine(Position pos) const
    {
        const int n = size();
        vector<bool> notVisited(n, true);
        vector<vector<shared_ptr<SegmentLine>>> vvs;
        for (int i = 0; i < n; i++)
        {
            if (notVisited[i] && lines[i]->pos == pos)
            {
                int last = 0;
                while (lines[(i + last - 1 + n) % n]->pos == pos)
                    last--;
                int next = 0;
                while (lines[(i + next + 1) % n]->pos == pos)
                    next++;

                vector<shared_ptr<SegmentLine>> tmp;
                tmp.reserve(next - last + 1);
                for (int j = last; j <= next; j++)
                {
                    int index = (i + j + n) % n;
                    notVisited[index] = false;
                    tmp.emplace_back(lines[index]);
                }
                vvs.emplace_back(tmp);
            }
        }
        return vvs;
    }

    double getArea() const
    {
        double area = 0.0;
        for (const auto &line : lines)
            area += line->getArea();
        return area;
    }
};

void setLineType(Polygon &left, Polygon &right)
{
    // 起点类型
    const int n_left = left.size();
    for (int i = 0; i < n_left; i++)
    {
        if (left.lines[i]->sp.pos == UNKNOW)
        {
            int last = 0;
            while (left.lines[(i + last - 1 + n_left) % n_left]->sp.pos == UNKNOW)
                last--;
            int next = 0;
            while (left.lines[(i + next + 1) % n_left]->sp.pos == UNKNOW)
                next++;

            Position pos = right.pointPosition(left.lines[i]->sp);
            for (int j = last; j <= next; j++)
                left.lines[(i + j + n_left) % n_left]->sp.pos = pos;
        }
    }
    // 片段类型
    for (int i = 0; i < n_left; i++)
    {
        Point cur_p = left.lines[i]->sp;
        if (cur_p.pos == IN || cur_p.pos == OUT)
        {
            left.lines[i]->pos = cur_p.pos;
            continue;
        }

        Point next_p = left.lines[(i + 1) % n_left]->sp;
        if (next_p.pos == IN || next_p.pos == OUT)
        {
            left.lines[i]->pos = next_p.pos;
            continue;
        }

        Point mid_p = left.lines[i]->centerPoint();
        Position pos = right.pointPosition(mid_p);
        left.lines[i]->pos = pos;
    }

    // 起点类型
    const int n_right = right.size();
    for (int i = 0; i < n_right; i++)
    {
        if (right.lines[i]->sp.pos == UNKNOW)
        {
            int last = 0;
            while (right.lines[(i + last - 1 + n_right) % n_right]->sp.pos == UNKNOW)
                last--;
            int next = 0;
            while (right.lines[(i + next + 1) % n_right]->sp.pos == UNKNOW)
                next++;

            Position pos = left.pointPosition(right.lines[i]->sp);
            for (int j = last; j <= next; j++)
                right.lines[(i + j + n_right) % n_right]->sp.pos = pos;
        }
    }
    // 片段类型
    for (int i = 0; i < n_right; i++)
    {
        Point cur_p = right.lines[i]->sp;
        if (cur_p.pos == IN || cur_p.pos == OUT)
        {
            right.lines[i]->pos = cur_p.pos;
            continue;
        }

        Point next_p = right.lines[(i + 1) % n_right]->sp;
        if (next_p.pos == IN || next_p.pos == OUT)
        {
            right.lines[i]->pos = next_p.pos;
            continue;
        }

        Point mid_p = right.lines[i]->centerPoint();
        Position pos = left.pointPosition(mid_p);
        right.lines[i]->pos = pos;
    }
}

void boolOperation(Polygon &left, Polygon &right)
{
    const int n_left = left.lines.size();
    const int n_rigiht = right.lines.size();
    int n = 0;
    for (int i = 0; i < n_left; i++)
    {
        double x2 = left.lines[i]->ep.x;
        double y2 = left.lines[i]->ep.y;
        for (int j = 0; j < n_rigiht; j++)
        {
            double x4 = right.lines[j]->ep.x;
            double y4 = right.lines[j]->ep.y;
            for (const auto &point : intersectPoint(left.lines[i], right.lines[j]))
            {
                double x = point.first;
                double y = point.second;

                if ((equal(x2, x) && equal(y2, y)) || (equal(x4, x) && equal(y4, y)))
                    continue;

                left.lines[i]->addPoint(x, y);
                right.lines[j]->addPoint(x, y);
                n++;
            }
        }
    }

    left = left.toPoly();
    right = right.toPoly();

    setLineType(left, right);
}

vector<Polygon> cover(const vector<vector<shared_ptr<SegmentLine>>> &vs_left, const vector<vector<shared_ptr<SegmentLine>>> &vs_right)
{
    vector<Polygon> vp;

    vector<vector<shared_ptr<SegmentLine>>> vvs;
    vvs.reserve(vs_left.size() + vs_right.size());
    vvs.insert(vvs.cend(), vs_left.cbegin(), vs_right.cend());

    while (!vvs.empty())
    {
        int i = vvs.size() - 1;
        if (equalPoint(vvs[i].back()->ep, vvs[i].front()->sp))
        {
            vp.emplace_back(vvs[i]);
        }
        else
        {
            for (int j = i - 2; j >= 0; j--)
            {
                if (equalPoint(vvs[i].back()->ep, vvs[j].front()->sp))
                {
                    vvs[j].insert(vvs[j].cbegin(), vvs[i].cbegin(), vvs[i].cend());
                    break;
                }
                else if (equalPoint(vvs[i].front()->sp, vvs[j].back()->ep))
                {
                    vvs[j].insert(vvs[j].cend(), vvs[i].cbegin(), vvs[i].cend());
                    break;
                }
            }
        }
        vvs.pop_back();
    }

    return vp;
}

vector<Polygon> boolOperationIntersect(const Polygon &left, const Polygon &right)
{
    vector<vector<shared_ptr<SegmentLine>>> left_in = left.getSegmentLine(IN);
    vector<vector<shared_ptr<SegmentLine>>> right_in = right.getSegmentLine(IN);
    return cover(left_in, right_in);
}

vector<Polygon> boolOperationUnion(const Polygon &left, const Polygon &right)
{
    vector<vector<shared_ptr<SegmentLine>>> left_in = left.getSegmentLine(OUT);
    vector<vector<shared_ptr<SegmentLine>>> right_in = right.getSegmentLine(OUT);
    return cover(left_in, right_in);
}

vector<Polygon> boolOperationDifference1(const Polygon &left, const Polygon &right)
{
    vector<vector<shared_ptr<SegmentLine>>> left_in = left.getSegmentLine(OUT);
    vector<vector<shared_ptr<SegmentLine>>> right_in = right.getSegmentLine(IN);
    for (auto &vs : right_in)
    {
        for (int i = 0, j = vs.size() - 1; i < j; i++, j--)
        {
            vs[i]->reverse();
            vs[j]->reverse();
            auto tmp = vs[i];
            vs[i] = vs[j];
            vs[j] = tmp;
        }
    }
    return cover(left_in, right_in);
}

vector<Polygon> boolOperationDifference2(const Polygon &left, const Polygon &right)
{
    vector<Polygon> vp1 = boolOperationDifference1(left, right);
    vector<Polygon> vp2 = boolOperationDifference1(right, left);
    vector<Polygon> vp;
    vp.reserve(vp1.size() + vp2.size());
    vp.insert(vp.cend(), vp1.cbegin(), vp1.cend());
    vp.insert(vp.cend(), vp2.cbegin(), vp2.cend());
    return vp;
}

void linkPolygon(Polygon &poly)
{
    const int n = poly.size();
    int i = 0;
    while (i < poly.size())
    {
        auto cur = poly.lines[i];
        auto next = poly.lines[i + 1];
        if (cur->link(next))
        {
            auto line = cur->linkLine(next);
            poly.lines[i] = line;
            poly.lines.erase(poly.lines.cbegin() + i + 1);
        }
        else
        {
            i++;
        }
    }
}

int main()
{
    return 0;
}