#include <vector>
#include <cmath>
#include <utility>
#include <memory>
using namespace std;

enum PointPosition
{
    IN,
    ON,
    OUT,
};

enum PointType
{
    UNKNOWN,
    INNER,
    OUTER,
    ENTER_ENTER,
    ENTER_LEAVE,
    LEAVE_ENTER,
    LEAVE_LEAVE,
};

struct Point
{
    double x{0.0};
    double y{0.0};
    bool flag{false}; // 交点?
    PointType type{UNKNOWN};
};

inline double cross(double x1, double y1, double x2, double y2)
{
    return x1 * y2 - x2 * y1;
}

inline bool equalPoint(Point pa, Point pb, double error = 0.00001)
{
    return abs(pa.x - pb.x) <= error && abs(pa.y - pb.y) <= error;
}

struct Segment
{
    Point lp;
    Point sp;
    Point ep;
    vector<Point> cp;

    ~Segment() = default;

    inline int getSize() const { return 1 + cp.size(); };

    void insertPoint(Point p)
    {
        if (equalPoint(p, sp))
        {
            sp.flag = true;
        }
        else if (equalPoint(p, ep))
        {
            ep.flag = true;
        }
        else
        {
        }
    }
};

struct LineSegment : Segment
{
};

struct ArcSegment : Segment
{
    double r;
    bool direct{true};
};

vector<Point> segmentIntersect(Segment s1, Segment s2)
{
    vector<Point> p;

    return p;
}

// 线段(x1, y1, x2, y2)与线段(x3, y3, x4, y4)的交点
// 零个交点
// 一个交点
// 两个交点，表示重叠区间
inline vector<pair<double, double>> segmentsIntersect(double x1, double y1, double x2, double y2,
                                                      double x3, double y3, double x4, double y4)
{
    vector<pair<double, double>> points;

    // 检查x范围重叠
    bool overlapX = between(x3, x1, x2) || between(x4, x1, x2);

    // 检查y范围重叠
    bool overlapY = between(y3, y1, y2) || between(y4, y1, y2);

    if (!overlapX || !overlapY)
        return points;

    // 检查是否共线
    bool collinear = cross(x1 - x3, y1 - y3, x2 - x3, y2 - y3) == 0.0;

    if (collinear)
    {
        if (between(x3, x1, x2))
        {
            points.emplace_back(make_pair(x3, y3));
            if (between(x4, x1, x2))
            {
                points.emplace_back(make_pair(x4, y4));
            }
            else
            {
                if (between(x1, x3, x4))
                {
                    if (!equal(x3, y3, x1, y1))
                        points.emplace_back(make_pair(x1, y1));
                }
                else
                {
                    if (!equal(x3, y3, x2, y2))
                        points.emplace_back(make_pair(x2, y2));
                }
            }
        }
        else
        {
            if (between(x4, x1, x2))
            {
                points.emplace_back(make_pair(x4, y4));
                if (between(x1, x3, x4))
                {
                    if (!equal(x4, y4, x1, y1))
                        points.emplace_back(make_pair(x1, y1));
                }
                else
                {
                    if (!equal(x4, y4, x2, y2))
                        points.emplace_back(make_pair(x2, y2));
                }
            }
            else
            {
                points.emplace_back(make_pair(x1, y1));
                points.emplace_back(make_pair(x2, y2));
            }
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
            points.emplace_back(make_pair(x, y));
        }
    }

    return points;
}

bool betweenArc(double angle, double start_angle, double span_angle)
{
    if (span_angle > 0.0)
    {
        while (angle < start_angle)
            angle += 2 * M_PI;
        if (angle <= start_angle + span_angle)
            return true;
    }
    else if (span_angle < 0.0)
    {
        while (angle > start_angle)
            angle -= 2 * M_PI;
        if (angle >= start_angle + span_angle)
            return true;
    }
    return false;
}

bool rayIntersectArc(double x, double y, double p_x, double p_y, double r, double start_angle, double span_angle)
{
    double angle = atan2(y - p_y, x - p_x);
    if (span_angle > 0.0)
    {
        while (angle < start_angle)
            angle += 2 * M_PI;
        if (angle <= start_angle + span_angle)
            return true;
    }
    else if (span_angle < 0.0)
    {
        while (angle > start_angle)
            angle -= 2 * M_PI;
        if (angle >= start_angle + span_angle)
            return true;
    }
    return false;
}

// 线段上两点p1(p1_x, p1_y)与p2(p2_x, p2_y)
// 圆心p(p_x, p_y), 半径r, 起始角start_angle, 跨越角span_angle
// 线段与圆弧交点
vector<pair<double, double>> segmentIntersectArc(double p1_x, double p1_y, double p2_x, double p2_y,
                                                 double p_x, double p_y, double r, double start_angle, double span_angle)
{
    vector<pair<double, double>> points;

    // 向量p1p2
    double p1p2_x = p2_x - p1_x;
    double p1p2_y = p2_y - p1_y;

    // 单位向量p1p2
    double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
    p1p2_x /= length;
    p1p2_y /= length;

    // 向量p1p
    double p1p_x = p_x - p1_x;
    double p1p_y = p_y - p1_y;

    // b为p在直线p1p2上投影
    // p1b长度(有方向)，为p1p和p1p2的点乘
    double p1b = p1p_x * p1p2_x + p1p_y * p1p2_y;

    // 向量p1b,向量p1p2缩放
    double p1b_x = p1p2_x * p1b;
    double p1b_y = p1p2_y * p1b;

    // 投影点b坐标,p1点坐标+向量p1b
    double b_x = p1b_x + p1b_x;
    double b_y = p1b_y + p1b_y;

    // 向量pb
    double pb_x = p_x - b_x;
    double pb_y = p_y - b_y;

    // 点p到直线p1p2距离，pb
    double pb = sqrt(pb_x * pb_x + pb_y * pb_y);

    // double end_angle = start_angle + span_angle;
    if (r == pb) // 圆与直线相切，一个交点，交点为b
    {
        if (between(b_x, p1_x, p2_x)) // 交点b位于线段p1p2间
        {
            // pb与x轴的夹角
            double angle = atan2(pb_y, pb_x);
            if (span_angle > 0.0)
            {
                while (angle < start_angle)
                    angle += 2 * M_PI;

                if (angle <= start_angle + span_angle)
                    points.emplace_back(make_pair(b_x, b_y));
            }
            else if (span_angle < 0.0)
            {
                while (angle > start_angle)
                    angle -= 2 * M_PI;

                if (angle >= start_angle + span_angle)
                    points.emplace_back(make_pair(b_x, b_y));
            }
        }
    }
    else if (r > pb) // 圆与直线相交，两个交点
    {
        // b与交点的距离
        double b_d = sqrt(r * r - pb * pb);

        // 交点bl坐标
        double bl_x = b_x + p1p2_x * b_d;
        double bl_y = b_y + p1p2_y * b_d;
        if (between(bl_x, p1_x, p2_x)) // 交点bl位于线段p1p2间
        {
            // pbl与x轴的夹角
            double angle = atan2(bl_y - p_y, bl_x - p_x);
            if (span_angle > 0.0)
            {
                while (angle < start_angle)
                    angle += 2 * M_PI;

                if (angle <= start_angle + span_angle)
                    points.emplace_back(make_pair(bl_x, bl_y));
            }
            else if (span_angle < 0.0)
            {
                while (angle > start_angle)
                    angle -= 2 * M_PI;

                if (angle >= start_angle + span_angle)
                    points.emplace_back(make_pair(bl_x, bl_y));
            }
        }

        // 交点br坐标
        double br_x = b_x - p1p2_x * b_d;
        double br_y = b_y - p1p2_y * b_d;
        if (between(bl_x, p1_x, p2_x)) // 交点br位于线段p1p2间
        {
            // pbr与x轴的夹角
            double angle = atan2(br_y - p_y, br_x - p_x);
            if (span_angle > 0.0)
            {
                while (angle < start_angle)
                    angle += 2 * M_PI;

                if (angle <= start_angle + span_angle)
                    points.emplace_back(make_pair(br_x, br_y));
            }
            else if (span_angle < 0.0)
            {
                while (angle > start_angle)
                    angle -= 2 * M_PI;

                if (angle >= start_angle + span_angle)
                    points.emplace_back(make_pair(br_x, br_y));
            }
        }
    }

    return points;
}

// 圆心p1(p1_x, p1_y), 半径r1, 起始角start_angle1, 跨越角span_angle1
// 圆心p2(p2_x, p2_y), 半径r2, 起始角start_angle2, 跨越角span_angle2
// 圆弧与圆弧交点
// 零个交点
// 一个交点
// 两个交点，表示相交或重叠区间
vector<pair<double, double>> segmentIntersectArc(double p1_x, double p1_y, double r1, double start_angle1, double span_angle1,
                                                 double p2_x, double p2_y, double r2, double start_angle2, double span_angle2)
{
    vector<pair<double, double>> points;

    double min_d = abs(r1 - r2);
    double max_d = r1 + r2;

    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));
    if (d == max_d) // 外切
    {
        if (rayIntersectArc(p2_x, p2_y, p1_x, p1_y, r1, start_angle1, span_angle1) &&
            rayIntersectArc(p1_x, p1_y, p2_x, p2_y, r2, start_angle2, span_angle2))
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
            points.emplace_back(make_pair(x, y));
        }
    }
    else if (d < max_d && d > min_d) // 相交
    {
        // r1<r2
        double sr = sqrt(r2 * r2 - r1 * r1);
        if (d < sr)
        {
            double h = sqrt((r2 * r2 + r1 * r1 - d) / 2);
            double sx = sqrt(r1 * r1 - h * h);

            // 向量p1p2
            double p1p2_x = p2_x - p1_x;
            double p1p2_y = p2_y - p1_y;
            // 单位向量p1p2
            double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
            p1p2_x /= length;
            p1p2_y /= length;

            // 垂直向量
            double d_p1p2_x = p1p2_y;
            double d_p1p2_y = -p1p2_x;

            // 交点
            double x = p1_x + sx * p1p2_x + h * d_p1p2_x;
            double y = p1_y + sx * p1p2_y + h * d_p1p2_y;

            x = p1_x + sx * p1p2_x - h * d_p1p2_x;
            y = p1_y + sx * p1p2_y - h * d_p1p2_y;
        }
        else if (d > sr)
        {
            double sx = (sr * sr - d * d) / (2 * d);
            double h = sqrt(r1 * r1 - sx * sx);

            // 向量p1p2
            double p1p2_x = p2_x - p1_x;
            double p1p2_y = p2_y - p1_y;
            // 单位向量p1p2
            double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
            p1p2_x /= length;
            p1p2_y /= length;

            // 垂直向量
            double d_p1p2_x = p1p2_y;
            double d_p1p2_y = -p1p2_x;

            // 交点
            double x = p1_x - sx * p1p2_x + h * d_p1p2_x;
            double y = p1_y - sx * p1p2_y + h * d_p1p2_y;

            x = p1_x - sx * p1p2_x - h * d_p1p2_x;
            y = p1_y - sx * p1p2_y - h * d_p1p2_y;
        }
        else
        {
            // 向量p1p2
            double p1p2_x = p2_x - p1_x;
            double p1p2_y = p2_y - p1_y;
            // 单位向量p1p2
            double length = sqrt(p1p2_x * p1p2_x + p1p2_y * p1p2_y);
            p1p2_x /= length;
            p1p2_y /= length;

            // 垂直向量
            double d_p1p2_x = p1p2_y;
            double d_p1p2_y = -p1p2_x;

            // 交点
            double x = p1_x + r1 * d_p1p2_x;
            double y = p1_y + r1 * d_p1p2_y;

            x = p1_x - r1 * d_p1p2_x;
            y = p1_y - r1 * d_p1p2_y;
        }
    }
    else if (d == min_d) // 内切
    {
        if (r2 > r1)
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
            points.emplace_back(make_pair(x, y));
        }
        else if (r2 < r1)
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
            points.emplace_back(make_pair(x, y));
        }
        else
        {
            if (betweenArc(start_angle1, start_angle2, span_angle2) ||
                betweenArc(start_angle1 + span_angle1, start_angle2, span_angle2))
            {
                if (betweenArc(start_angle1, start_angle2, span_angle2) &&
                    betweenArc(start_angle1 + span_angle1, start_angle2, span_angle2))
                {
                    points.emplace_back(make_pair(r1 * cos(start_angle1), r1 * sin(start_angle1)));
                    points.emplace_back(make_pair(r1 * cos(start_angle1 + span_angle1), r1 * sin(start_angle1 + span_angle1)));
                }
                else if (betweenArc(start_angle2, start_angle1, span_angle1) &&
                         betweenArc(start_angle2 + span_angle2, start_angle1, span_angle1))
                {
                    points.emplace_back(make_pair(r1 * cos(start_angle2), r1 * sin(start_angle2)));
                    points.emplace_back(make_pair(r1 * cos(start_angle2 + span_angle2), r1 * sin(start_angle2 + span_angle2)));
                }
                else if (betweenArc(start_angle1, start_angle2, span_angle2))
                {
                    points.emplace_back(make_pair(r1 * cos(start_angle1), r1 * sin(start_angle1)));
                    if (betweenArc(start_angle2, start_angle1, span_angle1))
                        points.emplace_back(make_pair(r1 * cos(start_angle2), r1 * sin(start_angle2)));
                    else
                        points.emplace_back(make_pair(r1 * cos(start_angle2 + span_angle2), r1 * sin(start_angle2 + span_angle2)));
                }
                else if (betweenArc(start_angle1 + span_angle1, start_angle2, span_angle2))
                {
                    points.emplace_back(make_pair(r1 * cos(start_angle1 + span_angle1), r1 * sin(start_angle1 + span_angle1)));
                    if (betweenArc(start_angle2, start_angle1, span_angle1))
                        points.emplace_back(make_pair(r1 * cos(start_angle2), r1 * sin(start_angle2)));
                    else
                        points.emplace_back(make_pair(r1 * cos(start_angle2 + span_angle2), r1 * sin(start_angle2 + span_angle2)));
                }
            }
        }
    }

    return points;
}


bool pointInPolygon(Point p, const vector<Segment> &vs)
{
    double x = p.x;
    double y = p.y;

    double x1, y1, x2, y2;
    double tmp;
    int count = 0;
    const int n = vs.size();
    for (int i = 0; i < n; i++)
    {
        x1 = vs[i].sp.x;
        y1 = vs[i].sp.y;
        x2 = vs[i].ep.x;
        y2 = vs[i].ep.y;

        if (vs[i].containPoint(p))
            return true;

        if (y1 != y2 && (y - y1) * (y - y2) <= 0.0)
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

// 判断两条线段是否交叉（不含端点）
bool isIntersection(const Segment &left, const Segment &right)
{
    double x1 = left.sp.x;
    double y1 = left.sp.y;
    double x2 = left.ep.x;
    double y2 = left.ep.y;

    double x3 = right.sp.x;
    double y3 = right.sp.y;
    double x4 = right.ep.x;
    double y4 = right.ep.y;

    if ((x3 - x1) * (x3 - x2) >= 0.0 && (x4 - x1) * (x4 - x2) >= 0.0)
        return false;

    if ((y3 - y1) * (y3 - y2) >= 0.0 && (y4 - y1) * (y4 - y2) >= 0.0)
        return false;

    // 向量叉乘，跨立试验
    double p12xp13 = cross(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
    double p12xp14 = cross(x2 - x1, y2 - y1, x4 - x1, y4 - y1);
    if (p12xp13 * p12xp14 >= 0.0)
        return false;

    double p34xp32 = cross(x4 - x3, y4 - y3, x2 - x3, y2 - y3);
    double p34xp31 = cross(x4 - x3, y4 - y3, x1 - x3, y1 - y3);
    if (p34xp32 * p34xp31 >= 0.0)
        return false;

    return false;
}

// 两条交叉线段的交点
Point intersectionPoint(const Segment &left, const Segment &right)
{
    double x1 = left.sp.x;
    double y1 = left.sp.y;
    double x2 = left.ep.x;
    double y2 = left.ep.y;

    double x3 = right.sp.x;
    double y3 = right.sp.y;
    double x4 = right.ep.x;
    double y4 = right.ep.y;

    double p13_x = x3 - x1;
    double p13_y = y3 - y1;

    double p12_x = x2 - x1;
    double p12_y = y2 - y1;

    double p34_x = x4 - x3;
    double p34_y = y4 - y3;

    double p13xp34 = p13_x * p34_y - p34_x * p13_y;
    double p12xp34 = p12_x * p34_y - p34_x * p12_y;
    double s = p13xp34 / p12xp34;

    double x = x1 + s * p12_x;
    double y = y1 + s * p12_y;
    return Point(x, y);
}

int main()
{
    return 0;
}