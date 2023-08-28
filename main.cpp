#include <utility>
#include <vector>
#include <memory>
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

// 区域[x1, x2]与[x3, x4]是否有交集
inline bool isOverlap(double x1, double x2, double x3, double x4)
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

// 重叠区域
inline vector<pair<double, double>> intersectArea(double x1, double x2, double x3, double x4)
{
    vector<pair<double, double>> points;
    if (isOverlap(x1, x2, x3, x4))
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

        double left_x = max(x1, x3);
        double right_x = min(x2, x4);
        points.emplace_back(left_x, right_x);
    }
    return points;
}

// 线段(x1, y1, x2, y2)与线段(x3, y3, x4, y4)的交点
// 零个交点
// 一个交点
// 两个交点，表示重叠区间
inline vector<pair<double, double>> segmentsIntersect(double x1, double y1, double x2, double y2,
                                                      double x3, double y3, double x4, double y4)
{
    vector<pair<double, double>> points;
    if (!isOverlap(x1, x2, x3, x4) || !isOverlap(y1, y2, y3, y4))
        return points;

    if (cross(x1 - x3, y1 - y3, x2 - x3, y2 - y3) == 0.0) // 共线
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
        int right_ind = x2 < x4 ? 1 : 3;

        points.emplace_back(x0[left_ind], y0[left_ind]);
        if (equal(x0[left_ind], x0[right_ind]) == false)
            points.emplace_back(x0[right_ind], y0[right_ind]);
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

// 射线与圆弧是否相交
bool rayIntersectArc(double x, double y,
                     double p_x, double p_y, double start_angle, double span_angle)
{
    double angle = atan2(y - p_y, x - p_x);
    if (span_angle > 0.0)
    {
        while (angle > start_angle)
            angle -= 2 * M_PI;
        while (angle < start_angle)
            angle += 2 * M_PI;
        if (angle <= start_angle + span_angle)
            return true;
    }
    else if (span_angle < 0.0)
    {
        while (angle < start_angle)
            angle += 2 * M_PI;
        while (angle > start_angle)
            angle -= 2 * M_PI;
        if (angle >= start_angle + span_angle)
            return true;
    }
    return false;
}

// 线段上两点p1(x1, y1)，p2(x2, y2)
// 圆心p(x, y), 半径r, 起始角start_angle, 跨越角span_angle
// 线段与圆弧交点
vector<pair<double, double>> segmentIntersectArc(double x1, double y1, double x2, double y2,
                                                 double x, double y, double r, double start_angle, double span_angle)
{
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
        if ((b_x - x1) * (b_x - x2) <= 0.0 && rayIntersectArc(b_x, b_y, x, y, start_angle, span_angle))
            points.emplace_back(b_x, b_y);
    }
    else if (r > pb_length) // 圆与直线相交，两个交点
    {
        // b与交点的距离
        double b_d = sqrt(r * r - pb_length * pb_length);

        // 交点bl坐标
        double bl_x = b_x + p1p2_x * b_d;
        double bl_y = b_y + p1p2_y * b_d;
        if ((bl_x - x1) * (bl_x - x2) <= 0.0 && rayIntersectArc(bl_x, bl_y, x, y, start_angle, span_angle))
            points.emplace_back(bl_x, bl_y);

        // 交点br坐标
        double br_x = b_x - p1p2_x * b_d;
        double br_y = b_y - p1p2_y * b_d;
        if ((br_x - x1) * (br_x - x2) <= 0.0 && rayIntersectArc(br_x, br_y, x, y, start_angle, span_angle))
            points.emplace_back(br_x, br_y);
    }

    return points;
}

// 重叠区域
inline vector<pair<double, double>> intersectArc(double start_angle1, double span_angle1, double start_angle2, double span_angle2)
{
    if (span_angle1 < 0.0)
    {
        start_angle1 += span_angle1;
        span_angle1 = -span_angle1;
    }
    while (start_angle1 < 0.0)
        start_angle1 += 2 * M_PI;
    while (start_angle1 > 2 * M_PI)
        start_angle1 -= 2 * M_PI;
    double end_angle1 = start_angle1 + span_angle1;

    if (span_angle2 < 0.0)
    {
        start_angle2 += span_angle2;
        span_angle2 = -span_angle2;
    }
    while (start_angle2 < 0.0)
        start_angle2 += 2 * M_PI;
    while (start_angle2 > 2 * M_PI)
        start_angle2 -= 2 * M_PI;
    double end_angle2 = start_angle2 + span_angle2;

    vector<pair<double, double>> arcs;
    if ((end_angle1 <= 2 * M_PI && end_angle2 <= 2 * M_PI) || (end_angle1 >= 2 * M_PI && end_angle2 >= 2 * M_PI))
    {
        auto angles = intersectArea(start_angle1, end_angle1, start_angle2, end_angle2);
        arcs.insert(arcs.cend(), angles.cbegin(), angles.cend());
    }
    else if (end_angle1 < 2 * M_PI && end_angle2 > 2 * M_PI)
    {
        auto angles = intersectArea(start_angle1, end_angle1, 0.0, end_angle2 - 2 * M_PI);
        arcs.insert(arcs.cend(), angles.cbegin(), angles.cend());

        auto angles2 = intersectArea(start_angle1, end_angle1, start_angle2, 2 * M_PI);
        arcs.insert(arcs.cend(), angles2.cbegin(), angles2.cend());
    }
    else if (end_angle2 < 2 * M_PI && end_angle1 > 2 * M_PI)
    {
        auto angles = intersectArea(start_angle2, end_angle2, 0.0, end_angle1 - 2 * M_PI);
        arcs.insert(arcs.cend(), angles.cbegin(), angles.cend());

        auto angles2 = intersectArea(start_angle2, end_angle2, start_angle1, 2 * M_PI);
        arcs.insert(arcs.cend(), angles2.cbegin(), angles2.cend());
    }
    return arcs;
}

// 圆心p1(p1_x, p1_y), 半径r1, 起始角start_angle1, 跨越角span_angle1
// 圆心p2(p2_x, p2_y), 半径r2, 起始角start_angle2, 跨越角span_angle2
// 圆弧与圆弧交点
// 零个交点
// 一个交点
// 两个交点，表示相交或重叠区间
vector<pair<double, double>> intersectArc(double p1_x, double p1_y, double r1, double start_angle1, double span_angle1,
                                          double p2_x, double p2_y, double r2, double start_angle2, double span_angle2)
{
    double tmp;
    if (r1 > r2)
    {
        tmp = p1_x, p1_x = p2_x, p2_x = tmp;
        tmp = p1_y, p1_y = p2_y, p2_y = tmp;
        tmp = r1, r1 = r2, r2 = tmp;
        tmp = start_angle1, start_angle1 = start_angle2, start_angle2 = tmp;
        tmp = span_angle1, span_angle1 = span_angle2, span_angle2 = tmp;
    }

    double min_d = r2 - r1;
    double max_d = r1 + r2;
    double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));

    vector<pair<double, double>> points;
    if (d == max_d) // 外切
    {
        // 圆心指向交点的射线位于圆弧范围内
        if (rayIntersectArc(p2_x, p2_y, p1_x, p1_y, start_angle1, span_angle1) &&
            rayIntersectArc(p1_x, p1_y, p2_x, p2_y, start_angle2, span_angle2))
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
            points.emplace_back(x, y);
        }
    }
    else if (d < max_d && d > min_d) // 相交
    {
        // r1<r2
        double sr = sqrt(r2 * r2 - r1 * r1);
        if (d < sr) // 圆心靠近
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
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
                points.emplace_back(x, y);

            x = p1_x + sx * p1p2_x - h * d_p1p2_x;
            y = p1_y + sx * p1p2_y - h * d_p1p2_y;
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
                points.emplace_back(x, y);
        }
        else if (d > sr) // 圆心较远
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
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
                points.emplace_back(x, y);

            x = p1_x - sx * p1p2_x - h * d_p1p2_x;
            y = p1_y - sx * p1p2_y - h * d_p1p2_y;
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
                points.emplace_back(x, y);
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

            // 垂直向量
            double d_p1p2_x = p1p2_y;
            double d_p1p2_y = -p1p2_x;

            // 交点
            double x = p1_x + r1 * d_p1p2_x;
            double y = p1_y + r1 * d_p1p2_y;
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
                points.emplace_back(x, y);

            x = p1_x - r1 * d_p1p2_x;
            y = p1_y - r1 * d_p1p2_y;
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
                points.emplace_back(x, y);
        }
    }
    else if (d == min_d) // 内切
    {
        if (r1 < r2)
        {
            // 圆心指向交点的射线位于圆弧范围内, r2关于r1的对称点
            if (rayIntersectArc(p1_x * 2 - p2_x, p1_y * 2 - p2_y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(p1_x, p1_y, p2_x, p2_y, start_angle2, span_angle2))
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
                points.emplace_back(x, y);
            }
        }
        else // 重叠
        {
            for (const auto &angle : intersectArc(start_angle1, span_angle1, start_angle2, span_angle2))
            {
                points.emplace_back(r1 * cos(angle.first), r1 * sin(angle.first));
                if (equal(angle.first, angle.second) == false)
                    points.emplace_back(r1 * cos(angle.second), r1 * sin(angle.second));
            }
        }
    }

    return points;
}



//点是否位于封闭区域内
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


int main()
{
    return 0;
}