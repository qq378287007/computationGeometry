#include <utility>
#include <vector>
#include <memory>
#include <cmath>
using namespace std;

inline double cross(double x1, double y1, double x2, double y2)
{
    return x1 * y2 - x2 * y1;
}

inline bool equal(double x1, double x2, double error = 0.00001)
{
    return abs(x1 - x2) <= error;
}

inline vector<double> intersectArea(double x1, double x2, double x3, double x4)
{
    vector<double> points;
    if (x1 > x4 || x2 < x3)
        return points;

    double left_x = max(x1, x3);
    points.push_back(left_x);

    double right_x = min(x2, x4);
    if (left_x != right_x)
        points.push_back(right_x);

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

    double left_minx = x1;
    double left_maxx = x2;
    if (left_minx > left_maxx)
    {
        left_minx = x2;
        left_maxx = x1;
    }

    double right_minx = x3;
    double right_maxx = x4;
    if (right_minx > right_maxx)
    {
        right_minx = x4;
        right_maxx = x3;
    }

    if (left_maxx < right_minx || left_minx > right_maxx)
        return points;

    double left_miny = y1;
    double left_maxy = y2;
    if (left_miny > left_maxy)
    {
        left_miny = y2;
        left_maxy = y1;
    }

    double right_miny = y3;
    double right_maxy = y4;
    if (right_miny > right_maxy)
    {
        right_miny = y4;
        right_maxy = y3;
    }

    if (left_maxy < right_miny || left_miny > right_maxy)
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

        intersectArea(x1, x2, x3, x4);

        // 4点排序
        double x0[4] = {x1, x2, x3, x4};
        double y0[4] = {y1, y2, y3, y4};
        for (int i = 0; i < 3; i++)
        {
            int min_index = i;
            for (int j = i + 1; j < 4; j++)
            {
                if (x0[j] < x0[min_index])
                    min_index = j;
            }
            if (min_index != i)
            {
                tmp = x0[i], x0[i] = x0[min_index], x0[min_index] = tmp;
                tmp = y0[i], y0[i] = y0[min_index], x0[min_index] = tmp;
            }
        }

        points.emplace_back(make_pair(x0[1], y0[1]));
        if (equal(x0[1], x0[2]) == false)
            points.emplace_back(make_pair(x0[2], y0[2]));

        return points;
    }

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
            points.emplace_back(make_pair(b_x, b_y));
    }
    else if (r > pb_length) // 圆与直线相交，两个交点
    {
        // b与交点的距离
        double b_d = sqrt(r * r - pb_length * pb_length);

        // 交点bl坐标
        double bl_x = b_x + p1p2_x * b_d;
        double bl_y = b_y + p1p2_y * b_d;
        if ((bl_x - x1) * (bl_x - x2) <= 0.0 && rayIntersectArc(bl_x, bl_y, x, y, start_angle, span_angle))
            points.emplace_back(make_pair(bl_x, bl_y));

        // 交点br坐标
        double br_x = b_x - p1p2_x * b_d;
        double br_y = b_y - p1p2_y * b_d;
        if ((br_x - x1) * (br_x - x2) <= 0.0 && rayIntersectArc(br_x, br_y, x, y, start_angle, span_angle))
            points.emplace_back(make_pair(br_x, br_y));
    }

    return points;
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
    vector<pair<double, double>> points;

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
            points.emplace_back(make_pair(x, y));
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
                points.emplace_back(make_pair(x, y));

            x = p1_x + sx * p1p2_x - h * d_p1p2_x;
            y = p1_y + sx * p1p2_y - h * d_p1p2_y;
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
                points.emplace_back(make_pair(x, y));
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
            points.emplace_back(make_pair(x, y));

            x = p1_x - sx * p1p2_x - h * d_p1p2_x;
            y = p1_y - sx * p1p2_y - h * d_p1p2_y;
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
            points.emplace_back(make_pair(x, y));
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
            points.emplace_back(make_pair(x, y));

            x = p1_x - r1 * d_p1p2_x;
            y = p1_y - r1 * d_p1p2_y;
            if (rayIntersectArc(x, y, p1_x, p1_y, start_angle1, span_angle1) &&
                rayIntersectArc(x, y, p2_x, p2_y, start_angle2, span_angle2))
            points.emplace_back(make_pair(x, y));
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
                points.emplace_back(make_pair(x, y));
            }
        }
        else // 重叠
        {
            if (span_angle1 < 0.0)
            {
                start_angle1 += span_angle1;
                span_angle1 = -span_angle1;
            }
            while (start_angle1 < 0.0)
                start_angle1 += 2 * M_PI;
            double end_angle1 = start_angle1 + span_angle1;

            if (span_angle2 < 0.0)
            {
                start_angle2 += span_angle2;
                span_angle2 = -span_angle2;
            }
            while (start_angle2 < 0.0)
                start_angle2 += 2 * M_PI;
            double end_angle2 = start_angle2 + span_angle2;

            if (end_angle1 >= start_angle2 && start_angle1 <= end_angle2)
            {
                // 4个角度排序
                double angle[4] = {start_angle1, end_angle1, start_angle2, end_angle2};
                double tmp;
                for (int i = 0; i < 3; i++)
                {
                    int min_index = i;
                    for (int j = i + 1; j < 4; j++)
                    {
                        if (angle[j] < angle[min_index])
                            min_index = j;
                    }
                    if (min_index != i)
                    {
                        tmp = angle[i], angle[i] = angle[min_index], angle[min_index] = tmp;
                    }
                }

                points.emplace_back(make_pair(r1 * cos(angle[1]), r1 * sin(angle[1])));
                if (equal(angle[1], angle[2]) == false)
                    points.emplace_back(make_pair(r1 * cos(angle[2]), r1 * sin(angle[2])));
            }
        }
    }

    return points;
}

int main()
{
    return 0;
}