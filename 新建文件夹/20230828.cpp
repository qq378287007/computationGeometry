#include <vector>
#include <cmath>
#include <utility>
#include <memory>
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

// 射线与圆弧是否相交
bool rayIntersectArc(double x, double y, double p_x, double p_y, double start_angle, double span_angle)
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

enum PointPosition
{
    UNKNOW,
    IN,
    ON,
    OUT,
};

struct Point
{
    double x{0.0};
    double y{0.0};
    bool flag{false};
    PointPosition pp{UNKNOW};
};

struct Segment
{
    // Point lp;
    Point sp;
    Point ep;
    vector<Point> ip;

    ~Segment() = default;

    virtual bool containPoint(Point p);

    /*
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
        */
};

struct LineSegment : Segment
{
    bool containPoint(Point p) override
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
};

struct ArcSegment : Segment
{
    Point cp;
    double r;
    double start_angle;
    double span_angle;

    bool containPoint(Point p) override
    {
        return pow(p.x - cp.x, 2) + pow(p.y - cp.y, 2) == r * r &&
               rayIntersectArc(p.x, p.y, cp.x, cp.y, start_angle, span_angle);
    }
};


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

       // if (vs[i].containPoint(p))
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