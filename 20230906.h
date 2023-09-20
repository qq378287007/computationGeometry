#pragma once

#include <cmath>
#include <memory>
#include <vector>
using namespace std;

static double gError = 1.0e-12;

// 相等
inline bool equal(double x1, double x2)
{
    return abs(x1 - x2) <= gError;
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

    inline bool equalP(double px, double py) const
    {
        return equal(x, px) && equal(y, py);
    }

    inline bool equalPoint(Point p) const
    {
        return equalP(p.x, p.y);
    }

    inline double distance(Point p) const
    {
        return pow(x - p.x, 2) + pow(y - p.y, 2);
    }

    // p点到当前点连线矢量，方向
    //(-pi, pi]
    inline double angle(Point p) const
    {
        return atan2(y - p.y, x - p.x);
    }

    inline void swap(Point &p)
    {
        double tmp;
        tmp = x, x = p.x, p.x = tmp;
        tmp = y, y = p.y, p.y = tmp;
        Position pos_tmp;
        pos_tmp = pos, pos = p.pos, p.pos = pos_tmp;
    }
};

struct Path;
struct Line
{
    Point sp;
    Point ep;
    Position pos;
    vector<Point> ip;
    Line(Point mSp = {}, Point mEp = {}, Position mPos = UNKNOW)
        : sp(mSp), ep(mEp), pos(mPos) {}
    virtual ~Line() = default;

    virtual void reverse()
    {
        sp.swap(ep);
        for (int i = 0, j = ip.size() - 1; i < j; i++, j--)
            ip[i].swap(ip[j]);
    }

    virtual Point centerPoint() const
    {
        return Point((sp.x + ep.x) * 0.5, (sp.y + ep.y) * 0.5);
    }

    virtual bool equalLine(shared_ptr<Line> line) const;
    virtual void intersect(shared_ptr<Line> line);
    virtual shared_ptr<Path> toPath() const;

    virtual void addPoint(double x, double y)
    {
        if (ep.equalP(x, y))
            return;

        if (sp.equalP(x, y))
        {
            sp.pos = ON;
            return;
        }

        Point p{x, y, ON};
        const int n = ip.size();
        const double distance = sp.distance(p);
        int index = 0;
        while (index < n && sp.distance(ip[index]) < distance)
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

    virtual shared_ptr<Line> linkLine(shared_ptr<Line> next) const;
};

struct Arc : Line
{
    Point cp;
    bool anticlockwise;
    double r;
    Arc(Point mSp = {}, Point mEp = {}, Point mCp = {}, bool mAnticlockwise = true, Position mPos = UNKNOW)
        : Line(mSp, mEp, mPos), cp(mCp), anticlockwise(mAnticlockwise)
    {
        r = sqrt(cp.distance(sp));
    }

    virtual void reverse() override
    {
        Line::reverse();
        anticlockwise = !anticlockwise;
    }

    virtual Point centerPoint() const override
    {
        double start_angle = cp.angle(sp);
        double end_angle = cp.angle(ep);

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

    virtual bool equalLine(shared_ptr<Line> line) const override;
    virtual void intersect(shared_ptr<Line> line) override;
    virtual shared_ptr<Path> toPath() const override;

    double absAngleToSp(Point p) const
    {
        double start_angle = cp.angle(sp);
        double angle = cp.angle(p);
        if (anticlockwise)
        {
            while (angle > start_angle)
                angle -= 2 * M_PI;
            while (angle < start_angle)
                angle += 2 * M_PI;
        }
        else
        {
            while (angle < start_angle)
                angle += 2 * M_PI;
            while (angle > start_angle)
                angle -= 2 * M_PI;
        }
        return abs(angle - start_angle);
    }

    virtual void addPoint(double x, double y) override
    {
        if (ep.equalP(x, y))
            return;

        if (sp.equalP(x, y))
        {
            sp.pos = ON;
            return;
        }

        Point p{x, y, ON};
        const int n = ip.size();
        double angle = absAngleToSp(p);
        int index = 0;
        while (index < n && absAngleToSp(ip[index]) < angle)
            index++;

        ip.insert(ip.cbegin() + index, p);
    }

    virtual bool containPoint(double x, double y) const override
    {
        double distance = pow(x - cp.x, 2) + pow(y - cp.y, 2);
        if (!equal(distance, r * r))
            return false;

        double start_angle = cp.angle(sp);
        double end_angle = cp.angle(ep);
        double angle = cp.angle({x, y});
        if (anticlockwise)
        {
            while (end_angle > start_angle)
                end_angle -= 2 * M_PI;
            while (end_angle < start_angle)
                end_angle += 2 * M_PI;

            while (angle > start_angle)
                angle -= 2 * M_PI;
            while (angle < start_angle)
                angle += 2 * M_PI;
            return angle <= end_angle;
        }
        else
        {
            while (end_angle < start_angle)
                end_angle += 2 * M_PI;
            while (end_angle > start_angle)
                end_angle -= 2 * M_PI;

            while (angle < start_angle)
                angle += 2 * M_PI;
            while (angle > start_angle)
                angle -= 2 * M_PI;
            return angle >= end_angle;
        }
    }

    // 点与弓形的位置关系
    Position pointPosition(Point p) const
    {
        if (Line::containPoint(p.x, p.y) || containPoint(p.x, p.y))
            return ON;

        if (cp.distance(p) >= r * r)
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

    // 类扇形面积，含正负
    virtual double getArea() const
    {
        double triangle1 = Line::getArea();

        double span_angle = absAngleToSp(ep);
        if (!anticlockwise)
        {
            span_angle = -span_angle;
        }
        double arc_area = 0.5 * span_angle * r * r;

        double triangle2 = 0.5 * cross(ep.y - cp.y, ep.x - cp.x, sp.y - cp.y, sp.x - cp.x);

        return triangle1 + arc_area + triangle2;
    }

    virtual shared_ptr<Line> linkLine(shared_ptr<Line> next) const override;
};

struct Path
{
    vector<shared_ptr<Line>> lines;
    vector<shared_ptr<Path>> inPath;

    Path() = default;
    Path(shared_ptr<Line> line)
    {
        lines.emplace_back(line);
    }
    Path(shared_ptr<Path> path)
    {
        lines.insert(lines.cend(), path->lines.cbegin(), path->lines.cend());
    }

    int size() const
    {
        return lines.size();
    }

    Point start() const
    {
        return lines.front()->sp;
    }

    Point end() const
    {
        return lines.back()->sp;
    }

    bool close() const
    {
        return start().equalPoint(end());
    }

    void reverse()
    {
        for (int i = 0, j = size() - 1; i < j; i++, j--)
        {
            lines[i]->reverse();
            lines[j]->reverse();
            shared_ptr<Line> tmp;
            tmp = lines[i], lines[i] = lines[j], lines[j] = tmp;
        }
    }

    bool addLine(shared_ptr<Line> line)
    {
        if (line->sp.equalPoint(end()))
        {
            lines.emplace_back(line);
            return true;
        }

        if (line->ep.equalPoint(end()))
        {
            line->reverse();
            lines.emplace_back(line);
            return true;
        }

        if (line->ep.equalPoint(start()))
        {
            lines.insert(lines.cbegin(), line);
            return true;
        }

        if (line->sp.equalPoint(start()))
        {
            line->reverse();
            lines.insert(lines.cbegin(), line);
            return true;
        }

        return false;
    }

    bool addPath(shared_ptr<Path> path)
    {
        if (path->start().equalPoint(end()))
        {
            lines.insert(lines.cend(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        if (path->end().equalPoint(end()))
        {
            path->reverse();
            lines.insert(lines.cend(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        if (path->end().equalPoint(start()))
        {
            lines.insert(lines.cbegin(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        if (path->start().equalPoint(start()))
        {
            path->reverse();
            lines.insert(lines.cbegin(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        return false;
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

            auto arc = dynamic_pointer_cast<Arc>(line);
            if (arc != nullptr)
            {
                Line seg{arc->sp, arc->ep};
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

    double getArea() const
    {
        double area = 0.0;
        for (const auto &line : lines)
            area += line->getArea();
        return area;
    }

    vector<shared_ptr<Path>> getVectorPath(Position pos) const
    {
        const int n = size();
        vector<bool> notVisited(n, true);
        vector<shared_ptr<Path>> vp;
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

                auto path = make_shared<Path>();
                path->lines.reserve(next - last + 1);
                for (int j = last; j <= next; j++)
                {
                    int index = (i + j + n) % n;
                    notVisited[index] = false;
                    path->addLine(lines[index]);
                }
                vp.emplace_back(path);
            }
        }
        return vp;
    }
    shared_ptr<Path> toPath() const;

    shared_ptr<Path> toLinkPath() const;
};
