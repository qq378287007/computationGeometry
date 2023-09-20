
#include <cmath>
#include <memory>
#include <vector>
using namespace std;

static double gError = 1.0e-6;

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

    bool equalP(Point p) const
    {
        return equal(x, p.x) && equal(y, p.y);
    }

    double distance(Point p) const
    {
        return pow(x - p.x, 2) + pow(y - p.y, 2);
    }

    // p点到当前点连线矢量，方向
    //(-pi, pi]
    double angle(Point p) const
    {
        return atan2(y - p.y, x - p.x);
    }

    void swap(Point &p)
    {
        double tmp;
        tmp = x, x = p.x, p.x = tmp;
        tmp = y, y = p.y, p.y = tmp;
        Position pos_tmp;
        pos_tmp = pos, pos = p.pos, p.pos = pos_tmp;
    }
};

bool equalPoint(Point p1, Point p2)
{
    return equal(p1.x, p2.x) && equal(p1.y, p2.y);
}

struct Arc;
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
        Point tmp;
        tmp = sp, sp = ep, ep = tmp;
    }

    virtual bool equalLine(shared_ptr<Line> line) const
    {
        shared_ptr<Arc> arc = dynamic_pointer_cast<Arc>(line);
        if (arc)
            return false;
        return (equalPoint(sp, line->sp) && equalPoint(ep, line->ep)) ||
               (equalPoint(sp, line->ep) && equalPoint(ep, line->sp));
    }

    virtual void intersect(shared_ptr<Line> line)
    {
        shared_ptr<Arc> arc = dynamic_pointer_cast<Arc>(line);
        if (arc == nullptr)
        {
            double x1 = sp.x;
            double x2 = ep.x;
            double x3 = line->sp.x;
            double x4 = line->ep.x;
            if (!overlap(x1, x2, x3, x4)) // x轴投影不重叠
                return;

            double y1 = sp.y;
            double y2 = ep.y;
            double y3 = line->sp.y;
            double y4 = line->ep.y;
            if (!overlap(y1, y2, y3, y4)) // y轴投影不重叠
                return;
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
                    // points.emplace_back(x0[left_ind], y0[left_ind]);

                    int right_ind = x2 < x4 ? 1 : 3;
                    // if (!equal(x0[left_ind], x0[right_ind]))
                    //     points.emplace_back(x0[right_ind], y0[right_ind]);
                }
            }
            else // 直线相交
            {
                // 向量叉乘，跨立试验
                double p12xp13 = cross(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
                double p12xp14 = cross(x2 - x1, y2 - y1, x4 - x1, y4 - y1);
                if (p12xp13 * p12xp14 > 0.0) // (x3, y3), (x4, y4)位于[(x1, y1), (x2, y2)]同侧
                    return;

                double p34xp32 = cross(x4 - x3, y4 - y3, x2 - x3, y2 - y3);
                double p34xp31 = cross(x4 - x3, y4 - y3, x1 - x3, y1 - y3);

                if (p34xp32 * p34xp31 > 0.0) //(x1, y1), (x2, y2)位于[(x3, y3), (x4, y4)]同侧
                    return;

                // 线段相交
                double s = cross(x1 - x3, y1 - y3, x2 - x1, y2 - y1) / cross(x4 - x3, y4 - y3, x2 - x1, y2 - y1);
                double x = x1 + s * (x2 - x1);
                double y = y1 + s * (y2 - y1);
                // vp.emplace_back(x, y);
            }
        }
        else
        {
        }
    }

    virtual shared_ptr<Path> toPath() const
    {
        const int n = ip.size();
        vector<Point> vp;
        vp.reserve(n + 1);
        vp.emplace_back(sp);
        vp.insert(vp.cend(), ip.cbegin(), ip.cend());
        vp.emplace_back(ep);

        auto path = make_shared<Path>();
        for (int i = 0; i < n; i++)
            path->addLine(make_shared<Line>(vp[i], vp[i + 1]));
        return path;
    }
};

struct Arc : Line
{
    Point cp;
    bool anticlockwise;
    // double r;
    Arc(Point mSp = {}, Point mEp = {}, Point mCp = {}, bool mAnticlockwise = true, Position mPos = UNKNOW)
        : Line(mSp, mEp, mPos), cp(mCp), anticlockwise(mAnticlockwise) {}

    virtual void reverse() override
    {
        Line::reverse();
        anticlockwise = !anticlockwise;
    }
    virtual bool equalLine(shared_ptr<Line> line) const override
    {
        shared_ptr<Arc> arc = dynamic_pointer_cast<Arc>(line);
        if (!arc)
            return false;
        return equalPoint(cp, arc->cp) &&
               ((equalPoint(sp, arc->sp) && equalPoint(ep, arc->ep) && anticlockwise == arc->anticlockwise) ||
                (equalPoint(sp, arc->ep) && equalPoint(ep, arc->sp) && !anticlockwise == arc->anticlockwise));
        return false;
    }
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
        return equalPoint(start(), end());
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
        if (equalPoint(end(), line->sp))
        {
            lines.emplace_back(line);
            return true;
        }

        if (equalPoint(end(), line->ep))
        {
            line->reverse();
            lines.emplace_back(line);
            return true;
        }

        if (equalPoint(start(), line->ep))
        {
            lines.insert(lines.cbegin(), line);
            return true;
        }

        if (equalPoint(start(), line->sp))
        {
            line->reverse();
            lines.insert(lines.cbegin(), line);
            return true;
        }

        return false;
    }

    bool addPath(shared_ptr<Path> path)
    {
        if (equalPoint(end(), path->start()))
        {
            lines.insert(lines.cend(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        if (equalPoint(end(), path->end()))
        {
            path->reverse();
            lines.insert(lines.cend(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        if (equalPoint(start(), path->end()))
        {
            lines.insert(lines.cbegin(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        if (equalPoint(start(), path->start()))
        {
            path->reverse();
            lines.insert(lines.cbegin(), path->lines.cbegin(), path->lines.cend());
            return true;
        }

        return false;
    }

    shared_ptr<Path> toPath() const
    {
        auto path = make_shared<Path>();
        for (const auto &line : lines)
             path->addPath(line->toPath());
        return path;
    }
};

vector<shared_ptr<Path>> coverPath(vector<shared_ptr<Path>> in_path)
{
    vector<shared_ptr<Path>> out_path;
    while (!in_path.empty())
    {
        shared_ptr<Path> path = in_path.back();
        in_path.pop_back();

        if (!path->close())
        {
            int last_size;
            do
            {
                last_size = in_path.size();
                for (int i = 0; i < in_path.size(); i++)
                {
                    if (path->addPath(in_path[i]))
                    {
                        in_path.erase(in_path.cbegin() + i);
                        break;
                    }
                }

                if (path->close())
                    break;
            } while (last_size != in_path.size());
        }
        out_path.emplace_back(path);
    }
    return out_path;
}

void boolOperation(shared_ptr<Path> &left, shared_ptr<Path> &right)
{
    const int n_left = left->size();
    const int n_rigiht = right->size();
    for (int i = 0; i < n_left; i++)
        for (int j = 0; j < n_rigiht; j++)
            left->lines[i]->intersect(right->lines[j]);
    
    left = left->toPath();
    right = right->toPath();
}

int main()
{
    return 0;
}