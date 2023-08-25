#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <set>
using namespace std;

class Point
{
public:
    double x, y;
    Point(double x0 = 0.0, double y0 = 0.0) : x(x0), y(y0) {}
};

inline bool equal(Point pa, Point pb, double error = 0.0001)
{
    return abs(pa.x - pb.x) <= error && abs(pa.y - pb.y) <= error;
}

class Path
{
public:
    vector<shared_ptr<Path>> objs;

    virtual int size() const
    {
        return objs.size();
    }

    virtual void reverse()
    {
        shared_ptr<Path> tmp;
        int n = size();
        for (int i = 0, j = n - 1; i < j; i++, j--)
        {
            tmp = objs[i];
            objs[i] = objs[j];
            objs[j] = tmp;
        }
        for (int i = 0; i < n; i++)
            objs[i]->reverse();
    }

    virtual ~Path() = default;
    virtual Point getSatartPoint() const
    {
        return objs.front()->getSatartPoint();
    }
    virtual Point getEndPoint() const
    {
        return objs.back()->getEndPoint();
    }
    bool isClosed()
    {
        return equal(getSatartPoint(), getEndPoint());
    }
};

class Line : public Path
{
public:
    Point start_p, end_p;
    virtual int size() const
    {
        return 1;
    }
    virtual void reverse()
    {
        Point tmp = start_p;
        start_p = end_p;
        end_p = tmp;
    }

    virtual Point getSatartPoint() const
    {
        return start_p;
    }
    virtual Point getEndPoint() const
    {
        return end_p;
    }
};

// 包含圆，椭圆，圆弧，椭圆弧
class Arc : public Path
{
public:
    Point center_p;      // 中心
    double ra, rb;       // 半长轴，半短轴，相等时为圆
    double rotate_angle; // 绕中心旋转角度（逆时针为正）
    double start_angle;  // 开始角度
    double span_angle;   // 横跨角度（逆时针为正）
    virtual int size() const
    {
        return 1;
    }
    virtual void reverse()
    {
        start_angle += span_angle;
        span_angle = -span_angle;
    }

    virtual Point getSatartPoint() const
    {
        return Point(center_p.x + ra * cos(start_angle) * cos(rotate_angle), center_p.y + rb * sin(start_angle) * sin(rotate_angle));
    }
    virtual Point getEndPoint() const
    {
        return Point(center_p.x + ra * cos(start_angle + span_angle) * cos(rotate_angle), center_p.y + rb * sin(start_angle + span_angle) * sin(rotate_angle));
    }
};

shared_ptr<Path> newPath(shared_ptr<Path> p1, shared_ptr<Path> p2)
{
    if (equal(p1->getEndPoint(), p2->getEndPoint()))
    {
        p2->reverse();
    }
    else if (equal(p1->getSatartPoint(), p2->getEndPoint()))
    {
        shared_ptr<Path> tmp = p1;
        p1 = p2;
        p2 = tmp;
    }
    else if (equal(p1->getSatartPoint(), p2->getSatartPoint()))
    {
        p2->reverse();

        shared_ptr<Path> tmp = p1;
        p1 = p2;
        p2 = tmp;
    }

    shared_ptr<Path> p;
    if (p1->size() == 1)
    {
        p->objs.emplace_back(p1);
    }
    else
    {
        for (int i = 0; i < p1->objs.size(); i++)
            p->objs.emplace_back(p1->objs[i]);
    }

    if (p2->size() == 1)
    {
        p->objs.emplace_back(p2);
    }
    else
    {
        for (int i = 0; i < p2->objs.size(); i++)
            p->objs.emplace_back(p2->objs[i]);
    }
    return p;
}

Path getPath(Path &in_path)
{
    Path out_path;

    int i = -1;
    while (!in_path.objs.empty())
    {
        i = (i + 1) % in_path.objs.size();
        shared_ptr<Path> pi = in_path.objs[i];
        if (pi->isClosed()) // 封闭路径
        {
            out_path.objs.emplace_back(pi);
            in_path.objs.erase(in_path.objs.begin() + i);
            continue;
        }

        set<int> set_left;  // 当前路径左端连接路径序号
        set<int> set_right; // 当前路径右端连接路径序号
        for (int j = 0; j < in_path.objs.size(); j++)
        {
            if (j == i)
                continue;

            shared_ptr<Path> pj = in_path.objs[j];
            if (equal(pi->getSatartPoint(), pj->getEndPoint()) || equal(pi->getSatartPoint(), pj->getSatartPoint()))
                set_left.insert(j);

            if (equal(pi->getEndPoint(), pj->getSatartPoint()) || equal(pi->getEndPoint(), pj->getEndPoint()))
                set_right.insert(j);
        }

        if (set_left.empty() && set_right.empty()) // 左右两端都无连接路径
        {
            out_path.objs.emplace_back(pi);
            in_path.objs.erase(in_path.objs.begin() + i);
        }
        else if (set_right.size() == 1) // 右端有一条连接路径
        {
            int j = *set_right.cbegin();
            shared_ptr<Path> tmp = newPath(pi, in_path.objs[j]); // 当前路径与右端路径相连，生成新路径
            // 移除当前路径与右端路径
            if (i < j)
            {
                in_path.objs.erase(in_path.objs.begin() + j);
                in_path.objs.erase(in_path.objs.begin() + i);
            }
            else
            {
                in_path.objs.erase(in_path.objs.begin() + i);
                in_path.objs.erase(in_path.objs.begin() + j);
            }

            if (tmp->isClosed()) // 新路径封闭，加入输出路径
                out_path.objs.emplace_back(tmp);
            else // 加入输入路径
                in_path.objs.emplace_back(tmp);
        }
        else if (set_left.size() == 1) // 左端有一条连接路径
        {
            int j = *set_left.cbegin();
            shared_ptr<Path> tmp = newPath(pi, in_path.objs[j]);
            if (i < j)
            {
                in_path.objs.erase(in_path.objs.begin() + j);
                in_path.objs.erase(in_path.objs.begin() + i);
            }
            else
            {
                in_path.objs.erase(in_path.objs.begin() + i);
                in_path.objs.erase(in_path.objs.begin() + j);
            }

            if (tmp->isClosed())
                out_path.objs.emplace_back(tmp);
            else
                in_path.objs.emplace_back(tmp);
        }
        else if (set_left.empty() || set_right.empty()) // 左端或右端无连接路径，且另一端有两个及以上连接路径
        {
            out_path.objs.emplace_back(pi);
            in_path.objs.erase(in_path.objs.begin() + i);
        }
        else // 左右两端各有两个及以上路径相连
        {
            for (auto left : set_left)
            {
                if (set_right.count(left) == 1) // 有交集，形成封闭路径
                {
                    int j = left;
                    shared_ptr<Path> tmp = newPath(pi, in_path.objs[j]);
                    if (i < j)
                    {
                        in_path.objs.erase(in_path.objs.begin() + j);
                        in_path.objs.erase(in_path.objs.begin() + i);
                    }
                    else
                    {
                        in_path.objs.erase(in_path.objs.begin() + i);
                        in_path.objs.erase(in_path.objs.begin() + j);
                    }
                    out_path.objs.emplace_back(tmp);

                    break;
                }
            }
        }
    }

    return out_path;
}

int main()
{
    return 0;
}
