#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <tuple>
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

vector<pair<double, double>> intersect(double x1, double y1, double x2, double y2,
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

        double x = x0[left_ind];
        double y = y0[left_ind];
        points.emplace_back(x, y);

        if (equal(x0[left_ind], x0[right_ind]) == false)
        {
            x = x0[right_ind];
            y = y0[right_ind];
            points.emplace_back(x, y);
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
            points.emplace_back(x, y);
        }
    }
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
    double x;
    double y;
    bool flag{false};
    PointPosition pp{UNKNOW};
    Point(double x0 = 0.0, double y0 = 0.0) : x(x0), y(y0) {}

    bool equalPoint(Point p, double error = 0.00001) const
    {
        return equal(p.x, x, error) && equal(p.y, y, error);
    }
    bool equalXy(double x0, double y0, double error = 0.00001) const
    {
        return equal(x0, x, error) && equal(y0, y, error);
    }
    double distancePoint(Point p) const
    {
        return pow(p.x - x, 2) + pow(p.y - y, 2);
    }
};

struct Segment
{
    Point sp;
    Point ep;
    vector<Point> ip;
    Segment(Point sp0 = Point(), Point ep0 = Point()) : sp(sp0.x, sp0.y), ep(ep0.x, ep0.y) {}

    inline int getSize() const { return 1 + ip.size(); };
    Point &getPoint(int index)
    {
        // if (index < 0)
        //    return lp;
        // else
        if (index == 0)
            return sp;
        else if (index < getSize())
            return ip[index - 1];
        else
            return ep;
    }

    int getIndex(double x, double y)
    {
        for (int index = 0; index < getSize(); index++)
        {
            if (equal(getPoint(index).x, x) && equal(getPoint(index).y, y))
                return index;
        }
        return -1;
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

        return cross(x - x1, y - y1, x - x2, y - y2) == 0.0;
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

        if (y1 == y2)
            return 0;

        if (y1 > y2)
        {
            double tmp = y1;
            y1 = y2;
            y2 = tmp;
            tmp = x1;
            x1 = x2;
            x2 = tmp;
        }
        if (cross(x2 - x1, y2 - y1, x - x1, y - y1) < 0.0 && y != y2)
            return 1;

        return 0;
    }

    void insertPoint(Point p)
    {
        p.flag = true;
        p.pp = ON;
        if (p.equalPoint(sp))
        {
            sp.flag = true;
            sp.pp == ON;
        }
        else if (p.equalPoint(ep))
        {
            ep.flag = true;
            ep.pp == ON;
        }
        else
        {
            const int n = ip.size();
            for (int i = 0; i < n; i++)
                if (p.equalPoint(ip[i]))
                    return;

            int index = 0;
            while (index < n && ip[index].distancePoint(sp) < p.distancePoint(sp))
                index++;
            ip.insert(ip.cbegin() + index, p);
        }
    }

    void insertXy(double x, double y)
    {
        Point p(x, y);
        p.flag = true;
        p.pp = ON;
        if (p.equalPoint(sp))
        {
            sp.flag = true;
            sp.pp == ON;
        }
        else if (p.equalPoint(ep))
        {
            ep.flag = true;
            ep.pp == ON;
        }
        else
        {
            const int n = ip.size();
            for (int i = 0; i < n; i++)
                if (p.equalPoint(ip[i]))
                    return;

            int index = 0;
            while (index < n && ip[index].distancePoint(sp) < p.distancePoint(sp))
                index++;
            ip.insert(ip.cbegin() + index, p);
        }
    }

    void intersect(Segment &s)
    {
        double x1 = sp.x;
        double x2 = ep.x;
        double x3 = s.sp.x;
        double x4 = s.ep.x;
        if (!isOverlap(x1, x2, x3, x4))
            return;

        double y1 = sp.y;
        double y2 = ep.y;
        double y3 = s.sp.y;
        double y4 = s.ep.y;
        if (!isOverlap(y1, y2, y3, y4))
            return;

        Point p;
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

            p.x = x0[left_ind];
            p.y = y0[left_ind];
            insertPoint(p);
            s.insertPoint(p);

            if (equal(x0[left_ind], x0[right_ind]) == false)
            {
                p.x = x0[right_ind];
                p.y = y0[right_ind];
                insertPoint(p);
                s.insertPoint(p);
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
                double ss = cross(x1 - x3, y1 - y3, x2 - x1, y2 - y1) / cross(x4 - x3, y4 - y3, x2 - x1, y2 - y1);
                double x = x1 + ss * (x2 - x1);
                double y = y1 + ss * (y2 - y1);

                p.x = x;
                p.y = y;
                insertPoint(p);
                s.insertPoint(p);
            }
        }
    }
};

PointPosition pointInPolygon(Point p, const vector<Segment> &vs)
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

void nextPoint(vector<Segment> &vs, int &i, int &j)
{
    j++;
    if (j == vs[i].getSize())
    {
        j = 0;
        i = (i + 1) % vs.size();
    }
}

bool findPointByPosition(vector<Segment> &vs, PointPosition pp, int &i, int &j)
{
    if (vs.size() == 0)
        return false;

    int start_i = i;
    int start_j = j;
    do
    {
        if (vs[i].getPoint(j).pp == pp)
            return true;
        nextPoint(vs, i, j);
    } while (i != start_i || j != start_j);

    return false;
}

bool findPointWithoutVisited(vector<Segment> &vs, PointPosition pp, int &i, int &j, const set<pair<int, int>> &s)
{
    if (vs.size() == 0)
        return false;

    int start_i = i;
    int start_j = j;
    do
    {
        if (vs[i].getPoint(j).pp == pp && s.count(make_pair(i, j)) == 0)
            return true;
        nextPoint(vs, i, j);
    } while (i != start_i || j != start_j);

    return false;
}

void findPoint(vector<Segment> &vs, Point p, int &i, int &j)
{
    const int n = vs.size();
    while (true)
    {
        Point c_p = vs[i].getPoint(j);
        if (c_p.x == p.x && c_p.y == p.y)
            break;
        nextPoint(vs, i, j);
    }
}

void setPointPosition(vector<Segment> &left, vector<Segment> &right)
{
    int i;
    int j;

    i = 0;
    j = 0;
    while (findPointByPosition(left, UNKNOW, i, j))
        left[i].getPoint(j).pp = pointInPolygon(left[i].getPoint(j), right);

    i = 0;
    j = 0;
    while (findPointByPosition(right, UNKNOW, i, j))
        right[i].getPoint(j).pp = pointInPolygon(left[i].getPoint(j), left);
}

vector<vector<Segment>> intersection(vector<Segment> &left, vector<Segment> &right)
{
    vector<tuple<double, double, int, int>> points; // 交点及对应片段序号
    const int n_left = left.size();
    const int n_right = right.size();
    for (int i = 0; i < n_left; i++)
    {
        double x1 = left[i].sp.x;
        double y1 = left[i].sp.y;
        double x2 = left[i].ep.x;
        double y2 = left[i].ep.y;

        for (int j = 0; j < n_right; j++)
        {
            double x3 = right[j].sp.x;
            double y3 = right[j].sp.y;
            double x4 = right[j].ep.x;
            double y4 = right[j].ep.y;

            for (const auto &point : intersect(x1, y1, x2, y2, x3, y3, x4, y4))
            {
                double x = point.first;
                double y = point.second;
                if (!left[i].ep.equalXy(x, y) && !right[j].ep.equalXy(x, y))
                {
                    points.emplace_back(x, y, i, j);
                    left[i].insertXy(x, y);
                    right[j].insertXy(x, y);
                }
            }
        }
    }

    setPointPosition(left, right);

    /*
        int i;
        int j;
        set<pair<int, int>> s;
        int left_i;
        int left_j;
        int right_i = 0;
        int right_j = 0;
        Point first_p;
        Point left_p;
        Point right_p;
        Point cur_p;
        Point next_p;

        vector<vector<Segment>> vvs;
        while (findPointWithoutVisited(left, ON, i, j, s))
        {
            s.insert(make_pair(i, j));
            first_p = left[i].getPoint(j);
            next_p = first_p;

            vector<Segment> vs;
            do
            {
                cur_p = next_p;

                left_i = i;
                left_j = j;
                nextPoint(left, left_i, left_j);
                left_p = left[left_i].getPoint(left_j);

                findPoint(right, left_p, right_i, right_i);
                right_p = left[right_i].getPoint(right_i);
                if (left_p.pp == IN || right_p.pp == OUT)
                {
                    next_p = left_p;
                }
                else if (right_p.pp == IN || left_p.pp == OUT)
                {
                    next_p = right_p;
                }
                else
                {
                    break;
                }
                vs.emplace_back(cur_p, next_p);

            } while (!next_p.equalPoint(first_p));
            vvs.emplace_back(vs);
        }
        return vvs;
    */

    vector<vector<Segment>> vvs;

    while (!points.empty())
    {
        tuple<double, double, int, int> first = points.back();
        points.pop_back();

        double first_x = get<0>(first);
        double first_y = get<1>(first);
        int left_i = get<2>(first);
        int right_i = get<3>(first);

        double next_x = first_x;
        double next_y = first_y;

        double cur_x;
        double cur_y;
        int left_j;
        int right_j;

        vector<Segment> vs;
        do
        {
            cur_x = next_x;
            cur_y = next_y;

            left_j = left[left_i].getIndex(cur_x, cur_y);
            if (left_j == left[left_i].getSize() - 1)
            {
                left_j = 0;
                left_i = (left_i + 1) % n_left;
            }

            right_j = left[right_i].getIndex(cur_x, cur_y);
            if (right_j == left[right_i].getSize() - 1)
            {
                right_j = 0;
                right_i = (right_i + 1) % n_right;
            }

            Point left_p = left[left_i].getPoint(left_j);
            Point right_p = left[right_i].getPoint(right_i);
            if (left_p.pp == IN || right_p.pp == OUT)
            {
                next_x = left_p.x;
                next_y = left_p.y;
            }
            else if (right_p.pp == IN || left_p.pp == OUT)
            {
                next_x = right_p.x;
                next_y = right_p.y;
            }
            else
            {
            }

        } while (!equal(next_x, first_x) || !equal(next_y, first_y));
        vvs.emplace_back(vs);
    }

    return vvs;
}

int main()
{
    return 0;
}
