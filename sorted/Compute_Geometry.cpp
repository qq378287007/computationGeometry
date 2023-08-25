#include <iostream>
#include <vector>
#include <map>
#include <stack>
#include <algorithm>
#include <random>
#include <ctime>
using namespace std;

const double PI = 3.14159265;

/******************************* 基本几何数据类型 *******************************/
struct Point // 点或矢量
{
    double x, y, z;
    Point(double a = 0.0, double b = 0.0, double c = 0.0)
    {
        x = a;
        y = b;
        z = c;
    }
};

Point add(const Point &lhs, const Point &rhs)
{
    return Point(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

Point sub(const Point &lhs, const Point &rhs)
{
    return Point(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

Point mul(const Point &p, double ratio)
{
    return Point(p.x * ratio, p.y * ratio, p.z * ratio);
}

Point div(const Point &p, double ratio)
{
    return Point(p.x / ratio, p.y / ratio, p.z / ratio);
}

struct Line // 线段或直线
{
    Point s, e;
    bool is_seg;
    Line(Point a = Point(), Point b = Point(), bool _is_seg = true)
    {
        s = a;
        e = b;
        is_seg = _is_seg;
    }
};

void boxOfPolygon(const vector<Point> &polygon, Point &down_left, Point &up_right);
int segToCircle(const Point &c, double radius, const Line &l);

struct Triangle // 三角形或平面
{
    Point v0, v1, v2;
    bool is_plane;
    Triangle(Point a = Point(), Point b = Point(), Point c = Point(), bool _is_plane = false)
    {
        v0 = a;
        v1 = b;
        v2 = c;
        is_plane = _is_plane;
    }
};

// 一、点
// 1.1、距离
double distance(const Point &p1, const Point &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

// 1.2、长度
double length(const Point &vec)
{
    return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

// 1.3、标准化
Point normalize(const Point &vec)
{
    return div(vec, length(vec));
}

// 1.4、点乘
double dotMultiply(const Point &vec1, const Point &vec2)
{
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}
double dotMultiply(const Point &op, const Point &p1, const Point &p2)
{
    return (p1.x - op.x) * (p2.x - op.x) + (p1.y - op.y) * (p2.y - op.y) + (p1.z - op.z) * (p2.z - op.z);
}

// 1.5、叉乘
Point multiply(const Point &vec1, const Point &vec2)
{
    double x = vec1.y * vec2.z - vec2.y * vec1.z;
    double y = vec1.z * vec2.x - vec2.z * vec1.x;
    double z = vec1.x * vec2.y - vec2.x * vec1.y;
    return Point(x, y, z);
}
Point multiply(const Point &op, const Point &p1, const Point &p2)
{
    double x = (p1.y - op.y) * (p2.z - op.z) - (p2.y - op.y) * (p1.z - op.z);
    double y = (p1.z - op.z) * (p2.x - op.x) - (p2.z - op.z) * (p1.x - op.x);
    double z = (p1.x - op.x) * (p2.y - op.y) - (p2.x - op.x) * (p1.y - op.y);
    return Point(x, y, z);
}

// 1.6、点到线的距离
// s和e是线l上两点，o是p在线l上的投影点
double ptolDistance(const Point &p, const Line &l)
{
    Point line_vec = sub(l.e, l.s); // 矢量se
    Point point_vec = sub(p, l.s);  // 矢量sp

    double line_length = length(line_vec);                               // 矢量se长度
    double point_length = length(point_vec);                             // 矢量sp长度
    double project_len = dotMultiply(line_vec, point_vec) / line_length; // 投影so长度（有正负）
    return sqrt(pow(point_length, 2) - pow(project_len, 2));             // 垂线po长度（勾股定理）
}

// 1.7、点到线的投影点
// s和e是线l上两点，o是p在线l上的投影点，e'是线l上一点满足se'为单位矢量
Point ptolProjection(const Point &p, const Line &l)
{
    Point line_vec = sub(l.e, l.s);            // 矢量se
    Point point_vec = sub(p, l.s);             // 矢量sp
    Point unit_line_vec = normalize(line_vec); // 单位矢量se'

    double project_len = dotMultiply(point_vec, unit_line_vec); // 投影so长度（有正负）
    return add(l.s, mul(unit_line_vec, project_len));           // 投影点o = s + se' * project_len
}

// 1.8、点关于线的对称点
Point ptolSymmetry(const Point &p, const Line &l)
{
    Point project_p = ptolProjection(p, l); // 点p在直线l的投影点o
    Point project_vec = sub(project_p, p);  // 点p到投影点o的向量po
    return add(p, mul(project_vec, 2));     // 对称点p' = p + po * 2
}

// 1.9、点是否在线(直线或线段)上
bool isponl(const Point &p, const Line &l)
{
    Point line_vec = sub(l.e, l.s); // 矢量se
    Point point_vec1 = sub(p, l.s); // 矢量sp
    Point point_vec2 = sub(p, l.e); // 矢量ep

    Point mul_vec = multiply(line_vec, point_vec1); // 叉乘为0，矢量se和sp共线
    if (!l.is_seg)
        return (0.0 == length(mul_vec)); // 点在直线上

    double dot = dotMultiply(point_vec1, point_vec2); // 点乘，矢量sp和ep的相对位置（点p是否位于线段se上）
    return (0.0 == length(mul_vec) && dot <= 0.0);    // 点在线段上
}

// 1.10、矢量夹角余弦
double Cos(const Point &vec1, const Point &vec2)
{
    Point unit_vec1 = normalize(vec1);
    Point unit_vec2 = normalize(vec2);
    return dotMultiply(unit_vec1, unit_vec2);
}
double Cos(const Point &op, const Point &p1, const Point &p2)
{
    Point vec1 = sub(p1, op);
    Point vec2 = sub(p2, op);
    return Cos(vec1, vec2);
}

// 1.11、矢量夹角正弦
double Sin(const Point &vec1, const Point &vec2)
{
    return sqrt(1.0 - pow(Cos(vec1, vec2), 2));
}
double Sin(const Point &op, const Point &p1, const Point &p2)
{
    Point vec1 = sub(p1, op);
    Point vec2 = sub(p2, op);
    return Sin(vec1, vec2);
}

// 1.12、矢量夹角正切
double Tan(const Point &vec1, const Point &vec2)
{
    double cos = Cos(vec1, vec2);
    if (0.0 == cos)
        return -1;

    double sin = Sin(vec1, vec2);
    return sin / cos;
}
double Tan(const Point &op, const Point &p1, const Point &p2)
{
    Point vec1 = sub(p1, op);
    Point vec2 = sub(p2, op);
    return Tan(vec1, vec2);
}

// 1.13、计算点的夹角角度
double Angle(const Point &vec1, const Point &vec2, bool is_radian)
{
    double cos_value = Cos(vec1, vec2);
    if (is_radian)
        return acos(cos_value);
    return acos(cos_value) / PI * 180.0;
}
double Angle(const Point &op, const Point &p1, const Point &p2, bool is_radian)
{
    Point vec1 = sub(p1, op);
    Point vec2 = sub(p2, op);
    return Angle(vec1, vec2, is_radian);
}

// 1.14、判断三点是否共线
bool isPointsCollinear(const Point &p1, const Point &p2, const Point &p3)
{
    Line line(p1, p2, false);
    return isponl(p3, line); // 第三个点是否在前两个点的直线上
}

// 1.15、在（-1，-1）到（1，1）随机生成num个点
vector<Point> randomGenPoints(int num)
{
    std::mt19937 rng;
    rng.seed(std::random_device{}());
    std::uniform_real_distribution<double> dist(-0.9, 0.9);
    vector<Point> result;
    for (int i = 0; i < num; ++i)
    {
        double rand_x = dist(rng);
        double rand_y = dist(rng);
        result.push_back(Point(rand_x, rand_y));
    }
    return result;
}

// 二、线
// 2.1、线段或直线是否相交
bool isSegIntersect(const Line &l1, const Line &l2, Point &inter_p)
{
    Point line1 = sub(l1.e, l1.s);
    Point line2 = sub(l2.e, l2.s);
    Point norm1 = normalize(line1);
    Point norm2 = normalize(line2);
    // 线段相交
    if (l1.is_seg)
    {
        // 端点在线段上
        if (isponl(l1.s, l2))
        {
            inter_p = l1.s;
            return true;
        }
        if (isponl(l1.e, l2))
        {
            inter_p = l1.e;
            return true;
        }
        if (isponl(l2.s, l1))
        {
            inter_p = l2.s;
            return true;
        }
        if (isponl(l2.e, l1))
        {
            inter_p = l2.e;
            return true;
        }
        // 判断线段是否相互跨立
        double dot1 = dotMultiply(multiply(sub(l2.s, l1.s), line1), multiply(sub(l2.e, l1.s), line1));
        double dot2 = dotMultiply(multiply(sub(l1.s, l2.s), line2), multiply(sub(l1.e, l2.s), line2));
        if (dot1 < 0.0 && dot2 < 0.0)
        {
            double t1 = length(multiply(sub(l1.s, l2.s), norm2)) / length(multiply(norm2, norm1));
            double t2 = length(multiply(sub(l2.s, l1.s), norm1)) / length(multiply(norm1, norm2));

            inter_p = add(l1.s, mul(norm1, t1));
            return true;
        }
        else
        {
            return false;
        }
    }
    // 直线相交
    else
    {
        if (Cos(line1, line2) == 1.0)
            return false;

        double t1 = length(multiply(sub(l1.s, l2.s), norm2)) / length(multiply(norm2, norm1));
        double t2 = length(multiply(sub(l2.s, l1.s), norm1)) / length(multiply(norm1, norm2));

        inter_p = add(l1.s, mul(norm1, t1));
        return true;
    }
}

// 2.2、求直线的夹角
double angleOfLines(const Line &l1, const Line &l2, bool is_radian)
{
    Point line1 = sub(l1.e, l1.s);
    Point line2 = sub(l2.e, l2.s);
    return Angle(line1, line2, is_radian);
}

// 2.3、一阶贝塞尔曲线插值
// 起点s和终点e间插入inter_num个点
// Path = (1-t)A + tB
vector<Point> firstOrderBezier(const Point &s, const Point &e, int inter_num)
{
    vector<Point> res;
    res.push_back(s);
    for (int i = 1; i <= inter_num; ++i)
    {
        double a1 = double(i) / double(inter_num + 1);
        double a2 = 1.0 - a1;
        res.push_back(add(mul(s, a2), mul(e, a1)));
    }
    res.push_back(e);
    return res;
}

// 2.4、二阶贝塞尔曲线插值
// Path = (1-t)²A + 2t(1-t)B + t²C
vector<Point> secondOrderBezier(const Point &s, const Point &e, const Point &p, int inter_num)
{
    vector<Point> res;
    res.push_back(s);
    for (int i = 1; i <= inter_num; ++i)
    {
        double a = double(i) / double(inter_num + 1);
        double a1 = pow(a, 2);
        double a2 = 2 * a * (1.0 - a);
        double a3 = pow(1.0 - a, 2);
        res.push_back(add(add(mul(s, a3), mul(p, a2)), mul(e, a1)));
    }
    res.push_back(e);
    return res;
}

// 2.5、三阶贝塞尔曲线插值
// Path = (1-t)³A + 3t(1-t)²B + 3t²(1-t)C + t³D
vector<Point> thirdOrderBezier(const Point &s, const Point &e, const Point &p1, const Point &p2, int inter_num)
{
    vector<Point> res;
    res.push_back(s);
    for (int i = 1; i <= inter_num; ++i)
    {
        double a = double(i) / double(inter_num + 1);
        double a1 = pow(a, 3);
        double a2 = 3 * pow(a, 2) * (1.0 - a);
        double a3 = 3 * pow(1.0 - a, 2) * a;
        double a4 = pow(1.0 - a, 3);
        res.push_back(add(add(add(mul(s, a4), mul(p1, a3)), mul(p2, a2)), mul(e, a1)));
    }
    res.push_back(e);
    return res;
}

// n阶贝塞尔曲线插值
// k从0遍历到n
// Path = Sum C_n^k t^k (1-t)^{n-k} P_k

// 2.6、在（-1，-1）到（1，1）随机生成num条线
vector<Line> randomGenLines(int num)
{
    std::mt19937 rng;
    rng.seed(std::random_device{}());
    std::uniform_real_distribution<double> dist(-0.9, 0.9);
    vector<Line> result;
    for (int i = 0; i < num; ++i)
    {
        double rand_sx = dist(rng);
        double rand_sy = dist(rng);
        Point p1(rand_sx, rand_sy);
        double rand_ex = dist(rng);
        double rand_ey = dist(rng);
        Point p2(rand_ex, rand_ey);
        result.push_back(Line(p1, p2, true));
    }
    return result;
}

// 三、三角形
// 3.1、三角形三个点是否能够构成三角形
bool isTriangle(const Triangle &t)
{
    return !isPointsCollinear(t.v0, t.v1, t.v2);
}

// 3.2、点是否在三角形内部（重心法）
// 算法方法链接： https://www.cnblogs.com/graphics/archive/2010/08/05/1793393.html
// 参数： t : 三角形 p : 需要判断的点 u, v分别为用于表示点在两条边上投影系数
bool isPointInTriangle(const Triangle &t, const Point &p, double &u, double &v)
{
    Point vec1 = sub(t.v1, t.v0);
    Point vec2 = sub(t.v2, t.v0);
    Point vec_p = sub(p, t.v0);

    double dot00 = dotMultiply(vec1, vec1);
    double dot01 = dotMultiply(vec1, vec2);
    double dot02 = dotMultiply(vec1, vec_p);
    double dot11 = dotMultiply(vec2, vec2);
    double dot12 = dotMultiply(vec2, vec_p);

    double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

    u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
    v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
    return u >= 0.0 && v >= 0.0 && u + v <= 1.0;
}

// 3.3、计算平面的单位法向量
Point getUnitNormal(const Triangle &t)
{
    Point vec1 = sub(t.v1, t.v0);
    Point vec2 = sub(t.v2, t.v0);
    return normalize(multiply(vec1, vec2));
}

// 3.4、点到平面最近的点，即点到平面的投影
Point ptotProjection(const Triangle &t, const Point &p)
{
    Point vec_p = sub(p, t.v0);
    Point unit_normal = getUnitNormal(t);

    double ratio = dotMultiply(vec_p, unit_normal);
    return sub(p, mul(unit_normal, ratio));
}

// 3.5、线段和平面的交点
Point ltotInterPoint(const Triangle &t, const Line &l)
{
    Point line_vec = sub(l.e, l.s);
    Point point_vec = sub(t.v0, l.s);
    Point unit_plane_normal = getUnitNormal(t);

    double ratio = dotMultiply(point_vec, unit_plane_normal) / dotMultiply(unit_plane_normal, line_vec);
    return add(l.s, mul(line_vec, ratio));
}

// 3.6、点到平面的距离
double ptotDistance(const Triangle &t, const Point &p)
{
    Point project_p = ptotProjection(t, p);
    return distance(p, project_p);
}

// 3.7、计算三角形的面积
double areaOfTriangle(const Triangle &t)
{
    return 0.5 * length(multiply(sub(t.v1, t.v0), sub(t.v2, t.v0)));
}

// 四、多边形（默认逆时针排列，二维点）
// 4.1、判断多边形顶点的凹凸性
// 参数： polygon : 多边形点集合 flags : 标志每个点是否是凸的
void checkConvex(const vector<Point> &polygon, vector<bool> &flags)
{
    flags.resize(polygon.size());

    // 找到第一个凸的顶点
    int index = 0;
    for (int i = 1; i < polygon.size(); ++i)
    {
        if (polygon[i].y < polygon[index].y || (polygon[i].y == polygon[index].y && polygon[i].x < polygon[index].x))
            index = i;
    }
    /* 判断每个点的凹凸性
     *  通过判断前后两个点的向量叉乘判断是否满足逆时针
     */
    int size = polygon.size() - 1;
    flags[index] = true;
    while (size)
    {
        if (multiply(polygon[index], polygon[(index + 1) % size], polygon[(index + 2) % size]).z >= 0)
            flags[(index + 1) % size] = true;
        else
            flags[(index + 1) % size] = false;
        index++;
        size--;
    }
}

// 4.2、判断多边形是否为凸多边形
bool isConvex(const vector<Point> &polygon)
{
    vector<bool> flags;
    // 判断每个点的凹凸性
    checkConvex(polygon, flags);
    // 如果有一个点不是凸的，则此多边形为非凸
    for (auto c : flags)
        if (!c)
            return false;
    return true;
}

// 4.3、求多边形围成的面积
double areaOfPolygon(const vector<Point> &polygon)
{
    // 如果多边形点的数量少于三个则非法
    int size = polygon.size();
    if (size < 3)
        return 0;

    double area(0.0);
    for (int i = 0; i < size; ++i)
        area += polygon[i].y * (polygon[(i - 1 + size) % size].x - polygon[(i + 1) % size].x);
    return (area / 2);
}

// 4.4、判断多边形是否按照逆时针排列
// 参数： polygon : 多边形
bool isConterClock(const vector<Point> &polygon)
{
    return areaOfPolygon(polygon) > 0;
}

// 4.5、判断点是否在多边形内部
// 判断从点引出一条线段与多边形的交点的个数
// 奇数个则相交、偶数个则不相交
// 参数： polygon : 多边形 p : 需要判断的点
bool isPointInPolygon(const vector<Point> &polygon, const Point &p)
{
    Point down_left, up_right;
    boxOfPolygon(polygon, down_left, up_right);

    // 位于多边形外部一点
    Point out_p = sub(down_left, Point(10.0, 0.0));

    int cnt(0);
    Line p_line(p, out_p, true);
    for (int i = 0; i < polygon.size(); ++i)
    {
        Point s = polygon[i];
        Point e = polygon[(i + 1) % polygon.size()];
        Line seg(s, e, true);
        Point inter_p;
        if (isSegIntersect(p_line, seg, inter_p))
            cnt++;
    }

    return (cnt % 2 == 1);
}

// 4.6、判断线段是否在多边形内部
// 线段在多边形内部的条件是两个端点都在多边形内且不与多边形相交
// 参数： polygon : 多边形 l ： 线段
bool isSegInPolygon(const vector<Point> &polygon, const Line &l)
{
    // 首先判断线段端点是否都在多边形内部
    bool is_s_in = isPointInPolygon(polygon, l.s);
    bool is_e_in = isPointInPolygon(polygon, l.e);

    // 然后判断线段是否相交
    if (is_s_in && is_e_in)
    {
        for (int i = 0; i < polygon.size(); ++i)
        {
            Point s = polygon[i];
            Point e = polygon[(i + 1) % polygon.size()];
            Line seg(s, e, true);
            Point inter_p;
            if (isSegIntersect(l, seg, inter_p))
                return false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

// 4.7、判断圆是否在多边形内部
// 只有多边形所有的边都在圆的外部，圆才处于多边形内部
//	参数： polygon : 多边形 c : 圆心 radius ： 半径
bool isCircleInPolygon(const vector<Point> &polygon, const Point &c, double radius)
{
    for (int i = 0; i < polygon.size(); ++i)
    {
        const Point &p1 = polygon[i];
        const Point &p2 = polygon[(i + 1) % polygon.size()];
        Line line(p1, p2, true);
        if (segToCircle(c, radius, line) != 2)
            return false;
    }
    return true;
}

// 4.8、寻找点集凸包算法（graham算法）
// 算法链接：https://blog.csdn.net/acm_zl/article/details/9342631
// 参数： points ： 平面点集
// 目前实现的版本有问题
vector<Point> findConvexGraham(const vector<Point> &points)
{
    vector<Point> result;

    // 点的数量必须大于三个
    if (points.size() < 3)
        return result;

    // 寻找最底部的点
    int index = 0;
    for (int i = 0; i < points.size(); ++i)
        if (points[i].y < points[index].y)
            index = i;

    Point convex_p = points[index];

    // 计算每个点的极角
    map<double, int> cos_map;
    Point x_vec(1.0, 0.0);
    for (int i = 0; i < points.size(); ++i)
    {
        if (i != index)
        {
            double cos_value = Cos(sub(points[i], convex_p), x_vec);
            // 如果有多个点有相同的极角，则取最远的点
            if (cos_map.count(-cos_value) != 0)
            {
                if (length(points[i]) > length(points[cos_map[-cos_value]]))
                    cos_map[-cos_value] = i;
            }
            else
                cos_map[-cos_value] = i;
        }
    }

    // 保存结果的栈
    stack<int> result_stack;
    // 存入开始的两个点
    result_stack.push(index);
    result_stack.push(cos_map.begin()->second);

    for (auto iter = (++cos_map.begin()); iter != cos_map.end(); ++iter)
    {
        int first = result_stack.top();
        result_stack.pop();
        int second = result_stack.top();

        Point vec1 = sub(points[first], points[second]);
        Point vec2 = sub(points[iter->second], points[first]);
        if (multiply(vec1, vec2).z >= 0)
            result_stack.push(first);
        result_stack.push(iter->second);
    }

    // 将数据从栈中读取
    while (!result_stack.empty())
    {
        result.push_back(points[result_stack.top()]);
        result_stack.pop();
    }

    std::reverse(result.begin(), result.end());

    return result;
}

//// 4.9、寻找点集凸包算法（上下凸包法）时间复杂度O(nlogn)
////
////	参数： points : 平面点集
////
//// 点根据字典序的比较函数
// bool cmp(Point a, Point b)
//{
//	if (a.x == b.x)
//		return a.y < b.y;
//	return a.x < b.x;
// }
// vector<Point> findConvex(const vector<Point>& points)
//{
//	vector<Point> result;
//	if (points.size() < 3)
//		return result;
//
//	vector<Point> tmp_points = points;
//	// 首先将所有点按照字典序排序
//	sort(tmp_points.begin(), tmp_points.end(), cmp);
//
//	// 上凸包
//	vector<Point> upper_hull;
//	// 存入第一个和第二个点
//	upper_hull.push_back(tmp_points[0]);
//	upper_hull.push_back(tmp_points[1]);
//	for (int i = 2; i < tmp_points.size(); ++i)
//	{
//		upper_hull.push_back(tmp_points[i]);
//		while (upper_hull.size() > 2 && multiply(sub(upper_hull[upper_hull.size() - 2], upper_hull[upper_hull.size() - 3]), sub(upper_hull[upper_hull.size() - 1], upper_hull[upper_hull.size() - 3])).z >= 0)
//		{
//			upper_hull.erase(upper_hull.end() - 2);
//		}
//	}
//	// 下凸包
//	vector<Point> lower_hull;
//	// 存入倒数第一第二个点
//	lower_hull.push_back(tmp_points[tmp_points.size() - 1]);
//	lower_hull.push_back(tmp_points[tmp_points.size() - 2]);
//	for (int i = tmp_points.size() - 3; i >= 0; --i)
//	{
//		lower_hull.push_back(tmp_points[i]);
//		while (lower_hull.size() > 2 && multiply(sub(lower_hull[lower_hull.size() - 2], lower_hull[lower_hull.size() - 3]), sub(lower_hull[lower_hull.size() - 1], lower_hull[lower_hull.size() - 3])).z >= 0)
//		{
//			lower_hull.erase(lower_hull.end() - 1);
//		}
//	}
//	// 删除重复点
//	lower_hull.erase(lower_hull.begin());
//	lower_hull.erase(lower_hull.end() - 1);
//
//	// 合并上下凸包
//	upper_hull.insert(upper_hull.end(), lower_hull.begin(), lower_hull.end());
//
//	result = upper_hull;
//
//	return result;
// }

// 4.10、求简单多边形重心
// 算法原理链接：
// 参数： polygon ： 简单多边形
Point centerOfPolygon(const vector<Point> &polygon)
{
    double polygon_area(0.0);
    Point center;
    Point origin;

    for (int i = 0; i < polygon.size(); ++i)
    {
        Point curr_p = polygon[i];
        Point next_p = polygon[(i + 1) % polygon.size()];
        Triangle t(origin, curr_p, next_p);

        double curr_area = areaOfTriangle(t);
        polygon_area += curr_area;

        center = add(center, mul(div(add(curr_p, next_p), 3), curr_area));
    }

    center = div(center, polygon_area);

    return center;
}

// 4.11、求肯定在多边形内部的一个点
// 定理1: 每个多边形至少有一个凸顶点，x坐标最大、最小的点肯定是凸顶点，y坐标最大、最小的点肯定是凸顶点
// 定理2：顶点数>= 4的简单多边形至少有一条对角线
// 参数： polygon ： 简单多边形
Point pointInPolygon(const vector<Point> &polygon)
{
    // 凸顶点和索引
    int index = 0;
    Point convex_p = polygon[0];
    // 寻找一个凸顶点
    for (int i = 0; i < polygon.size(); ++i)
    {
        if (polygon[i].y < convex_p.y)
        {
            index = i;
            convex_p = polygon[i];
        }
    }
    // 获取凸顶点前后一个点
    int size = polygon.size();
    Point pre_p = polygon[(index - 1 + size) % size];
    Point next_p = polygon[(index + 1) % size];
    Triangle t(convex_p, pre_p, next_p);
    double min_d = double(INT_MAX);
    bool flag = false;
    Point min_p;
    for (int i = 0; i < polygon.size(); ++i)
    {
        if (i == index || i == ((index - 1 + size) % size) || i == ((index + 1) % size))
            continue;
        flag = true;
        if (distance(convex_p, polygon[i]) < min_d)
        {
            min_p = polygon[i];
            min_d = distance(convex_p, polygon[i]);
        }
    }
    // 如何没有顶点在三角形内部，则返回前后点的中点
    if (!flag)
        return div(add(pre_p, next_p), 2);

    // 返回最近点和凸顶点的中点
    return div(add(convex_p, min_p), 2);
}

// 4.12、多边形的最小矩形包围轮廓
void boxOfPolygon(const vector<Point> &polygon, Point &down_left, Point &up_right)
{
    double max_x = double(INT_MIN), min_x = double(INT_MAX);
    double max_y = double(INT_MIN), min_y = double(INT_MAX);

    for (auto c : polygon)
    {
        max_x = (c.x > max_x) ? c.x : max_x;
        min_x = (c.x < min_x) ? c.x : min_x;
        max_y = (c.y > max_y) ? c.y : max_y;
        min_y = (c.y < min_y) ? c.y : min_y;
    }

    down_left = Point(min_x, min_y);
    up_right = Point(max_x, max_y);
}

// 五、圆
// 5.1、点和圆的关系
// 0：圆内， 1：圆上， 2：圆外
int pointToCircle(const Point &c, double radius, const Point &p)
{
    double ptoc_d = distance(c, p);
    if (ptoc_d < radius)
        return 0;
    if (ptoc_d == radius)
        return 1;
    return 2;
}

// 5.2、直线和圆的关系
// 0：相交， 1：相切， 2：相离
int lineToCircle(const Point &c, double radius, const Line &l)
{
    double ctol_d = ptolDistance(c, l);
    if (ctol_d < radius)
        return 0;
    if (ctol_d == radius)
        return 1;
    return 2;
}

// 5.3、线段和圆的关系
// 0：圆内， 1：相交，  2：圆外
int segToCircle(const Point &c, double radius, const Line &l)
{
    int flag = pointToCircle(c, radius, l.s) + pointToCircle(c, radius, l.e);
    if (flag == 0) // 0+0，两个端点都在圆内
        return 0;
    if (flag < 4)
        return 1; // 圆上至少一个端点

    // 2+2，两个端点都在圆外
    int distance = ptolDistance(c, l);
    if (distance <= radius)
    {
        Point project_p = ptolProjection(c, l); // 点在线上的投影点
        if (isponl(project_p, l))               // 投影点位于线段上
            return 1;
    }
    return 2;
}

// 5.4、两圆之间的关系
// 0：内含， 1：内切， 2：相交， 3：外切， 4：外离
int circleToCircle(const Point &c1, double r1, const Point &c2, double r2)
{
    double ctoc_d = distance(c1, c2);
    double min_r = abs(r1 - r2);
    if (ctoc_d < min_r)
        return 0;
    if (ctoc_d == min_r)
        return 1;

    double max_r = r1 + r2;
    if (ctoc_d < max_r)
        return 2;
    if (ctoc_d == max_r)
        return 3;
    // if (ctoc_d > max_r)
    return 4;
}

int main()
{
    return 0;
}