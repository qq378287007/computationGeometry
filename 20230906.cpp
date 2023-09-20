#include "20230906.h"

bool Line::equalLine(shared_ptr<Line> line) const
{
    shared_ptr<Arc> arc = dynamic_pointer_cast<Arc>(line);
    if (arc)
        return false;
    return (line->sp.equalPoint(sp) && line->ep.equalPoint(ep)) ||
           (line->ep.equalPoint(sp) && line->sp.equalPoint(ep));
}
void Line::intersect(shared_ptr<Line> line)
{
    auto arc = dynamic_pointer_cast<Arc>(line);
    if (arc == nullptr) // 直线与直线交点
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
                double x = x0[left_ind];
                double y = y0[left_ind];

                if (!(equal(x2, x) && equal(y2, y)) && !(equal(x4, x) && equal(y4, y)))
                {
                    addPoint(x, y);
                    line->addPoint(x, y);
                }

                int right_ind = x2 < x4 ? 1 : 3;
                if (!equal(x0[left_ind], x0[right_ind]))
                {
                    x = x0[right_ind];
                    y = y0[right_ind];
                    if (!(equal(x2, x) && equal(y2, y)) && !(equal(x4, x) && equal(y4, y)))
                    {
                        addPoint(x, y);
                        line->addPoint(x, y);
                    }
                }
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
            if (!(equal(x2, x) && equal(y2, y)) && !(equal(x4, x) && equal(y4, y)))
            {
                addPoint(x, y);
                line->addPoint(x, y);
            }
        }
    }
    else // 直线与圆弧交点
    {
        double x1 = sp.x;
        double y1 = sp.y;
        double x2 = ep.x;
        double y2 = ep.y;

        double x = arc->cp.x;
        double y = arc->cp.y;
        double r = arc->r;

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

        if (r == pb_length) // 圆与直线相切，一个交点，交点为b
        {
            // 交点b位于线段p1p2间
            // 交点b位于圆弧区间
            if ((b_x - x1) * (b_x - x2) <= 0.0 && arc->containPoint(b_x, b_y))
            {
                if (!ep.equalP(b_x, b_y) && !arc->ep.equalP(b_x, b_y))
                {
                    addPoint(b_x, b_y);
                    line->addPoint(b_x, b_y);
                }
            }
        }
        else if (r > pb_length) // 圆与直线相交，两个交点
        {
            // b与交点的距离
            double b_d = sqrt(r * r - pb_length * pb_length);

            // 交点bl坐标
            double bl_x = b_x + p1p2_x * b_d;
            double bl_y = b_y + p1p2_y * b_d;
            if ((bl_x - x1) * (bl_x - x2) <= 0.0 && arc->containPoint(bl_x, bl_y))
            {
                if (!ep.equalP(bl_x, bl_y) && !arc->ep.equalP(bl_x, bl_y))
                {
                    addPoint(bl_x, bl_y);
                    line->addPoint(bl_x, bl_y);
                }
            }

            // 交点br坐标
            double br_x = b_x - p1p2_x * b_d;
            double br_y = b_y - p1p2_y * b_d;
            if ((br_x - x1) * (br_x - x2) <= 0.0 && arc->containPoint(br_x, br_y))
            {
                if (!ep.equalP(br_x, br_y) && !arc->ep.equalP(br_x, br_y))
                {
                    addPoint(br_x, br_y);
                    line->addPoint(br_x, br_y);
                }
            }
        }
    }
}
shared_ptr<Path> Line::toPath() const
{
    const int n = ip.size() + 1;
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
shared_ptr<Line> Line::linkLine(shared_ptr<Line> next) const
{
    if (!ep.equalPoint(next->sp))
        return nullptr;
    if (!equal(sp.angle(ep), next->sp.angle(next->ep)))
        return nullptr;
    if (dynamic_pointer_cast<Arc>(next))
        return nullptr;
    return make_shared<Line>(sp, next->ep);
}

bool Arc::equalLine(shared_ptr<Line> line) const
{
    shared_ptr<Arc> arc = dynamic_pointer_cast<Arc>(line);
    if (!arc)
        return false;
    return cp.equalPoint(arc->cp) &&
           ((sp.equalPoint(arc->sp) && ep.equalPoint(arc->ep) && anticlockwise == arc->anticlockwise) ||
            (sp.equalPoint(arc->ep) && ep.equalPoint(arc->sp) && !anticlockwise == arc->anticlockwise));
    return false;
}
void Arc::intersect(shared_ptr<Line> line)
{
    auto arc = dynamic_pointer_cast<Arc>(line);
    if (arc == nullptr) // 圆弧与直线交点
    {
        double x1 = line->sp.x;
        double y1 = line->sp.y;
        double x2 = line->ep.x;
        double y2 = line->ep.y;

        double x = cp.x;
        double y = cp.y;

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

        if (r == pb_length) // 圆与直线相切，一个交点，交点为b
        {
            // 交点b位于线段p1p2间
            // 交点b位于圆弧区间
            if ((b_x - x1) * (b_x - x2) <= 0.0 && containPoint(b_x, b_y))
            {
                if (!ep.equalP(b_x, b_y) && !line->ep.equalP(b_x, b_y))
                {
                    addPoint(b_x, b_y);
                    line->addPoint(b_x, b_y);
                }
            }
        }
        else if (r > pb_length) // 圆与直线相交，两个交点
        {
            // b与交点的距离
            double b_d = sqrt(r * r - pb_length * pb_length);

            // 交点bl坐标
            double bl_x = b_x + p1p2_x * b_d;
            double bl_y = b_y + p1p2_y * b_d;
            if ((bl_x - x1) * (bl_x - x2) <= 0.0 && containPoint(bl_x, bl_y))
            {
                if (!ep.equalP(bl_x, bl_y) && !line->ep.equalP(bl_x, bl_y))
                {
                    addPoint(bl_x, bl_y);
                    line->addPoint(bl_x, bl_y);
                }
            }

            // 交点br坐标
            double br_x = b_x - p1p2_x * b_d;
            double br_y = b_y - p1p2_y * b_d;
            if ((br_x - x1) * (br_x - x2) <= 0.0 && containPoint(br_x, br_y))
            {
                if (!ep.equalP(br_x, br_y) && !line->ep.equalP(br_x, br_y))
                {
                    addPoint(br_x, br_y);
                    line->addPoint(br_x, br_y);
                }
            }
        }
    }
    else // 圆弧与圆弧交点
    {
        double p1_x = this->cp.x;
        double p1_y = this->cp.y;
        double r1 = this->r;
        double p2_x = arc->cp.x;
        double p2_y = arc->cp.y;
        double r2 = arc->r;

        if (r1 > r2)
        {
            double tmp;
            tmp = p1_x, p1_x = p2_x, p2_x = tmp;
            tmp = p1_y, p1_y = p2_y, p2_y = tmp;
            tmp = r1, r1 = r2, r2 = tmp;
        }

        double min_d = r2 - r1;
        double max_d = r1 + r2;
        double d = sqrt(pow(p1_x - p2_x, 2) + pow(p1_y - p2_y, 2));

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
            if (this->containPoint(x, y) && arc->containPoint(x, y))
            {
                if (!ep.equalP(x, y) && !arc->ep.equalP(x, y))
                {
                    addPoint(x, y);
                    line->addPoint(x, y);
                }
            }
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
            if (this->containPoint(x1, y1) && arc->containPoint(x1, y1))
            {
                if (!ep.equalP(x1, y1) && !arc->ep.equalP(x1, y1))
                {
                    addPoint(x1, y1);
                    line->addPoint(x1, y1);
                }
            }

            if (this->containPoint(x2, y2) && arc->containPoint(x2, y2))
            {
                if (!ep.equalP(x2, y2) && !arc->ep.equalP(x2, y2))
                {
                    addPoint(x2, y2);
                    line->addPoint(x2, y2);
                }
            }
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
                if (this->containPoint(x, y) && arc->containPoint(x, y))
                {
                    if (!ep.equalP(x, y) && !arc->ep.equalP(x, y))
                    {
                        addPoint(x, y);
                        line->addPoint(x, y);
                    }
                }
            }
            else // 重叠
            {
                double x, y;

                x = arc->sp.x;
                y = arc->sp.y;
                if (this->containPoint(x, y))
                {
                    if (!ep.equalP(x, y) && !arc->ep.equalP(x, y))
                    {
                        addPoint(x, y);
                        line->addPoint(x, y);
                    }
                }

                x = arc->ep.x;
                y = arc->ep.y;
                if (this->containPoint(x, y))
                {
                    if (!ep.equalP(x, y) && !arc->ep.equalP(x, y))
                    {
                        addPoint(x, y);
                        line->addPoint(x, y);
                    }
                }

                x = this->sp.x;
                y = this->sp.y;
                if (arc->containPoint(x, y))
                {
                    if (!ep.equalP(x, y) && !arc->ep.equalP(x, y))
                    {
                        addPoint(x, y);
                        line->addPoint(x, y);
                    }
                }

                x = this->ep.x;
                y = this->ep.y;
                if (arc->containPoint(x, y))
                {
                    if (!ep.equalP(x, y) && !arc->ep.equalP(x, y))
                    {
                        addPoint(x, y);
                        line->addPoint(x, y);
                    }
                }
            }
        }
    }
}
shared_ptr<Path> Arc::toPath() const
{
    const int n = ip.size() + 1;
    vector<Point> vp;
    vp.reserve(n + 1);
    vp.emplace_back(sp);
    vp.insert(vp.cend(), ip.cbegin(), ip.cend());
    vp.emplace_back(ep);

    auto path = make_shared<Path>();
    for (int i = 0; i < n; i++)
        path->addLine(make_shared<Arc>(vp[i], vp[i + 1], cp, anticlockwise));
    return path;
}
shared_ptr<Line> Arc::linkLine(shared_ptr<Line> next) const
{
    auto arc = dynamic_pointer_cast<Arc>(next);
    if (arc == nullptr)
        return nullptr;

    if (!ep.equalP(arc->sp.x, arc->sp.y))
        return nullptr;

    if (!cp.equalP(arc->cp.x, arc->cp.y))
        return nullptr;
    if (!equal(r, arc->r))
        return nullptr;
    if (anticlockwise != arc->anticlockwise)
        return nullptr;

    return make_shared<Arc>(sp, next->ep, cp, anticlockwise);
}

shared_ptr<Path> Path::toPath() const
{
    auto path = make_shared<Path>();
    for (const auto &line : lines)
        path->addPath(line->toPath());
    return path;
}
shared_ptr<Path> Path::toLinkPath() const
{
    auto path = make_shared<Path>();
    int i = 0;
    while (i < size())
    {
        auto tmp_line = lines[i];
        int j = i + 1;
        while (j < size())
        {
            auto tmp_line2 = tmp_line->linkLine(lines[j]);
            if (tmp_line2 == nullptr)
                break;
            tmp_line = tmp_line2;
            j++;
        }
        path->addLine(tmp_line);
        i = j;
    }

    int n = path->size();
    if (n >= 2)
    {
        auto tmp_line = path->lines[n - 1]->linkLine(path->lines[0]);
        if (tmp_line)
        {
            path->lines[0] = tmp_line;
            path->lines.pop_back();
        }
    }
    return path;
}

void setLineType(shared_ptr<Path> left, shared_ptr<Path> right)
{
    // 起点类型
    const int n_left = left->size();
    for (int i = 0; i < n_left; i++)
    {
        if (left->lines[i]->sp.pos == UNKNOW)
        {
            int last = 0;
            while (left->lines[(i + last - 1 + n_left) % n_left]->sp.pos == UNKNOW)
                last--;
            int next = 0;
            while (left->lines[(i + next + 1) % n_left]->sp.pos == UNKNOW)
                next++;

            Position pos = right->pointPosition(left->lines[i]->sp);
            for (int j = last; j <= next; j++)
                left->lines[(i + j + n_left) % n_left]->sp.pos = pos;
        }
    }
    // 片段类型
    for (int i = 0; i < n_left; i++)
    {
        Point cur_p = left->lines[i]->sp;
        if (cur_p.pos == IN || cur_p.pos == OUT)
        {
            left->lines[i]->pos = cur_p.pos;
            continue;
        }

        Point next_p = left->lines[(i + 1) % n_left]->sp;
        if (next_p.pos == IN || next_p.pos == OUT)
        {
            left->lines[i]->pos = next_p.pos;
            continue;
        }

        Point mid_p = left->lines[i]->centerPoint();
        Position pos = right->pointPosition(mid_p);
        left->lines[i]->pos = pos;
    }

    // 起点类型
    const int n_right = right->size();
    for (int i = 0; i < n_right; i++)
    {
        if (right->lines[i]->sp.pos == UNKNOW)
        {
            int last = 0;
            while (right->lines[(i + last - 1 + n_right) % n_right]->sp.pos == UNKNOW)
                last--;
            int next = 0;
            while (right->lines[(i + next + 1) % n_right]->sp.pos == UNKNOW)
                next++;

            Position pos = left->pointPosition(right->lines[i]->sp);
            for (int j = last; j <= next; j++)
                right->lines[(i + j + n_right) % n_right]->sp.pos = pos;
        }
    }
    // 片段类型
    for (int i = 0; i < n_right; i++)
    {
        Point cur_p = right->lines[i]->sp;
        if (cur_p.pos == IN || cur_p.pos == OUT)
        {
            right->lines[i]->pos = cur_p.pos;
            continue;
        }

        Point next_p = right->lines[(i + 1) % n_right]->sp;
        if (next_p.pos == IN || next_p.pos == OUT)
        {
            right->lines[i]->pos = next_p.pos;
            continue;
        }

        Point mid_p = right->lines[i]->centerPoint();
        Position pos = left->pointPosition(mid_p);
        right->lines[i]->pos = pos;
    }
}

void boolOperation(shared_ptr<Path> left, shared_ptr<Path> right)
{
    const int n_left = left->size();
    const int n_rigiht = right->size();
    for (int i = 0; i < n_left; i++)
        for (int j = 0; j < n_rigiht; j++)
            left->lines[i]->intersect(right->lines[j]);

    left = left->toPath();
    right = right->toPath();
    setLineType(left, right);
}

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

vector<shared_ptr<Path>> boolOperationIntersect(shared_ptr<Path> left, shared_ptr<Path> right)
{
    vector<shared_ptr<Path>> left_in = left->getVectorPath(IN);
    vector<shared_ptr<Path>> right_in = right->getVectorPath(IN);
    vector<shared_ptr<Path>> vp;
    vp.insert(vp.cend(), left_in.cbegin(), left_in.cend());
    vp.insert(vp.cend(), right_in.cbegin(), right_in.cend());
    return coverPath(vp);
}

vector<shared_ptr<Path>> boolOperationUnion(shared_ptr<Path> left, shared_ptr<Path> right)
{
    vector<shared_ptr<Path>> left_in = left->getVectorPath(OUT);
    vector<shared_ptr<Path>> right_in = right->getVectorPath(OUT);
    vector<shared_ptr<Path>> vp;
    vp.insert(vp.cend(), left_in.cbegin(), left_in.cend());
    vp.insert(vp.cend(), right_in.cbegin(), right_in.cend());
    return coverPath(vp);
}

vector<shared_ptr<Path>> boolOperationDifference1(shared_ptr<Path> left, shared_ptr<Path> right)
{
    vector<shared_ptr<Path>> left_in = left->getVectorPath(OUT);
    vector<shared_ptr<Path>> right_in = right->getVectorPath(IN);
    for (int i = 0; i < right_in.size(); i++)
        right_in[i]->reverse();

    vector<shared_ptr<Path>> vp;
    vp.insert(vp.cend(), left_in.cbegin(), left_in.cend());
    vp.insert(vp.cend(), right_in.cbegin(), right_in.cend());
    return coverPath(vp);
}

vector<shared_ptr<Path>> boolOperationDifference2(shared_ptr<Path> left, shared_ptr<Path> right)
{
    vector<shared_ptr<Path>> vp1 = boolOperationDifference1(left, right);
    vector<shared_ptr<Path>> vp2 = boolOperationDifference1(right, left);
    vector<shared_ptr<Path>> vp;
    vp.insert(vp.cend(), vp1.cbegin(), vp1.cend());
    vp.insert(vp.cend(), vp2.cbegin(), vp2.cend());
    return coverPath(vp);
}

int main()
{
    return 0;
}
