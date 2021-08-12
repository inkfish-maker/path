#ifndef ROUTE_H
#define ROUTE_H

#include "Triangle.h"

//路径深度
int mindepth = 10;
int maxdepth = 12;
//路径宽度
int minwidth = 0;
int maxwidth = 200;
//路径
vector<PointType> path;
vector<vector<PointType>> paths;
vector<pair<int, Edge>> nextinfo;
vector<vector<pair<int, Edge>>> nextinfos;
//路径宽度（即点所在边的长度）
vector<DataType> width;
vector<vector<DataType>> widths;
//上一条（已确认的）路径
vector<PointType> last_path;

//生成邻接图
void ConnectTri(const vector<Triangle> &triangles_);
//DFS搜索备选路径
void DFS_Path(int start, int p1_index, int p2_index, bool dfs_flag);
//重置变量，为下一次规划做准备
void ClearPath();

//判断一个点是否在三角形内
bool PointinTriangle(const Triangle &tri, const PointType &p);
//如果起始点在三角形内，可通过此函数找到DFS的接口参数
vector<int> FindStart_intri(const PointType &start_pose, const PointType &start_orientation);
//如果起始点在三角形外，可通过此函数找到DFS的接口参数
vector<int> FindStart_outtri(const PointType &start_pose, const PointType &start_orientation);

//计算两点之间距离
DataType Dist(const PointType &p1, const PointType &p2)
{
    return sqrt(pow(fabs(p1.x - p2.x), 2) + pow(fabs(p1.y - p2.y), 2));
}
//计算两点之间距离的平方
DataType Dist2(const PointType &p1, const PointType &p2)
{
    return pow(fabs(p1.x - p2.x), 2) + pow(fabs(p1.y - p2.y), 2);
}
//计算角度，返回值为弧度制
//取值范围[-pi,pi],右转+，左转-
DataType GetAngle(const PointType &p1, const PointType &p2, const PointType &p3)
{
    DataType angle = atan2(p2.y - p1.y, p2.x - p1.x) - atan2(p3.y - p2.y, p3.x - p2.x);
    if (angle < -M_PI)
        angle += 2 * M_PI;
    else if (angle > M_PI)
        angle -= 2 * M_PI;
    // cout << angle / M_PI * 180 << endl;
    return angle;
}

void ConnectTri(const vector<Triangle> &triangles_)
{
    adj_edges.resize(point_num);
    for (int i = 0; i < adj_edges.size(); i++)
        adj_edges[i].resize(point_num);
    adj_tirs.resize(triangles_.size());

    for (int i = 0; i < triangles_.size(); i++)
    {
        adj_edges[triangles_[i].p1.index][triangles_[i].p2.index].AdjTriIndex.push_back(i);
        adj_edges[triangles_[i].p2.index][triangles_[i].p1.index].AdjTriIndex.push_back(i);

        adj_edges[triangles_[i].p1.index][triangles_[i].p3.index].AdjTriIndex.push_back(i);
        adj_edges[triangles_[i].p3.index][triangles_[i].p1.index].AdjTriIndex.push_back(i);

        adj_edges[triangles_[i].p2.index][triangles_[i].p3.index].AdjTriIndex.push_back(i);
        adj_edges[triangles_[i].p3.index][triangles_[i].p2.index].AdjTriIndex.push_back(i);
    }
    for (int i = 0; i < triangles_.size(); i++)
    {
        vector<int> e1_ = adj_edges[triangles_[i].e1.px.index][triangles_[i].e1.py.index].AdjTriIndex;
        for (int j = 0; j < e1_.size(); j++)
        {
            if (e1_[j] != i)
            {
                pair<int, Edge> p;
                p.first = e1_[j];
                p.second = triangles_[i].e1;
                adj_tirs[i].AdjTriIndex_Edge.push_back(p);
            }
        }

        vector<int> e2_ = adj_edges[triangles_[i].e2.px.index][triangles_[i].e2.py.index].AdjTriIndex;
        for (int j = 0; j < e2_.size(); j++)
        {
            if (e2_[j] != i)
            {
                pair<int, Edge> p;
                p.first = e2_[j];
                p.second = triangles_[i].e2;
                adj_tirs[i].AdjTriIndex_Edge.push_back(p);
            }
        }
        vector<int> e3_ = adj_edges[triangles_[i].e3.px.index][triangles_[i].e3.py.index].AdjTriIndex;
        for (int j = 0; j < e3_.size(); j++)
        {
            if (e3_[j] != i)
            {
                pair<int, Edge> p;
                p.first = e3_[j];
                p.second = triangles_[i].e3;
                adj_tirs[i].AdjTriIndex_Edge.push_back(p);
            }
        }

        if (i == 150)
        {
            cout << "i=150:" << e1_.size() << " " << e2_.size() << " " << e3_.size() << endl;
        }
    }
    cout << "ConnectTri successfully" << endl;
}
//tir序号，边的两个端点序号
//可以使用分支界限优化效率（待实现）
void DFS_Path(int start, int p1_index, int p2_index, bool dfs_flag)
{
    //中点加入path
    Edge nowedge(points[p1_index], points[p2_index]);
    if (dfs_flag)
    {
        path.push_back(nowedge.mid_p);
        width.push_back(nowedge.edge_width);
        nextinfo.push_back(pair<int, Edge>(start, nowedge));
    }
    else
    {
        dfs_flag = true;
    }
    //path.size()
    if (path.size() >= mindepth)
    {
        paths.push_back(path);
        widths.push_back(width);
        nextinfos.push_back(nextinfo);
        return;
    }
    if (path.size() <= maxdepth)
    {
        triangles[start].visit = true;
        for (int i = 0; i < adj_tirs[start].AdjTriIndex_Edge.size(); i++)
        {
            int next_tri = adj_tirs[start].AdjTriIndex_Edge[i].first;
            if (triangles[next_tri].visit == false)
            {
                // path.push_back(adj_tirs[start].AdjTriIndex_Edge[i].second.mid_p);
                // adj_tirs[start].AdjTriIndex_Edge[i].second.getEdgeWidth();
                // width.push_back(adj_tirs[start].AdjTriIndex_Edge[i].second.edge_width);
                Edge nextedge(adj_tirs[start].AdjTriIndex_Edge[i].second.px, adj_tirs[start].AdjTriIndex_Edge[i].second.py);
                //如果启用限制，自制的地图可能由于只有极端值可行，而导致paths的size为0出现段错误
                // if (nextedge.edge_width > maxwidth || nextedge.edge_width < minwidth)
                //     continue;
                // if (path.size() > 3)
                // {
                //     DataType angle = GetAngle(path[path.size() - 2], path[path.size() - 1], nextedge.mid_p);
                //     if (angle > M_PI_2 || angle < -M_PI_2)
                //         continue;
                // }
                if (nowedge == nextedge)
                {
                    dfs_flag = false;
                }
                DFS_Path(next_tri, adj_tirs[start].AdjTriIndex_Edge[i].second.px.index, adj_tirs[start].AdjTriIndex_Edge[i].second.py.index, dfs_flag);
                if (dfs_flag)
                {
                    path.pop_back();
                    width.pop_back();
                    nextinfo.pop_back();
                }
            }
        }
        triangles[start].visit = false;
    }
}

vector<int> FindStart_intri(const PointType &start_pose, const PointType &start_orientation)
{
    cout << "========FindStart========" << endl;
    circle(board, start_pose, 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);
    vector<int> start;
    //找点在哪个三角形
    int tri_index = -1;
    for (int i = 0; i < triangles.size(); i++)
    {
        if (PointinTriangle(triangles[i], start_pose))
        {
            tri_index = i;
            break;
        }
    }
    bool flag = true;
    if (tri_index == -1)
        flag = false;
    if (flag)
    {
        start.push_back(tri_index);
        line(board, triangles[tri_index].p1, triangles[tri_index].p2, Scalar(255, 255, 0));
        line(board, triangles[tri_index].p1, triangles[tri_index].p3, Scalar(255, 255, 0));
        line(board, triangles[tri_index].p2, triangles[tri_index].p3, Scalar(255, 255, 0));
        cv::imshow("main", board);
    }
    else
        cout << "not in triangles!" << endl;
    //判断应该往哪个方向走
    if (flag)
    {
        Point_<DataType> v = start_orientation;
        Point_<DataType> v1 = triangles[tri_index].p1 - start_pose;
        Point_<DataType> v2 = triangles[tri_index].p2 - start_pose;
        Point_<DataType> v3 = triangles[tri_index].p3 - start_pose;
        double angle = atan2(v.y, v.x);
        double angle1 = atan2(v1.y, v1.x);
        double angle2 = atan2(v2.y, v2.x);
        double angle3 = atan2(v3.y, v3.x);
        pair<int, double> p(0, angle);
        pair<int, double> p1(1, angle1);
        pair<int, double> p2(2, angle2);
        pair<int, double> p3(3, angle3);
        vector<pair<int, double>> angles = {p, p1, p2, p3};
        sort(angles.begin(), angles.end(), [](pair<int, double> p, pair<int, double> p2)
             { return p.second < p2.second; });
        vector<pair<int, double>> choose;
        for (int i = 0; i < angles.size(); i++)
        {
            if (angles[i].first == 0)
            {
                if (i == 0 || i == 2)
                {
                    choose.push_back(angles[1]);
                    choose.push_back(angles[3]);
                }
                else
                {
                    choose.push_back(angles[0]);
                    choose.push_back(angles[2]);
                }
            }
        }
        for (int i = 0; i < choose.size(); i++)
        {
            if (choose[i].first == 1)
            {
                start.push_back(triangles[tri_index].p1.index);
            }
            else if (choose[i].first == 2)
            {
                start.push_back(triangles[tri_index].p2.index);
            }
            else
            {
                start.push_back(triangles[tri_index].p3.index);
            }
        }
        path.push_back(start_pose);
        width.push_back(triangles[tri_index].r);

        // Mat temp2;
        // board.copyTo(temp2);
        // for (int i = 0; i < adj_tirs[tri_index].AdjTriIndex_Edge.size(); i++)
        // {
        //     int index = adj_tirs[tri_index].AdjTriIndex_Edge[i].first;

        //     line(temp2, triangles[index].p1, triangles[index].p2, Scalar(255, 255, 255));
        //     line(temp2, triangles[index].p1, triangles[index].p3, Scalar(255, 255, 255));
        //     line(temp2, triangles[index].p2, triangles[index].p3, Scalar(255, 255, 255));
        //     waitKey(0);
        //     cv::imshow("main", temp2);
        // }

        Edge next(points[start[1]], points[start[2]]);
        nextinfo.push_back(pair<int, Edge>(start[0], next));
        cout << "start:" << start[0] << "-" << start[1] << "-" << start[2] << endl;
        cout << "FindStart(in) successfully" << endl;
        return start;
    }
    else
    {
        return FindStart_outtri(start_pose, start_orientation);
    }
}

//算法逻辑
//遍历所有三角形，找到距离车最近的cnt个三角形外心
//遍历cnt个待选三角形中边的中点，找到距离车最近的中点（但该点所在边的宽度需要是设定范围内的合理值)
struct Find_tri
{
    int index;
    double distance;
};
struct cmp_tri
{
    bool operator()(Find_tri c1, Find_tri c2)
    {
        return c1.distance > c2.distance;
    }
};
vector<int> FindStart_outtri(const PointType &start_pose, const PointType &start_orientation)
{
    //搜距离车最近的几个外接圆圆心
    vector<Triangle> triangles_ = triangles;
    priority_queue<Find_tri, vector<Find_tri>, cmp_tri> dis; //大根堆
    int cnt = 10;
    vector<int> triangles_index;
    Find_tri temp;
    //找到最近的cnt个三角形编号
    for (int i = 0; i < triangles_.size(); i++)
    {
        if (cnt)
        {
            cnt--;
            temp.index = i;
            temp.distance = Dist(start_pose, triangles_[i].centre);
            dis.push(temp);
        }
        if (cnt <= 0 && Dist(start_pose, triangles_[i].centre) < dis.top().distance)
        {
            dis.pop();
            temp.index = i;
            temp.distance = Dist(start_pose, triangles_[i].centre);
            dis.push(temp);
        }
    }
    while (!dis.empty())
    {
        triangles_index.push_back(dis.top().index);
        dis.pop();
    }
    //在待选中寻找最近的边
    double MinDist = DBL_MAX;
    vector<int> best;
    for (int i = 0; i < triangles_index.size(); i++)
    {
        // cout << MinDist << endl;
        double dis_e1 = Dist(start_pose, triangles_[triangles_index[i]].e1.mid_p);
        double width_e1 = triangles_[triangles_index[i]].e1.edge_width;
        if (dis_e1 < MinDist && width_e1 > minwidth && width_e1 < maxwidth)
        {
            MinDist = dis_e1;
            best.resize(0);
            best.push_back(triangles_index[i]);
            best.push_back(triangles[triangles_index[i]].e1.px.index);
            best.push_back(triangles[triangles_index[i]].e1.py.index);
        }
        double dis_e2 = Dist(start_pose, triangles_[triangles_index[i]].e2.mid_p);
        double width_e2 = triangles_[triangles_index[i]].e2.edge_width;
        if (dis_e2 < MinDist && width_e2 > minwidth && width_e2 < maxwidth)
        {
            MinDist = dis_e2;
            best.resize(0);
            best.push_back(triangles_index[i]);
            best.push_back(triangles[triangles_index[i]].e2.px.index);
            best.push_back(triangles[triangles_index[i]].e2.py.index);
        }
        double dis_e3 = Dist(start_pose, triangles_[triangles_index[i]].e3.mid_p);
        double width_e3 = triangles_[triangles_index[i]].e3.edge_width;
        if (dis_e3 < MinDist && width_e3 > minwidth && width_e3 < maxwidth)
        {
            MinDist = dis_e3;
            best.resize(0);
            best.push_back(triangles_index[i]);
            best.push_back(triangles[triangles_index[i]].e3.px.index);
            best.push_back(triangles[triangles_index[i]].e3.py.index);
        }
    }
    if (best.size() != 0)
    {
        cout << "start:" << best[0] << "-" << best[1] << "-" << best[2] << endl;
        cout << "FindStart(out) successfully" << endl;
        return best;
    }
    else
    {
        cout << "can't find possible start point!" << endl;
        exit(0);
    }
}

// Determine whether point P in triangle ABC
//重心法,来源https://blog.csdn.net/wkl115211/article/details/80215421
bool PointinTriangle(const Triangle &tri, const PointType &p)
{
    Point_<DataType> v0 = tri.p3 - tri.p1;
    Point_<DataType> v1 = tri.p2 - tri.p1;
    Point_<DataType> v2 = p - tri.p1;

    float dot00 = v0.dot(v0);
    float dot01 = v0.dot(v1);
    float dot02 = v0.dot(v2);
    float dot11 = v1.dot(v1);
    float dot12 = v1.dot(v2);

    float inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

    float u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
    if (u < 0 || u > 1) // if u out of range, return directly
    {
        return false;
    }

    float v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
    if (v < 0 || v > 1) // if v out of range, return directly
    {
        return false;
    }

    return u + v <= 1;
}

//重置变量，为下一次规划做准备
void ClearPath()
{
    path.resize(0);
    paths.resize(0);
    width.resize(0);
    widths.resize(0);
    nextinfo.resize(0);
    nextinfos.resize(0);
    cout << "ClearPath successfully" << endl;
}
#endif