#ifndef PATH_H
#define PATH_H

#include "Triangle.h"

//路径深度
int mindepth = 8;
int maxdepth = 12;
//路径
vector<PointType> path;
vector<vector<PointType>> paths;
vector<pair<int, Edge>> nextinfo;
vector<vector<pair<int, Edge>>> nextinfos;
//路径宽度（即点所在边的长度）
vector<DataType> width;
vector<vector<DataType>> widths;

//生成邻接图
void ConnectTri(const vector<Triangle> &triangles_);
//DFS搜索备选路径
void DFS_Path(int start, int p1_index, int p2_index);
//重置变量，为下一次规划做准备
void ClearPath();
//找到DFS的起始点
vector<int> FindStart(const PointType &start_pose, const PointType &start_orientation);
//判断一个点是否在三角形内
bool PointinTriangle(const Triangle &tri, const PointType &p);
//从点集到最佳路径
void process(const PointType &start_point);

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
        vector<int> e1 = adj_edges[triangles_[i].e1.px.index][triangles_[i].e1.py.index].AdjTriIndex;
        for (int j = 0; j < e1.size(); j++)
        {
            if (e1[j] != i)
            {
                pair<int, Edge> p;
                p.first = e1[j];
                p.second = triangles_[i].e1;
                adj_tirs[i].AdjTriIndex_Edge.push_back(p);
            }
        }

        vector<int> e2 = adj_edges[triangles_[i].e2.px.index][triangles_[i].e2.py.index].AdjTriIndex;
        for (int j = 0; j < e2.size(); j++)
        {
            if (e2[j] != i)
            {
                pair<int, Edge> p;
                p.first = e2[j];
                p.second = triangles_[i].e2;
                adj_tirs[i].AdjTriIndex_Edge.push_back(p);
            }
        }
        vector<int> e3 = adj_edges[triangles_[i].e3.px.index][triangles_[i].e3.py.index].AdjTriIndex;
        for (int j = 0; j < e3.size(); j++)
        {
            if (e3[j] != i)
            {
                pair<int, Edge> p;
                p.first = e3[j];
                p.second = triangles_[i].e3;
                adj_tirs[i].AdjTriIndex_Edge.push_back(p);
            }
        }
    }
}
//tir序号，边的两个端点序号
//可以使用分支界限优化效率（待实现）
bool dfs_flag = true;
void DFS_Path(int start, int p1_index, int p2_index)
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
                if (nowedge == nextedge)
                {
                    dfs_flag = false;
                }
                DFS_Path(next_tri, adj_tirs[start].AdjTriIndex_Edge[i].second.px.index, adj_tirs[start].AdjTriIndex_Edge[i].second.py.index);
                path.pop_back();
                width.pop_back();
                nextinfo.pop_back();
            }
        }
        triangles[start].visit = false;
    }
}

vector<int> FindStart(const PointType &start_pose, const PointType &start_orientation)
{
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
    if (tri_index != -1)
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
        float dot1 = v1.dot(v);
        float dot2 = v2.dot(v);
        float dot3 = v3.dot(v);
        if (dot1 >= 0 && dot2 >= 0)
        {
            start.push_back(triangles[tri_index].p1.index);
            start.push_back(triangles[tri_index].p2.index);
        }
        else if (dot1 >= 0 && dot3 >= 0)
        {
            start.push_back(triangles[tri_index].p1.index);
            start.push_back(triangles[tri_index].p3.index);
        }
        else
        {
            start.push_back(triangles[tri_index].p2.index);
            start.push_back(triangles[tri_index].p3.index);
        }
        return start;
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

void process(const PointType &start_point, const PointType &start_orientation)
{
    triangles.resize(0);
    initDelaunay();
    Delaunay();
    ClearPath();
    ConnectTri(triangles);
    //解决起点接口的问题
    vector<int> start = FindStart(start_point, start_orientation);
    DFS_Path(start[0], start[1], start[2]);
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
}

#endif