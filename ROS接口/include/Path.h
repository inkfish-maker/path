#ifndef PATH_H
#define PATH_H

#include "Triangle.h"

//路径深度
int mindepth = 8;
int maxdepth = 10;
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
void DFS_Path(int start, int p1_index, int p2_index)
{
    //中点加入path
    Edge nowedge(points[p1_index], points[p2_index]);
    path.push_back(nowedge.mid_p);
    width.push_back(nowedge.edge_width);
    nextinfo.push_back(pair<int, Edge>(start, nowedge));
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
                    path.pop_back();
                    width.pop_back();
                    nextinfo.pop_back();
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