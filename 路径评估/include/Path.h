#ifndef PATH_H
#define PATH_H

#include "Triangle.h"

//路径深度
int depth = 5;
//路径
vector<PointType> path;
vector<vector<PointType>> paths;
//路径宽度（即点所在边的长度）
vector<DataType> width;
vector<vector<DataType>> widths;

//生成邻接图
void ConnectTri(const vector<Triangle> &triangles_);
//DFS搜索备选路径
void DFS_Path(int start);

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
        for (int j = 0; j < paths.size(); j++)
        {

            // cout << triangles_[i].e1.px.index << "    " << triangles_[i].e1.py.index << "    " << triangles_[i].e2.px.index << "    " << triangles_[i].e2.py.index << "    " << triangles_[i].e3.px.index << "    " << triangles_[i].e3.py.index << "    " << endl;
        }
    }
}
void DFS_Path(int start)
{
    if (path.size() >= depth)
    {
        paths.push_back(path);
        widths.push_back(width);
        return;
    }
    triangles[start].visit = true;
    for (int i = 0; i < adj_tirs[start].AdjTriIndex_Edge.size(); i++)
    {
        int next_tri = adj_tirs[start].AdjTriIndex_Edge[i].first;
        if (triangles[next_tri].visit == false)
        {
            path.push_back(adj_tirs[start].AdjTriIndex_Edge[i].second.mid_p);
            width.push_back(adj_tirs[start].AdjTriIndex_Edge[i].second.edge_width);
            DFS_Path(next_tri);
            path.pop_back();
            width.pop_back();
        }
    }
}

#endif