#ifndef PATH_H
#define PATH_H

#include "config.h"
#include "Triangle.h"

//路径深度
int depth = 5;
vector<PointType> path;
//可能的路径
vector<vector<PointType>> paths;

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
                pair<int, PointType> p;
                p.first = e1[j];
                p.second = triangles_[i].e1.mid_p;
                adj_tirs[i].AdjTriIndex_Point.push_back(p);
            }
        }

        vector<int> e2 = adj_edges[triangles_[i].e2.px.index][triangles_[i].e2.py.index].AdjTriIndex;
        for (int j = 0; j < e2.size(); j++)
        {
            if (e2[j] != i)
            {
                pair<int, PointType> p;
                p.first = e2[j];
                p.second = triangles_[i].e2.mid_p;
                adj_tirs[i].AdjTriIndex_Point.push_back(p);
            }
        }
        vector<int> e3 = adj_edges[triangles_[i].e3.px.index][triangles_[i].e3.py.index].AdjTriIndex;
        for (int j = 0; j < e3.size(); j++)
        {
            if (e3[j] != i)
            {
                pair<int, PointType> p;
                p.first = e3[j];
                p.second = triangles_[i].e3.mid_p;
                adj_tirs[i].AdjTriIndex_Point.push_back(p);
            }
        }

        // cout << triangles_[i].e1.px.index << "    " << triangles_[i].e1.py.index << "    " << triangles_[i].e2.px.index << "    " << triangles_[i].e2.py.index << "    " << triangles_[i].e3.px.index << "    " << triangles_[i].e3.py.index << "    " << endl;
    }
}
void DFS_Path(int start)
{
    if (path.size() >= 5)
    {
        paths.push_back(path);
        return;
    }
    triangles[start].visit = true;
    for (int i = 0; i < adj_tirs[start].AdjTriIndex_Point.size(); i++)
    {
        int next_tri = adj_tirs[start].AdjTriIndex_Point[i].first;
        if (triangles[next_tri].visit == false)
        {
            path.push_back(adj_tirs[start].AdjTriIndex_Point[i].second);
            depth++;
            DFS_Path(next_tri);
            path.pop_back();
        }
    }
}

#endif