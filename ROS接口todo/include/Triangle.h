#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "Edge.h"

//x升序
bool cmp_x(PointType p1, PointType p2);
//x降序
bool cmp_x_down(PointType p1, PointType p2);
//y升序
bool cmp_y(PointType p1, PointType p2);
//判等
bool cmp_edge(Edge e1, Edge e2);

//三角类
class Triangle
{
public:
    //顶点
    PointType p1, p2, p3;
    Edge e1, e2, e3;
    //边长
    DataType a, b, c;
    //内接圆圆心，半径
    PointType centre;
    DataType r;
    //是否访问过
    bool visit;

    Triangle(const PointType &pp1, const PointType &pp2, const PointType &pp3);
    Triangle(const PointType &p, const Edge &e);
    Triangle(){};
    //求面积
    DataType Area();
    //初始化内接圆
    void InitCircle();
    bool operator==(Triangle const &tri);
};

Triangle::Triangle(const PointType &pp1, const PointType &pp2, const PointType &pp3)
{
    //pp1,pp2,pp3按x升序
    // vector<PointType> ord;
    // ord.push_back(pp1);
    // ord.push_back(pp2);
    // ord.push_back(pp3);
    // std::sort(ord.begin(), ord.end(), cmp_x);

    // p1 = ord[0];
    // p2 = ord[1];
    // p3 = ord[2];
    p1 = pp1;
    p2 = pp2;
    p3 = pp3;
    e1 = Edge(pp1, pp2);
    e2 = Edge(pp1, pp3);
    e3 = Edge(pp2, pp3);
    InitCircle();
}
Triangle::Triangle(const PointType &p, const Edge &e)
{
    //pp1,pp2,pp3按x升序
    // vector<PointType> ord;
    // ord.push_back(p);
    // ord.push_back(e.px);
    // ord.push_back(e.py);
    // std::sort(ord.begin(), ord.end(), cmp_x);

    // p1 = ord[0];
    // p2 = ord[1];
    // p3 = ord[2];
    // e1 = Edge(p1, p2);
    // e2 = Edge(p1, p3);
    // e3 = Edge(p2, p3);
    p1 = p;
    p2 = e.px;
    p3 = e.py;
    e1 = Edge(p, e.px);
    e2 = Edge(p, e.py);
    e3 = Edge(e.px, e.py);
    InitCircle();
}
DataType Triangle::Area()
{
    //假如有三点为（X1，y1）（X2，y2）（X3，y3）则这三点围成的三角形面积S = 1 / 2×丨（Ⅹ3 - X2）(y1 - y2) - (X1 - Ⅹ2)(y3 - y2) 丨
    DataType x1, y1, x2, y2, x3, y3;
    x1 = p1.x;
    y1 = p1.y;
    x2 = p2.x;
    y2 = p2.y;
    x3 = p3.x;
    y3 = p3.y;
    return 0.5 * fabs((x3 - x2) * (y1 - y2) - (x1 - x2) * (y3 - y2));
}
void Triangle::InitCircle()
{
    //三角形三点坐标
    DataType xa, ya, xb, yb, xc, yc;
    DataType c1, c2;
    a = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    b = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2));
    c = sqrt(pow(p2.x - p3.x, 2) + pow(p2.y - p3.y, 2));
    /*
    正弦定理a/sinA=b/sinB=c/sinC=2R,R为三角形外接圆半径
    三角形面积S=absinC/2=ab(c/2R)/2=abc/4R;
    */
    r = a * b * c / Area() / 4;
    xa = p1.x;
    ya = p1.y;
    xb = p2.x;
    yb = p2.y;
    xc = p3.x;
    yc = p3.y;
    c1 = (xa * xa + ya * ya - xb * xb - yb * yb) / 2;
    c2 = (xa * xa + ya * ya - xc * xc - yc * yc) / 2;
    //求外接圆圆心坐标
    //三角形外接圆是三条边上垂直平分线的交点
    centre.x = (c1 * (ya - yc) - c2 * (ya - yb)) / ((xa - xb) * (ya - yc) - (xa - xc) * (ya - yb));
    centre.y = (c1 * (xa - xc) - c2 * (xa - xb)) / ((ya - yb) * (xa - xc) - (ya - yc) * (xa - xb));
}

bool Triangle::operator==(Triangle const &tri)
{
    if (tri.p1 == p1 && tri.p2 == p2 && tri.p3 == p3)
        return true;
    return false;
}
//对点集进行排序
bool cmp_x(PointType p1, PointType p2)
{
    return p1.x < p2.x;
}
bool cmp_x_down(PointType p1, PointType p2)
{
    return p1.x > p2.x;
}
bool cmp_y(PointType p1, PointType p2)
{ /* condition */
    return p1.y < p2.y;
}
bool cmp_edge(Edge e1, Edge e2)
{
    if (e1 == e2)
        return true;
    return false;
}

//一些全局变量
//之后改写为类

//点集区域,点集左下角与右上角sort(points.begin(), points.end(), [](PointType p1, PointType p2) { return p1.index < p2.index; });
vector<PointType> region;
//列表顶点
vector<PointType> points;
//缓存边
vector<Edge> temp_edges;
//列表三角形（确认）
vector<Triangle> triangles;
//列表三角形（暂时）
queue<Triangle> temp_triangles;

//相对边的邻接三角形
struct AdjacencyEdge
{
    //存储邻接三角形的序号
    vector<int> AdjTriIndex;
};
vector<vector<AdjacencyEdge>> adj_edges;
//相对三角形的邻接三角形
struct AdjacencyTri
{
    //邻接三角形的序号与邻接的边
    vector<pair<int, Edge>> AdjTriIndex_Edge;
};
vector<AdjacencyTri> adj_tirs;

#endif