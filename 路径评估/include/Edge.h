#ifndef EDGE_H
#define EDGE_H

#include "Point.h"

//边类
class Edge
{
public:
    PointType px, py;
    PointType mid_p;
    DataType edge_width;
    Edge();
    Edge(const PointType &ppx, const PointType &ppy);
    void getEdgeWidth();
    Edge operator=(const Edge &e);
    bool operator==(const Edge &e);
    bool operator!=(const Edge &e);
};
Edge ::Edge()
{
    px = PointType(0, 0);
    py = PointType(0, 0);
    mid_p = PointType(0, 0);
    edge_width = 0;
}
Edge::Edge(const PointType &ppx, const PointType &ppy)
{
    px = ppx;
    py = ppy;
    mid_p.x = (ppx.x + ppy.x) / 2;
    mid_p.y = (ppx.y + ppy.y) / 2;
    edge_width = 0;
    // edge_width = sqrt(pow(fabs(ppx.x - ppy.x), 2) + pow(fabs(ppx.y - ppy.y), 2));
}
void Edge::getEdgeWidth()
{
    edge_width = sqrt(pow(fabs(px.x - py.x), 2) + pow(fabs(px.y - py.y), 2));
}
Edge Edge::operator=(const Edge &e)
{
    px = e.px;
    py = e.py;
    mid_p = e.mid_p;
}
bool Edge::operator==(const Edge &e)
{
    if (px == e.px && py == e.py || px == e.py && py == e.px)
        return true;
    return false;
}
bool Edge::operator!=(const Edge &e)
{
    return *this == e;
}
ostream &operator<<(ostream &out, Edge &e)
{
    std::cout << "px:" << e.px.x << "  " << e.px.y << "py:" << e.py.x << " " << e.py.y << endl;
}
#endif