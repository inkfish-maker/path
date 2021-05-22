#ifndef POINT_H
#define POINT_H

#include "config.h"

#define PointType MyPoint
#define DataType double

using namespace cv;

class MyPoint : public Point2d
{
public:
    int index;
    MyPoint() : Point2d() { index = 0; };
    template <typename T>
    MyPoint(T x, T y) : Point2d(x, y) { index = 0; };

    // MyPoint operator=(const MyPoint &p);
    MyPoint operator-(const MyPoint &p);
    MyPoint operator+(const MyPoint &p);
};
// MyPoint MyPoint::operator=(const MyPoint &p)
// {
//     x = p.x;
//     y = p.y;
//     index = p.index;
//     return *this;
// }
MyPoint MyPoint::operator-(const MyPoint &p)
{
    MyPoint diff(x - p.x, y - p.y);
    return diff;
}
MyPoint MyPoint::operator+(const MyPoint &p)
{
    MyPoint diff(x + p.x, y - +p.y);
    return diff;
}

#endif