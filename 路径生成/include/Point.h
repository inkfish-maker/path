#ifndef POINT_H
#define POINT_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class MyPoint : public Point2d
{
public:
    int index;
    MyPoint() : Point2d(){};
    template <typename T>
    MyPoint(T x, T y) : Point2d(x, y){};

    MyPoint operator=(const MyPoint &p);
    // MyPoint(int x, int y) : Point2d(x, y){};
    // MyPoint(float x, float y) : Point2d(x, y){};
    // MyPoint(double x, double y) : Point2d(x, y){};
};
MyPoint MyPoint::operator=(const MyPoint &p)
{
    x = p.x;
    y = p.y;
    index = p.index;
}

#endif