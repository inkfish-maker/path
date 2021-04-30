#include "include/Edge.h"
#include "include/Triangle.h"
#include "include/Delaunay.h"
#include "include/Path.h"
#include "include/Assessment.h"

int main()
{
    board = Mat(Mat_len, Mat_width, CV_8UC3, Scalar(0, 0, 0));
    namedWindow("main");

    srand((unsigned)time(NULL));
    read_points();
    // create_points();

    initDelaunay();

    Delaunay();
    show_tri(triangles);
    ConnectTri(triangles);
    //搞清楚为什么point的序号会变
    sort(points.begin(), points.end(), [](PointType p1, PointType p2) { return p1.index < p2.index; });

    SetWeight(3, 3, 1);
    run();
}