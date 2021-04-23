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

    // Mat temp;
    // for (int j = 0; j < paths.size(); j++)
    // {
    //     board.copyTo(temp);
    //     circle(temp, Point2d(paths[j][0].x, paths[j][0].y), 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);
    //     for (int i = 0; i < paths[j].size() - 1; i++)
    //     {
    //         line(temp, Point2d(paths[j][i].x, paths[j][i].y), Point2d(paths[j][i + 1].x, paths[j][i + 1].y), Scalar(0, 255, 0));
    //     }
    //     // cout << "max_angle:" << get_MaximumAngleChange(j) << endl;
    //     // cout << "track_widthSD:" << get_TrackWidthStandardDeviation(j) << endl;
    //     // cout << "track_distanceSD:" << get_DistanceStandardDeviation(j) << endl;
    //     imshow("main", temp);
    //     waitKey(0);
    // }
    // SetWeight(1, 0, 0);
    // int j = BestPath();
    // board.copyTo(temp);
    // circle(temp, Point2d(paths[j][0].x, paths[j][0].y), 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);
    // for (int i = 0; i < paths[j].size() - 1; i++)
    // {
    //     line(temp, Point2d(paths[j][i].x, paths[j][i].y), Point2d(paths[j][i + 1].x, paths[j][i + 1].y), Scalar(255, 255, 255));
    // }
    // imshow("main", temp);
    // waitKey(0);

    SetWeight(2, 1, 1);
    run();
}