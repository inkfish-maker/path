#include "include/Edge.h"
#include "include/Triangle.h"
#include "include/Delaunay.h"
#include "include/Path.h"
#include "include/Assessment.h"

Point p;
void on_mouse(int event, int x, int y, int flags, void *ustc);
int main()
{
    board = Mat(Mat_len, Mat_width, CV_8UC3, Scalar(0, 0, 0));
    namedWindow("main");
    srand((unsigned)time(NULL));
    read_points();
    // // create_points();
    // initDelaunay();
    // Delaunay();
    // show_tri(triangles);
    // ConnectTri(triangles);
    // //搞清楚为什么point的序号会变
    // sort(points.begin(), points.end(), [](PointType p1, PointType p2) { return p1.index < p2.index; });

    //测试点在哪个三角形里
    // cv::setMouseCallback("main", on_mouse, 0); //调用回调函数
    // while (true)
    // {
    //     cv::imshow("main", board);
    //     while (true)
    //     {
    //         char key = cvWaitKey(10);
    //         if (key == 's')
    //         {
    //             break;
    //         }
    //     }
    //     PointType p2(p.x, p.y);
    //     PointType p3(1, 0);
    //     FindStart(p2, p3);
    // }

    SetWeight(3, 3, 1);
    PointType pose(255, 74);
    PointType ori(1, 0);
    process(pose, ori);
}

void on_mouse(int event, int x, int y, int flags, void *ustc) //event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
    static cv::Point pre_pt = cv::Point(-1, -1); //初始坐标
    static cv::Point cur_pt = cv::Point(-1, -1); //实时坐标
    char temp[16];
    if (event == cv::EVENT_LBUTTONDOWN) //左键按下，读取初始坐标，并在图像上该点处划圆
    {
        pre_pt = cv::Point(x, y);
        cv::circle(board, pre_pt, 2, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0); //划圆
        cv::imshow("main", board);
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        p.x = x;
        p.y = y;
        cout << "x:" << x << "  "
             << "y:" << y << endl;
    }
}