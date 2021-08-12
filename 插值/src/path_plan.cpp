#include "Edge.h"
#include "Triangle.h"
#include "Delaunay.h"
#include "Route.h"
#include "Assessment.h"

Point p;
void on_mouse(int event, int x, int y, int flags, void *ustc);
int main()
{
    board = Mat(Mat_len, Mat_width, CV_8UC3, Scalar(0, 0, 0));
    namedWindow("main");
    srand((unsigned)time(NULL));
    read_points();

    //测试点在哪个三角形里
    cv::setMouseCallback("main", on_mouse, 0); //调用回调函数
    cv::imshow("main", board);
    while (true)
    {
        char key = cvWaitKey(10);
        if (key == 's')
        {
            break;
        }
    }

    SetWeight(1, 1, 1, 0, 0, 1);
    PointType pose(p.x, p.y);
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