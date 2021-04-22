//作者：吴迪
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
using namespace std;
using namespace cv;
cv::Mat org, dst, img, tmp;
std::vector<Point> points;
std::vector<int> colors;
int color = 1;
void on_mouse(int event, int x, int y, int flags, void *ustc) //event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
    static cv::Point pre_pt = cv::Point(-1, -1); //初始坐标
    static cv::Point cur_pt = cv::Point(-1, -1); //实时坐标
    char temp[16];
    if (event == cv::EVENT_LBUTTONDOWN) //左键按下，读取初始坐标，并在图像上该点处划圆
    {
        org.copyTo(img); //将原始图片复制到img中
        //sprintf(temp,"(%d,%d)",x,y);
        pre_pt = cv::Point(x, y);
        //cv::putText(org,temp,pre_pt,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0,255),1,8);//在窗口上显示坐标
        if (color == 1)
        {
            cv::circle(org, pre_pt, 2, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA, 0); //划圆
        }
        else if (color == 2)
        {
            cv::circle(org, pre_pt, 2, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA, 0); //划圆
        }
        cv::imshow("img", org);
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        Point p;
        p.x = x;
        p.y = y;
        colors.push_back(color);
        points.push_back(p);
        //        std::cout<<"mouse button up"<<std::endl;
    }
}

int main()
{
    org = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::namedWindow("img");
    cv::setMouseCallback("img", on_mouse, 0); //调用回调函数
    cv::imshow("img", org);
    colors.clear();
    while (true)
    {
        char key = cvWaitKey(10);
        if (key == 's')
        {
            ofstream fout("/home/jinzedong/桌面/路径评估/path_datas/path.txt");
            fout << points.size() << endl;
            for (int i = 0; i < points.size(); ++i)
            {
                fout << points[i].x << ',' << points[i].y << ',' << colors[i] << endl;
            }
            fout.close();
            return 0;
        }
        if (key == 'b')
        {
            color = 1;
        }
        if (key == 'r')
        {
            color = 2;
        }
    }
    return 0;
}