#ifndef CONFIG_H
#define CONFIG_H
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <ostream>
#include <vector>
#include <queue>
#include <algorithm>

using namespace cv;
using namespace std;

//背景板
Mat board;
//背景板参数
int Mat_len = 800;
int Mat_width = 800;
//随机点数量
int point_num = 143;
//记得修改Delaunay.h中read_point的路径

#endif
