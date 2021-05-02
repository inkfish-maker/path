#ifndef DELAUNAY_H
#define DELAUNAY_H

#include "Triangle.h"

//生成点集（随机）
void create_points();
//读取点集
void read_points();
//展示效果
void show_tri(vector<Triangle> &triangles_);
//构建超级三角形
void initDelaunay();
//三角剖分
void Delaunay();
//判断一个点是否在三角形内部(在内部返回1，在外部且在右侧返回2，在外部但不在右侧返回3)
int Circle_Judge(Triangle tri, PointType p);
//判断一点是否在点集内(相等于判断是否与超级三角形相关)
bool InRegion(PointType p);
//删去与超级三角形有关的三角
bool CheckSuper(Triangle tri);

void create_points()
{
    // std::cout << "point_position:" << endl;
    for (int i = 0; i < point_num; i++)
    {
        PointType p(rand() % Mat_len, rand() % Mat_width);
        // PointType p(rand() % Mat_len / 3 + Mat_len / 3, rand() % Mat_width / 3 + Mat_width / 3);
        p.index = i;
        points.push_back(p);
        circle(board, p, 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);
        // std::cout << p.x << "      " << p.y << endl;
    }
}
void read_points()
{
    fstream fin;
    fin.open("/home/jinzedong/new_data/path/src/ROS接口/path_datas/path2.txt");
    int numberPoints, color;
    double x, y;
    char none_key;
    fin >> numberPoints;
    point_num = numberPoints;
    for (int i = 0; i < numberPoints; ++i)
    {
        fin >> x;
        fin >> none_key;
        fin >> y;
        fin >> none_key;
        fin >> color;
        PointType p(x, y);
        p.index = i;
        points.push_back(p);
        circle(board, p, 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);
    }
    fin.close();
}
void show_tri(vector<Triangle> &triangles_)
{
    for (int i = 0; i < triangles.size(); i++)
    {
        line(board, triangles_[i].p1, triangles_[i].p2, Scalar(255, 0, 0));
        line(board, triangles_[i].p1, triangles_[i].p3, Scalar(255, 0, 0));
        line(board, triangles_[i].p2, triangles_[i].p3, Scalar(255, 0, 0));
    }
}
void initDelaunay()
{
    std::sort(points.begin(), points.end(), cmp_y);
    DataType down_y = points[0].y;
    DataType up_y = points[point_num - 1].y;

    // //上垂线
    // line(board, PointType(0, points[0].y), PointType(Mat_len, points[0].y), Scalar(255, 0, 0));
    // //下垂线
    // line(board, PointType(0, poinTriangle super_trianglets[point_num - 1].y), PointType(Mat_len, points[point_num - 1].y), Scalar(255, 0, 0));

    std::sort(points.begin(), points.end(), cmp_x);
    DataType left_x = points[0].x;
    DataType right_x = points[point_num - 1].x;

    // //左垂线
    // line(board, PointType(points[0].x, 0), PointType(points[0].x, Mat_width), Scalar(255, 0, 0));
    // //右垂线
    // line(board,PointType(points[point_num - 1].x, 0), PointType(points[point_num - 1].x, Mat_width), Scalar(255, 0, 0));

    //点集的长
    DataType point_len = right_x - left_x;
    //点集的宽
    DataType point_wid = up_y - down_y;
    //输入点集区间
    region.push_back(PointType(left_x, down_y));
    region.push_back(PointType(right_x, up_y));
    //超级三角形如果太小，会由于精度问题，导致部分三角形无法生成
    //上顶点
    PointType p_up(left_x + point_len / 2, down_y + point_wid * 3);
    //左顶点
    PointType p_left(left_x - point_len * 0.6, down_y - point_wid * 0.5);
    //右顶点
    PointType p_right(left_x + point_len * 1.6, down_y - point_wid * 0.5);

    //超级三角形
    // line(board, p_up, p_left, Scalar(255, 0, 0));
    // line(board, p_up, p_right, Scalar(255, 0, 0));
    // line(board, p_left, p_right, Scalar(255, 0, 0));

    Triangle super_triangle(p_up, p_left, p_right);
    temp_triangles.push(super_triangle);
    triangles.push_back(super_triangle);
}
int Circle_Judge(Triangle tri, PointType p)
{
    DataType x = tri.centre.x;
    DataType y = tri.centre.y;
    if (sqrt(pow(x - p.x, 2) + pow(y - p.y, 2)) <= tri.r)
    {
        return 1;
    }
    else
    {
        if (tri.centre.x + tri.r < p.x)
            return 2;
        else
            return 3;
    }
}
bool InRegion(PointType p)
{
    if (p.x >= region[0].x && p.x <= region[1].x && p.y >= region[0].y && p.y <= region[1].y)
        return true;
    return false;
}
bool CheckSuper(Triangle tri)
{
    if (InRegion(tri.p1) && InRegion(tri.p2) && InRegion(tri.p3))
        return false;
    return true;
}
void Delaunay()
{
    for (int index = 0; index < point_num; index++)
    {
        //目前比较到的顶点
        PointType now_point = points[index];
        // cout << index << ":" << now_point.index << "-" << points[index].index << endl;
        //这一轮比较的三角
        int temp_triangle_last = temp_triangles.size();

        for (int i = 0; i < temp_triangle_last; i++)
        {
            Triangle now_tri = temp_triangles.front();
            temp_triangles.pop();
            //判断点与三角形内接圆的关系
            int flag = Circle_Judge(now_tri, now_point);
            //不在内接圆内，不是D三角
            if (flag == 1)
            {
                //该点在三角形外接圆内侧，则这时向清空后的edge buffer加入该三角形的三条边，并用该点写edge buffer中的三角边进行组合，组合成了三个三角形并加入到temp triangles中
                temp_edges.push_back(now_tri.e1);
                temp_edges.push_back(now_tri.e2);
                temp_edges.push_back(now_tri.e3);
            }
            //在内部且在右边，是D三角
            else if (flag == 2)
            {
                //该点在三角形外接圆右侧，则表示左侧三角形为Delaunay三角形，将该三角形保存至triangles中
                // circle(board, now_tri.centre, now_tri.r, Scalar(0, 0, 255), CV_16S, CV_AA, 0);
                triangles.push_back(now_tri);
            }
            //在内部但不在右边，无法判断
            else
            {
                //该点在三角形外接圆外侧，为不确定三角形，所以跳过（后面会讲到为什么要跳过该三角形），但并不在temp triangles中删除
                temp_triangles.push(now_tri);
            }
        }
        //边去重
        std::sort(temp_edges.begin(), temp_edges.end(), cmp_edge);
        for (vector<Edge>::iterator e1 = temp_edges.begin(); e1 != temp_edges.end();)
        {
            Edge e = *e1;
            int isdouble = 0;
            for (vector<Edge>::iterator e2 = e1 + 1; e2 != temp_edges.end();)
            {
                if (e == *e2)
                {
                    temp_edges.erase(e2);
                    isdouble = 1;
                }
                else
                {
                    e2++;
                }
            }
            if (isdouble)
            {
                temp_edges.erase(e1);
            }
            else
                e1++;
        }

        //构建新的三角形
        for (int i = 0; i < temp_edges.size(); i++)
        {
            temp_triangles.push(Triangle(now_point, temp_edges[i]));
            //查看temp_tri
            // line(board, temp_edges[i].px, temp_edges[i].py, Scalar(255, 0, 0));
            // imshow("main", board);
            // line(board, now_point, temp_edges[i].px, Scalar(255, 0, 0));
            // imshow("main", board);
            // line(board, now_point, temp_edges[i].py, Scalar(255, 0, 0));
            // imshow("main", board);
        }
        temp_edges.resize(0);
    }

    //将temp_tri和tri合并
    while (!temp_triangles.empty())
    {
        triangles.push_back(temp_triangles.front());
        temp_triangles.pop();
    }
    // 去除与超级三角形有关的tri
    //cnt最后有几个合法的tri
    int cnt = 0;
    for (vector<Triangle>::iterator tri = triangles.begin(); tri != triangles.end();)
    {
        if (CheckSuper(*tri))
        {
            tri = triangles.erase(tri);
        }
        else
        {
            cnt++;
            ++tri;
        }
    }
    std::cout << "cnt:" << cnt << endl;
    // for (int i = 0; i < triangles.size(); i++)
    // {
    //     cout << i << ":" << triangles[i].p1.index << "-" << triangles[i].p2.index << "-" << triangles[i].p3.index << endl;
    // }
}

#endif