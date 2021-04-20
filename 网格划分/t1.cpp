#include "include/Edge.h"
#include "include/Triangle.h"

//背景板
Mat board;
//背景板参数
int Mat_len = 800;
int Mat_width = 800;
//随机点数量
int point_num = 3;
//点集区域,点集左下角与右上角
vector<PointType> region;
//列表顶点
vector<PointType> points;
//缓存边
vector<Edge> temp_edges;
//列表三角形（确认）
vector<Triangle> triangles;
//列表三角形（暂时）
queue<Triangle> temp_triangles;

//生成点集
void create_points();
//构建超级三角形
void initDelaunay();
//三角剖分
void Delaunay();
//判断一个点是否在三角形内部(在内部1，在外部且在右侧2，在外部但不在右侧3)
int Circle_Judge(Triangle tri, PointType p);
//判断一点是否在点集内
bool InRegion(PointType p);
//删去与超级三角形有关的三角
bool CheckSuper(Triangle tri);
//展示效果
void show_tri();

int main()
{
    board = Mat(Mat_len, Mat_width, CV_8UC3, Scalar(0, 0, 0));
    namedWindow("main");

    srand((unsigned)time(NULL));
    create_points();
    // circle(board, PointType(100, 200), 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);

    initDelaunay();
    Delaunay();
    show_tri();
    imshow("main", board);
    waitKey(0);
}

void create_points()
{
    std::cout << "point_position:" << endl;
    for (int i = 0; i < point_num; i++)
    {
        // PointType p(rand() % Mat_len, rand() % Mat_width);
        PointType p(rand() % Mat_len / 3 + Mat_len / 3, rand() % Mat_width / 3 + Mat_width / 3);
        points.push_back(p);
        circle(board, p, 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);
        std::cout << p.x << "      " << p.y << endl;
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
    //判等t_num - 1].x, 0), PointType(points[point_num - 1].x, Mat_width), Scalar(255, 0, 0));

    //点集的长
    DataType point_len = right_x - left_x;
    //点集的宽
    DataType point_wid = up_y - down_y;
    //输入点集区间
    region.push_back(PointType(left_x, down_y));
    region.push_back(PointType(right_x, up_y));
    //上顶点
    PointType p_up(left_x + point_len / 2, down_y + point_wid * 2);
    //左顶点
    PointType p_left(left_x - point_len * 0.6, down_y - point_wid * 0.05);
    //右顶点
    PointType p_right(left_x + point_len * 1.6, down_y - point_wid * 0.05);
    // //超级三角形
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
    //遍历基于indices顺序的vertices中每一个点
    for (int index = 0; index < point_num; index++)
    {
        //目前比较到的顶点
        PointType now_point = points[index];
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

        //debug
        std::cout << "temp_edge:" << temp_edges.size() << "      temp_tri:" << temp_triangles.size() << "     tri:" << triangles.size() << endl;
        std::cout << "temp_edges:" << endl;
        for (int i = 0; i < temp_edges.size(); i++)
            std::cout << temp_edges[i];

        //构建新的三角形
        for (int i = 0; i < temp_edges.size(); i++)
        {
            temp_triangles.push(Triangle(now_point, temp_edges[i]));
            // std::cout << i + 1 << ":" << now_point.x << "   " << now_point.y << "  " << temp_edges[i].px.x << "   " << temp_edges[i].px.y << "  " << temp_edges[i].py.x << "   " << temp_edges[i].py.y << "  " << endl;
        }
        temp_edges.resize(0);
        // std::cout << "temp_edge:" << temp_edges.size() << "      temp_tri:" << temp_triangles.size() << "     tri:" << triangles.size() << endl;
    }

    std::cout << "sizeof tri:" << triangles.size() << endl;
    //将temp_tri和tri合并
    while (!temp_triangles.empty())
    {
        triangles.push_back(temp_triangles.front());
        temp_triangles.pop();
    }
    // 去除与超级三角形有关的tri
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
            // std::cout << cnt << " :" << tri->p1.x << "   " << tri->p1.y << "  " << tri->p2.x << "   " << tri->p2.y << "  " << tri->p3.x << "   " << tri->p3.y << "  " << endl;
            ++tri;
        }
    }
    std::cout << "cnt:" << cnt << endl;
    //check
    // int flag = 1;
    // for (vector<Triangle>::iterator tri = triangles.begin(); tri != triangles.end();)
    // {
    //     for (int i = 0; i < point_num; i++)
    //     {
    //         if (points[i] == tri->p1 || points[i] == tri->p2 || points[i] == tri->p3)
    //             continue;
    //         if (Circle_Judge(*tri, points[i]) == 1)
    //         {
    //             triangles.erase(tri);
    //             flag = 0;
    //             break;
    //         }
    //     }
    //     if (flag)
    //         tri++;
    //     flag = 1;
    // }
}

void show_tri()
{
    for (int i = 0; i < triangles.size(); i++)
    {
        line(board, triangles[i].p1, triangles[i].p2, Scalar(255, 0, 0));
        line(board, triangles[i].p1, triangles[i].p3, Scalar(255, 0, 0));
        line(board, triangles[i].p2, triangles[i].p3, Scalar(255, 0, 0));
    }
}