#include "include/Edge.h"
#include "include/Triangle.h"                              
#include "include/Delaunay.h"
#include "include/Path.h"

int main()
{
    board = Mat(Mat_len, Mat_width, CV_8UC3, Scalar(0, 0, 0));
    namedWindow("main");

    srand((unsigned)time(NULL));
    create_points();

    initDelaunay();
    Delaunay();
    show_tri(triangles);
    ConnectTri(triangles);
    DFS_Path(0);

    cout << paths.size() << endl;
    for (int j = 0; j < paths.size(); j++)
    {
        Mat temp = board;
        cout << paths[j].size() << endl;
        for (int i = 0; i < paths[j].size() - 1; i++)
        {
            line(temp, Point2d(paths[j][i].x, paths[j][i].y), Point2d(paths[j][i + 1].x, paths[j][i + 1].y), Scalar(0, 255, 0));
        }
        imshow("main", temp);
        waitKey(0);
    }
}