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
    cout << path.size() << endl;
    cout << triangles.size() << endl;

    for (int i = 0; i < path.size() - 1; i++)
    {
        line(board, Point2d(path[i].x, path[i].y), Point2d(path[i + 1].x, path[i + 1].y), Scalar(0, 255, 0));
    }

    imshow("main", board);
    waitKey(0);
}