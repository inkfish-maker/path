#ifndef ASSESSMENT_H
#define ASSESSMENT_H

#include "Path.h"
//=============================评价函数===============================
//权重
DataType weight1, weight2, weight3;
//归一化范围
const DataType zone = 100;
//能不能提前优化去掉一些极端值
//评估函数
DataType get_MaximumAngleChange(int index);
DataType get_TrackWidthStandardDeviation(int index);
DataType get_DistanceStandardDeviation(int index);
DataType get_MaximalWrongColorProbability(int index);
DataType get_LengthSensorRangeSquaredDifference(int index);
//从点集到最佳路径
void process(const PointType &start_point);

//计算两点之间距离
DataType Dist(const PointType &p1, const PointType &p2)
{
    return sqrt(pow(fabs(p1.x - p2.x), 2) + pow(fabs(p1.y - p2.y), 2));
}

//计算角度，返回值为弧度制
//取值范围[-pi,pi],右转+，左转-
DataType GetAngle(const PointType &p1, const PointType &p2, const PointType &p3)
{
    DataType angle = atan2(p2.y - p1.y, p2.x - p1.x) - atan2(p3.y - p2.y, p3.x - p2.x);
    if (angle < -M_PI)
        angle += 2 * M_PI;
    else if (angle > M_PI)
        angle -= 2 * M_PI;
    // cout << angle / M_PI * 180 << endl;
    return angle;
}

// //最大角度变化version_1(相邻角的变化)
DataType get_MaximumAngleChange(int index)
{
    vector<DataType> angles;
    // cout << "angles:" << endl;
    for (int i = 0; i < paths[index].size() - 2; i++)
        angles.push_back(GetAngle(paths[index][i], paths[index][i + 1], paths[index][i + 2]));
    DataType maxAngleChange = -1;
    for (int i = 0; i < angles.size() - 1; i++)
        maxAngleChange = max(maxAngleChange, fabs(angles[i + 1] - angles[i]));
    // cout << "MaximumAngleChange:" << endl;
    // cout << maxAngleChange / M_PI * 180 << endl;
    return maxAngleChange;
}

//最大角度变化version_2(全局角的变化)
// DataType get_MaximumAngleChange(int index)
// {
//     vector<DataType> angles;
//     // cout << "angles:" << endl;
//     for (int i = 0; i < paths[index].size() - 2; i++)
//     {
//         angles.push_back(GetAngle(paths[index][i], paths[index][i + 1], paths[index][i + 2]));
//         // cout << angles[i] << endl;
//     }
//     DataType maxAngleChange = -1;
//     sort(angles.begin(), angles.end());
//     maxAngleChange = angles[angles.size() - 1] - angles[0];
//     // cout << "MaximumAngleChange:" << endl;
//     // cout << maxAngleChange / M_PI * 180 << endl;
//     return maxAngleChange;
// }

//路径宽度的标准差
DataType get_TrackWidthStandardDeviation(int index)
{
    DataType average_val = 0;
    DataType deviation_val = 0;
    // cout << "path width:" << endl;
    for (int i = 0; i < widths[index].size(); i++)
    {
        average_val += widths[index][i];
        // cout << widths[index][i] << endl;
    }
    average_val = average_val / widths[index].size();
    for (int i = 0; i < widths[index].size(); i++)
        deviation_val += pow(fabs(widths[index][i] - average_val), 2);
    deviation_val = sqrt(deviation_val);
    if (widths[index].size() > 1)
    {
        deviation_val = deviation_val / (widths[index].size() - 1);
        return deviation_val;
    }
    else
    {
        cout << "deviation_val error!" << endl;
        return -1;
    }
}

//路径长度的标准差
DataType get_DistanceStandardDeviation(int index)
{
    vector<DataType> distance;
    DataType average_val = 0;
    DataType deviation_val = 0;
    DataType dis_temp = 0;
    for (int i = 0; i < paths[index].size() - 1; i++)
    {
        dis_temp = Dist(paths[index][i], paths[index][i + 1]);
        distance.push_back(dis_temp);
        average_val += dis_temp;
    }
    average_val = average_val / paths[index].size();
    for (int i = 0; i < distance.size(); i++)
        deviation_val += pow(fabs(distance[i] - average_val), 2);
    deviation_val = sqrt(deviation_val);
    if (distance.size() > 1)
    {
        deviation_val = deviation_val / (distance.size() - 1);
        return deviation_val;
    }
    else
    {
        cout << "deviation_val error!" << endl;
        return -1;
    }
}

//颜色误差，暂时搁置
DataType get_MaximalWrongColorProbability(int index)
{
    ;
}

// 没有实现
DataType get_LengthSensorRangeSquaredDifference(int index)
{
    ;
}

void SetWeight(DataType w1, DataType w2, DataType w3)
{
    weight1 = w1;
    weight2 = w2;
    weight3 = w3;
}

//评估一条路径
DataType AssessPath(DataType angle_change, DataType path_width_dev, DataType path_lenth_dev)
{
    return weight1 * angle_change + weight2 * path_width_dev + weight3 * path_lenth_dev;
}

//选择最佳路径
int BestPath()
{
    int min_path = -1;
    DataType min_coefficient = FLT_MAX; //double可以改成DBL_MAX
    DataType now_coefficient = FLT_MAX;
    DataType angle_change = -1, path_width_dev = -1, path_lenth_dev = -1;
    vector<DataType> angle_path, width_path, lenth_path;
    //记录每条路径的数据
    for (int i = 0; i < paths.size(); i++)
    {
        angle_change = get_MaximumAngleChange(i);
        path_width_dev = get_TrackWidthStandardDeviation(i);
        path_lenth_dev = get_DistanceStandardDeviation(i);
        angle_path.push_back(angle_change);
        width_path.push_back(path_width_dev);
        lenth_path.push_back(path_lenth_dev);
        now_coefficient = AssessPath(angle_change, path_width_dev, path_lenth_dev);
    }
    //求极值
    DataType min_angleDiff = *min_element(angle_path.begin(), angle_path.end());
    DataType max_angleDiff = *max_element(angle_path.begin(), angle_path.end());
    DataType min_widthDiff = *min_element(width_path.begin(), width_path.end());
    DataType max_widthDiff = *max_element(width_path.begin(), width_path.end());
    DataType min_lenthDiff = *min_element(lenth_path.begin(), lenth_path.end());
    DataType max_lenthDiff = *max_element(lenth_path.begin(), lenth_path.end());

    //数值归一化
    DataType angle_value = -1, width_value = -1, lenth_value = -1;
    // cout << "*******************************" << endl;
    // cout << max_widthDiff << " - " << min_widthDiff << " = " << max_widthDiff - min_widthDiff << endl;
    for (int i = 0; i < paths.size(); i++)
    {
        angle_value = angle_path[i];
        width_value = width_path[i];
        lenth_value = lenth_path[i];
        angle_value = zone * (angle_value - min_angleDiff) / (max_angleDiff - min_angleDiff);
        width_value = zone * (width_value - min_widthDiff) / (max_widthDiff - min_widthDiff);
        lenth_value = zone * (lenth_value - min_lenthDiff) / (max_lenthDiff - min_lenthDiff);
        // cout << angle_value << "  " << width_value << "  " << lenth_value << endl;
        //更新最小值
        now_coefficient = AssessPath(angle_value, width_value, lenth_value);
        if (now_coefficient < min_coefficient)
        {
            min_coefficient = now_coefficient;
            min_path = i;
        }
    }
    return min_path;
}
//===================================================================

//在OpenCv中模拟跑动
void run(int start_tri, int p1_index, int p2_index)
{
    DFS_Path(start_tri, p1_index, p2_index);
    //防止回头
    int start = start_tri;
    int pre = -1;
    int prepre = -1;
    while (true)
    {
        clock_t start_c = clock(), end_c;
        int pathindex = BestPath();
        // cout << start << "  ";
        // for (int i = 0; i < nextinfos[pathindex].size(); i++)
        // {
        //     cout << nextinfos[pathindex][i].first << " ";
        // }
        //展示最佳路径
        Mat temp;
        board.copyTo(temp);
        circle(temp, Point2d(paths[pathindex][0].x, paths[pathindex][0].y), 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);
        // cout << "pre:" << pre << endl;
        // line(temp, triangles[start].p1, triangles[start].p2, Scalar(255, 255, 0));
        // line(temp, triangles[start].p1, triangles[start].p3, Scalar(255, 255, 0));
        // line(temp, triangles[start].p2, triangles[start].p3, Scalar(255, 255, 0));
        for (int i = 0; i < paths[pathindex].size() - 1; i++)
        {
            line(temp, Point2d(paths[pathindex][i].x, paths[pathindex][i].y), Point2d(paths[pathindex][i + 1].x, paths[pathindex][i + 1].y), Scalar(0, 255, 0));
        }
        end_c = clock();
        cout << "runtime:" << (end_c - start_c) * 1.0 / CLOCKS_PER_SEC << endl;
        imshow("main", temp);
        waitKey(0);
        start_c = clock();

        triangles[start].visit = true;
        if (pre != -1)
            triangles[pre].visit = true;
        if (prepre != -1)
            triangles[prepre].visit = false;
        prepre = pre;
        pre = start;
        start = nextinfos[pathindex][1].first;
        int nextp1 = nextinfos[pathindex][1].second.px.index, nextp2 = nextinfos[pathindex][1].second.py.index;
        ClearPath();
        DFS_Path(start, nextp1, nextp2);
    }
}

void process(const PointType &start_point, const PointType &start_orientation)
{
    triangles.resize(0);
    initDelaunay();
    Delaunay();
    show_tri(triangles);
    ConnectTri(triangles);
    sort(points.begin(), points.end(), [](PointType p1, PointType p2) { return p1.index < p2.index; });
    ClearPath();
    //解决起点接口的问题
    vector<int> start = FindStart(start_point, start_orientation);
    run(start[0], start[1], start[2]);
}
#endif