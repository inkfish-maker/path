#ifndef ASSESSMENT_H
#define ASSESSMENT_H

#include "Route.h"
//=============================评价函数===============================
//权重
DataType weight1, weight2, weight3, weight4, weight5, weight6;
//归一化范围
const DataType zone = 100;
//能不能提前优化去掉一些极端值
//评估函数
DataType get_MaximumAngleChange(int index);
DataType get_TrackWidthStandardDeviation(int index);
DataType get_DistanceStandardDeviation(int index);
DataType get_MaximalWrongColorProbability(int index);
DataType get_LengthSensorRangeSquaredDifference(int index);
DataType get_PathMatchingDegree(int index);
//从点集到最佳路径
void process(const PointType &start_point);

//最大角度变化version_1(相邻角的变化)
DataType get_MaximumAngleChange(int index)
{
    vector<DataType> angles;
    // cout << "angles:" << endl;
    for (int i = 0; i < paths[index].size() - 2; i++)
    {
        angles.push_back(GetAngle(paths[index][i], paths[index][i + 1], paths[index][i + 2]));
    }
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
//     // if (maxAngleChange > M_PI_2)
//     //     maxAngleChange *= 2;
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
        // if (widths[index][i] > maxwidth || widths[index][i] < minwidth)
        // cout<<"=============="<<endl;
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
    return 1;
}

// 没有实现
DataType get_LengthSensorRangeSquaredDifference(int index)
{
    return 1;
}

//使用DTW算法计算路径的匹配度
//参考理解https://blog.csdn.net/qq_39516859/article/details/81705010#commentBox
DataType get_PathMatchingDegree(int index)
{
    vector<PointType> prepath = last_path;
    vector<PointType> nowpath = paths[index];
    int pre_size = prepath.size();
    int now_size = nowpath.size();
    DataType sum = 0;
    //路径长度不一样，使用动态规划计算
    //(pre_size+1)*(now_size+1)的矩阵
    vector<vector<DataType>> dis_matrix(pre_size + 1, vector<DataType>(now_size + 1, 0));
    vector<vector<DataType>> best_matrix(pre_size + 1, vector<DataType>(now_size + 1, 0));
    //初始化距离
    // PointType vec = nowpath[0] - prepath[0];
    for (int i = 1; i <= pre_size; i++)
    {
        for (int j = 1; j <= now_size; j++)
        {
            dis_matrix[i][j] = Dist2(prepath[i - 1], nowpath[j - 1]);
        }
    }
    //动态规划
    for (int i = 1; i <= pre_size; i++)
    {
        for (int j = 1; j <= now_size; j++)
        {
            best_matrix[i][j] = min(min(best_matrix[i - 1][j], best_matrix[i][j - 1]), best_matrix[i - 1][j - 1]) + dis_matrix[i][j];
        }
    }
    sum = sqrt(best_matrix[pre_size][now_size] / (min(pre_size, now_size)));

    // //如果路径长度一致，直接计算欧拉距离
    // if (pre_size == now_size)
    // {
    //     cout << "dp:" << sum << endl;
    //     sum = 0;
    //     for (int i = 0; i < now_size; i++)
    //     {
    //         sum += Dist2(prepath[i], nowpath[i]);
    //     }
    //     sum = sqrt(sum / now_size);
    //     cout << "direct:" << sum << endl;
    // }
    return sum;
}

void SetWeight(DataType w1, DataType w2, DataType w3, DataType w4, DataType w5, DataType w6)
{
    weight1 = w1;
    weight2 = w2;
    weight3 = w3;
    weight4 = w4;
    weight5 = w5;
    weight6 = w6;
}

//评估一条路径
DataType AssessPath(DataType angle_change, DataType path_width_dev, DataType path_lenth_dev, DataType color, DataType sensor_range, DataType matching_degree)
{
    DataType w1 = weight1, w2 = weight2, w3 = weight3, w4 = weight4, w5, weight5, w6 = weight6;
    // if (angle_change > zone * 0.2)
    //     w1 *= 3;
    // if (path_width_dev > zone * 0.2)
    //     w2 *= 3;
    // if (path_lenth_dev > zone * 0.2)
    //     w3 *= 3;
    // if (matching_degree > zone * 0.2)
    //     w6 *= 3;
    return w1 * angle_change + w2 * path_width_dev + w3 * path_lenth_dev + w4 * color + w5 * sensor_range + w6 * matching_degree;
}

//选择最佳路径
int BestPath()
{
    cout << "========BestPath========" << endl;
    int min_path = -1;
    DataType min_coefficient = FLT_MAX; //double可以改成DBL_MAX
    DataType now_coefficient = FLT_MAX;
    DataType angle_change = -1, path_width_dev = -1, path_lenth_dev = -1, matching_degree = -1;
    vector<DataType> angle_path, width_path, lenth_path, match_path;
    cout << paths.size() << endl;
    //记录每条路径的数据
    for (int i = 0; i < paths.size(); i++)
    {
        angle_change = get_MaximumAngleChange(i);
        path_width_dev = get_TrackWidthStandardDeviation(i);
        path_lenth_dev = get_DistanceStandardDeviation(i);
        if (last_path.size() > 0)
            matching_degree = get_PathMatchingDegree(i);
        angle_path.push_back(angle_change);
        width_path.push_back(path_width_dev);
        lenth_path.push_back(path_lenth_dev);
        if (last_path.size() > 0)
            match_path.push_back(matching_degree);
        // if (last_path.size() > 0)
        //     now_coefficient = AssessPath(angle_change, path_width_dev, path_lenth_dev, 0, 0, matching_degree);
        // else
        //     now_coefficient = AssessPath(angle_change, path_width_dev, path_lenth_dev, 0, 0, 0);
    }
    //求极值
    DataType min_angleDiff = *min_element(angle_path.begin(), angle_path.end());
    DataType max_angleDiff = *max_element(angle_path.begin(), angle_path.end());
    DataType min_widthDiff = *min_element(width_path.begin(), width_path.end());
    DataType max_widthDiff = *max_element(width_path.begin(), width_path.end());
    DataType min_lenthDiff = *min_element(lenth_path.begin(), lenth_path.end());
    DataType max_lenthDiff = *max_element(lenth_path.begin(), lenth_path.end());
    DataType min_matchingdegree = -1;
    DataType max_matchingdegree = -1;
    if (last_path.size() > 0)
    {
        min_matchingdegree = *min_element(match_path.begin(), match_path.end());
        max_matchingdegree = *max_element(match_path.begin(), match_path.end());
    }

    //数值归一化
    DataType angle_value = zone, width_value = zone, lenth_value = zone, match_value = zone;
    for (int i = 0; i < paths.size(); i++)
    {
        angle_value = angle_path[i];
        width_value = width_path[i];
        lenth_value = lenth_path[i];
        if (last_path.size() > 0)
            match_value = match_path[i];
        else
            match_value = 0;
        angle_value = zone * (angle_value - min_angleDiff) / (max_angleDiff - min_angleDiff);
        width_value = zone * (width_value - min_widthDiff) / (max_widthDiff - min_widthDiff);
        lenth_value = zone * (lenth_value - min_lenthDiff) / (max_lenthDiff - min_lenthDiff);
        if (last_path.size() > 0)
            match_value = zone * (match_value - min_matchingdegree) / (max_matchingdegree - min_matchingdegree);
        //更新最小值
        now_coefficient = AssessPath(angle_value, width_value, lenth_value, 0, 0, match_value);
        if (now_coefficient < min_coefficient)
        {
            min_coefficient = now_coefficient;
            min_path = i;
        }
        // cout << "i:" << i << endl;
        // cout << "anlge_value:" << angle_value << endl;
        // cout << "width_value:" << width_value << endl;
        // cout << "lenth_value:" << lenth_value << endl;
        // if (last_path.size() > 0)
        //     cout << "match_value:" << match_value << endl;
        // cout << "now_coefficient:" << now_coefficient << endl;
        // cout << "min_coefficient:" << min_coefficient << endl;
        // cout << "==========================" << endl;
    }
    // cout << "min_path:" << min_path << endl;
    // cout << "anlge_value:" << angle_value << endl;
    // cout << "width_value:" << width_value << endl;
    // cout << "lenth_value:" << lenth_value << endl;
    // if (last_path.size() > 0)
    //     cout << "match_value:" << match_value << endl;
    // cout << "now_coefficient:" << now_coefficient << endl;
    // cout << "min_coefficient:" << min_coefficient << endl;
    // cout << "==========================" << endl;
    last_path = paths[min_path];
    last_path.resize(paths[min_path].size());
    return min_path;
}
//===================================================================

//在OpenCv中模拟跑动
void run(int start_tri, int p1_index, int p2_index)
{
    cout << "===========run===========" << endl;
    DFS_Path(start_tri, p1_index, p2_index, true);
    //防止回头
    int start = start_tri;
    int pre = -1;
    int prepre = -1;
    while (true)
    {
        clock_t start_c = clock(), end_c;
        int pathindex = BestPath();
        Mat temp;
        board.copyTo(temp);
        circle(temp, Point2d(paths[pathindex][0].x, paths[pathindex][0].y), 2, Scalar(0, 0, 255), CV_FILLED, CV_AA, 0);

        //debug用,"s"键直接显示最佳路径,"j"键显示所有路径
        // while (true)
        // {
        //     char key = cvWaitKey(10);
        //     if (key == 's')
        //     {
        //         break;
        //     }
        //     else if (key == 'j')
        //     {
        //         for (int i = 0; i < paths.size(); i++)
        //         {
        //             Mat temp2;
        //             board.copyTo(temp2);
        //             for (int j = 0; j < paths[i].size() - 1; j++)
        //             {
        //                 line(temp2, Point2d(paths[i][j].x, paths[i][j].y), Point2d(paths[i][j + 1].x, paths[i][j + 1].y), Scalar(0, 255, 0));
        //             }
        //             imshow("main", temp2);
        //             waitKey(0);
        //         }
        //     }
        // }

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
        DFS_Path(start, nextp1, nextp2, true);
    }
}

void process(const PointType &start_point, const PointType &start_orientation)
{
    cout << "========process========" << endl;
    triangles.resize(0);
    initDelaunay(points);
    Delaunay();
    show_tri(triangles);
    ConnectTri(triangles);
    ClearPath();
    //解决起点接口的问题
    vector<int> start = FindStart_intri(start_point, start_orientation);
    sort(points.begin(), points.end(), [](PointType p1, PointType p2)
         { return p1.index < p2.index; });
    run(start[0], start[1], start[2]);
}
#endif