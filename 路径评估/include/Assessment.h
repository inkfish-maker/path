#ifndef ASSESSMENT_H
#define ASSESSMENT_H

#include "Path.h"
DataType weight1, weight2, weight3;
//能不能提前优化去掉一些极端值

DataType get_MaximumAngleChange(int index);
DataType get_TrackWidthStandardDeviation(int index);
DataType get_DistanceStandardDeviation(int index);
DataType get_MaximalWrongColorProbability(int index);
DataType get_LengthSensorRangeSquaredDifference(int index);

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

//最大角度变化？？
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
    int i = 0;
    for (i = 0; i < paths.size(); i++)
    {
        angle_change = get_MaximumAngleChange(i);
        path_width_dev = get_TrackWidthStandardDeviation(i);
        path_lenth_dev = get_DistanceStandardDeviation(i);
        now_coefficient = AssessPath(angle_change, path_width_dev, path_lenth_dev);
        if (now_coefficient < min_coefficient)
        {
            min_coefficient = now_coefficient;
            min_path = i;
        }
    }
    return i;
}
#endif