#ifndef ASSESSMENT_H
#define ASSESSMENT_H

#include "Path.h"

DataType get_MaximumAngleChange(int index);
DataType get_TrackWidthStandardDeviation(int index);
DataType get_DistanceStandardDeviation(int index);
DataType get_MaximalWrongColorProbability(int index);
DataType get_LengthSensorRangeSquaredDifference(int index);

DataType Dist(const PointType &p1, const PointType &p2)
{
    return sqrt(pow(fabs(p1.x - p2.x), 2) + pow(fabs(p1.y - p2.y), 2));
}

//计算角度，返回值为弧度制
//取值范围[-pi,pi],右转+，左转-
DataType GetAngle(const PointType &p1, const PointType &p2, const PointType &p3)
{
    DataType angle = atan2(p2.x - p1.x, p2.y - p1.y) - atan2(p3.x - p2.x, p3.y - p2.y);
    if (angle < -M_PI)
        angle += 2 * M_PI;
    else if (angle > M_PI)
        angle -= 2 * M_PI;
    return angle;
}

DataType get_MaximumAngleChange(int index)
{
    vector<DataType> angles;
    for (int i = 0; i < paths[index].size() - 2; i++)
        angles.push_back(GetAngle(paths[index][i], paths[index][i + 1], paths[index][i + 2]));
    sort(angles.begin(), angles.end());
    return angles[angles.size() - 1] - angles[0];
}

DataType get_TrackWidthStandardDeviation(int index)
{
    DataType average_val = 0;
    DataType deviation_val = 0;
    for (int i = 0; i < widths[index].size(); i++)
        average_val += widths[index][i];
    average_val = average_val / widths[index].size();
    for (int i = 0; i < widths[index].size(); i++)
        deviation_val += pow(fabs(widths[index][i] - average_val), 2);
    if (widths[index].size() > 1)
        deviation_val = deviation_val / (widths[index].size() - 1);
    else
        cout << "deviation_val error" << endl;
    return deviation_val;
}

#endif