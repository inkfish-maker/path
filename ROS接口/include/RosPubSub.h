#ifndef ROSPUBSUB_H
#define ROSPUBSUN_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <back_end/BackEndOutput.h>
#include <autoware_msgs/lane.h>
#include <Eigen/Dense>
#include <Path.h>

class RosPubSub
{
public:
    RosPubSub();
    ros::Subscriber sub_circle_race; //接收跑动的圈数
    ros::Subscriber sub_back_end;    //后端发来的桩筒数据
    ros::Publisher pub2pure_pursuit; //发送给纯追踪控制的路径消息
    ros::Publisher pub2lqr;          //发送给lqr控制的路径消息
    ros::NodeHandle nh;

    //圈数计时器的一些变量
    bool global_path_flag;
    bool realtime_planning_flag;
    bool initPath;
    int race_count = 0;
};
RosPubSub ppnode;
//历史遗留问题
void circle_race_callback(const std_msgs::Int32 &msg)
{
    try
    {
        if (msg.data - ppnode.race_count == 1)
        {
            ppnode.race_count = msg.data;
            ppnode.initPath = true;
            std::cout << "Round " << ppnode.race_count << std::endl;
        }
    }
    catch (...)
    {
        std::cout << "Exception:" << std::endl;
        return;
    }
}

void generate_path_callback(const back_end::BackEndOutput &msg)
{
    try
    {
        clock_t time_start, time_end;
        time_start = clock();
        if (ppnode.global_path_flag == false)
        {
            if (ppnode.realtime_planning_flag == true)
            {
                int num = msg.bboxArray.boxes.size();
                if (num == 0)
                {
                    return;
                }
                PointType start_pose(msg.odometry.pose.pose.position.x, msg.odometry.pose.pose.position.y);
                DataType ori_x, ori_y;
                DataType x, y, z, w;
                std::cout << "num of cones = " << num << std::endl;
                if (num < 20)
                {
                    mindepth = 4;
                    maxdepth = 6;
                }
                else
                {
                    ;
                }
                x = msg.odometry.pose.pose.orientation.x;
                y = msg.odometry.pose.pose.orientation.y;
                z = msg.odometry.pose.pose.orientation.z;
                w = msg.odometry.pose.pose.orientation.w;
                Eigen::Quaterniond quaternion4(w, x, y, z);
                Eigen::Vector3d eulerAngle = quaternion4.matrix().eulerAngles(0, 1, 2);

                ori_y = sin(eulerAngle[2]);
                ori_x = cos(eulerAngle[2]);
                PointType orientation(ori_x, ori_y);
                std::cout << "set pose" << std::endl;
                //load cones
                points.resize(0);
                for (int i = 0; i < num; ++i)
                {
                    PointType point(msg.bboxArray.boxes[i].pose.position.x, msg.bboxArray.boxes[i].pose.position.y);
                    point.index = i;
                    points.push_back(point);
                }
                std::cout << "load cones" << std::endl;
                path_maker.process();
                std::cout << "generate finished" << std::endl;
                // way = path_maker.getWay();
                way = path_maker.way_splined;
                if (way.size() > 0)
                {
                    //如果是初始化路径
                    std::cout << "generate path size()=" << way.size() << std::endl;
                }
                else
                {
                    std::cout << "Error:way is empty!" << std::endl;
                    return;
                }
                //ppnode.waypoints_record.push_back(way[0]);
                time_end = clock();
                way_points.waypoints.clear();
                way_points.header.frame_id = "map";
                way_points.header.stamp = ros::Time::now();
                autoware_msgs::waypoint tmp_point;
                for (int i = 0; i < way.size(); ++i)
                {
                    tmp_point.pose.pose.position.x = way[i].x;
                    tmp_point.pose.pose.position.y = way[i].y;
                    tmp_point.pose.pose.position.z = 0;
                    way_points.waypoints.push_back(tmp_point);
                }
                ppnode.pub2pure_pursuit.publish(way_points);
                cout << "time cost:" << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << endl;
                std::cout << "---------" << std::endl;

                for (int i = 0; i < way.size() - 1; ++i)
                {
                    geometry_msgs::Point pp1, pp2;
                    pp1.x = way[i].x;
                    pp1.y = way[i].y;
                    pp1.z = 0;
                    pp2.x = way[i + 1].x;
                    pp2.y = way[i + 1].y;
                    pp2.z = 0;
                    vis_lines.points.push_back(pp1);
                    vis_lines.points.push_back(pp2);
                }
                std::vector<Edge<double>> edges = path_maker.triangulation.getEdges();
                for (auto e = begin(edges); e != end(edges); ++e)
                {
                    geometry_msgs::Point p1, p2;
                    p1.x = e->p1.x;
                    p1.y = e->p1.y;
                    p2.x = e->p2.x;
                    p2.y = e->p2.y;
                    p1.z = p2.z = 0;
                    vis_edges.points.push_back(p1);
                    vis_edges.points.push_back(p2);
                }
                for (int i = 0; i < way.size(); ++i)
                {
                    geometry_msgs::Point pp;
                    pp.x = way[i].x;
                    pp.y = way[i].y;
                    pp.z = 0;
                    vis_points.points.push_back(pp);
                }
                geometry_msgs::Point ori_point;
                ori_point.x = start_pose.x + ori_x;
                ori_point.y = start_pose.y + ori_y;
                ori_point.z = 0;
                vis_orientation.points.push_back(ori_point);

                ppnode.pub.publish(vis_orientation);
                ppnode.pub.publish(vis_points);
                ppnode.pub.publish(vis_lines);
                ppnode.pub.publish(vis_edges);
                return;
            }
            else
            {
                int num = msg.bboxArray.boxes.size();
                if (num == 0)
                {
                    return;
                }
                Vector2<double> start_pose(msg.odometry.pose.pose.position.x, msg.odometry.pose.pose.position.y);
                double ori_x, ori_y;
                vis_edges.points.clear();
                vis_lines.points.clear();
                vis_orientation.points.clear();
                vis_points.points.clear();
                //set oritenation
                double x, y, z, w;
                vis_edges.points.clear();
                vis_lines.points.clear();
                vis_orientation.points.clear();
                vis_points.points.clear();
                //set oritenation
                x = msg.odometry.pose.pose.orientation.x;
                y = msg.odometry.pose.pose.orientation.y;
                z = msg.odometry.pose.pose.orientation.z;
                w = msg.odometry.pose.pose.orientation.w;
                Eigen::Quaterniond quaternion4(w, x, y, z);
                Eigen::Vector3d eulerAngle = quaternion4.matrix().eulerAngles(0, 1, 2);

                ori_y = sin(eulerAngle[2]);
                ori_x = cos(eulerAngle[2]);

                Vector2<double> orientation(ori_x, ori_y);
                std::cout << "num of cones = " << num << std::endl;
                if (ppnode.race_count == 1)
                {
                    if (num < 20)
                    {
                        path_maker.setMinDepth(4);
                        path_maker.setMaxDepth(6);
                    }
                    else
                    {
                        path_maker.setMinDepth(MinDepth);
                        path_maker.setMaxDepth(MaxDepth);
                    }

                    path_maker.setStartposition(start_pose, orientation);
                    std::cout << "set pose" << std::endl;
                    if (ppnode.initPath == true)
                    {
                        //load cones
                        cones.clear();
                        for (int i = 0; i < num; ++i)
                        {
                            Vector2<double> tmp_cone(msg.bboxArray.boxes[i].pose.position.x, msg.bboxArray.boxes[i].pose.position.y);
                            tmp_cone.color = NOCOLOR_CONE;
                            cones.push_back(tmp_cone);
                        }
                    }
                    std::cout << "load cones" << std::endl;
                    path_maker.setVertices(cones);
                    path_maker.process();
                    std::cout << "generate finished" << std::endl;
                    // way = path_maker.getWay();
                    // way = path_maker.way_splined;
                    way = path_maker.points_way;
                    data_fout << time_count << "," << path_maker.num_vertices << "," << path_maker.num_candidate_trees << ","
                              << path_maker.triangulation.time_generateTriangles << "," << path_maker.time_init << "," << path_maker.time_trigraph << "," << path_maker.time_select_startpoints << ","
                              << path_maker.time_generatePath << "," << path_maker.time_dfsTime << "," << path_maker.time_estimateValue << std::endl;
                    if (way.size() > 0)
                    {
                        ppnode.waypoints_record.push_back(path_maker.points_way[1]);
                    }
                    else
                    {
                        return;
                    }
                }
                else
                {
                    int best_mark = 0;
                    double min_dist = std::numeric_limits<double>::max();
                    Vector2<double> pose_ori = start_pose + orientation;
                    for (int i = 0; i < ppnode.waypoints_record.size(); ++i)
                    {
                        double tmp_angle = getAngle<double>(pose_ori, start_pose, ppnode.waypoints_record[i]);
                        if (tmp_angle + EPS < M_PI / 2 || tmp_angle + EPS > M_PI * 3 / 2)
                        {
                            double tmp_dist = start_pose.dist(ppnode.waypoints_record[i]);
                            if (tmp_dist < EPS)
                            {
                                continue;
                            }
                            if (tmp_dist < min_dist)
                            {
                                min_dist = tmp_dist;
                                best_mark = i;
                            }
                        }
                    }
                    way.clear();
                    way.push_back(start_pose);
                    int maxlen = ppnode.waypoints_record.size();
                    for (int i = 0; i < MaxDepth; ++i)
                    {
                        way.push_back(ppnode.waypoints_record[(best_mark + i) % maxlen]);
                    }
                    std::vector<double> s_list, x_list, y_list;
                    s_list.push_back(0);
                    for (int i = 0; i < way.size(); ++i)
                    {
                        x_list.push_back(way[i].x);
                        y_list.push_back(way[i].y);
                        if (i != 0)
                        {
                            double dist = way[i].dist(way[i - 1]);
                            dist += s_list[i - 1];
                            s_list.push_back(dist);
                        }
                    }
                    tk::spline spline_x, spline_y;
                    way.clear();
                    spline_x.set_points(s_list, x_list, true);
                    spline_y.set_points(s_list, y_list, true);
                    double total_len = s_list[int(s_list.size()) - 1];
                    for (double len = 0; len + EPS < total_len; len += SubLength)
                    {
                        Vector2<double> p(spline_x(len), spline_y(len));
                        way.push_back(p);
                    }
                }
                //set position and orientation

                if (way.size() > 0)
                {
                    //如果是初始化路径
                    std::cout << "generate path size()=" << way.size() << std::endl;
                }
                else
                {
                    std::cout << "Error:way is empty!" << std::endl;
                    return;
                }
                //ppnode.waypoints_record.push_back(way[0]);
                time_end = clock();
                way_points.waypoints.clear();
                way_points.header.frame_id = "map";
                way_points.header.stamp = ros::Time::now();
                autoware_msgs::waypoint tmp_point;
                std::cout << "in planning way_points" << std::endl;
                for (int i = 0; i < way.size(); ++i)
                {
                    // std::cout<<"way["<<i<<"]="<<way[i]<<std::endl;
                    tmp_point.pose.pose.position.x = way[i].x;
                    tmp_point.pose.pose.position.y = way[i].y;
                    tmp_point.pose.pose.position.z = 0;
                    // std::cout<<"way_points.waypoints.push_back"<<std::endl;
                    way_points.waypoints.push_back(tmp_point);
                }
                ppnode.pub2pure_pursuit.publish(way_points);
                cout << "time cost:" << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << endl;
                std::cout << "---------" << std::endl;

                for (int i = 0; i < way.size() - 1; ++i)
                {
                    geometry_msgs::Point pp1, pp2;
                    pp1.x = way[i].x;
                    pp1.y = way[i].y;
                    pp1.z = 0;
                    pp2.x = way[i + 1].x;
                    pp2.y = way[i + 1].y;
                    pp2.z = 0;
                    vis_lines.points.push_back(pp1);
                    vis_lines.points.push_back(pp2);
                }
                std::vector<Edge<double>> edges = path_maker.triangulation.getEdges();
                for (auto e = begin(edges); e != end(edges); ++e)
                {
                    geometry_msgs::Point p1, p2;
                    p1.x = e->p1.x;
                    p1.y = e->p1.y;
                    p2.x = e->p2.x;
                    p2.y = e->p2.y;
                    p1.z = p2.z = 0;
                    vis_edges.points.push_back(p1);
                    vis_edges.points.push_back(p2);
                }
                for (int i = 0; i < way.size(); ++i)
                {
                    geometry_msgs::Point pp;
                    pp.x = way[i].x;
                    pp.y = way[i].y;
                    pp.z = 0;
                    vis_points.points.push_back(pp);
                }
                geometry_msgs::Point ori_point;
                ori_point.x = start_pose.x + ori_x;
                ori_point.y = start_pose.y + ori_y;
                ori_point.z = 0;
                vis_orientation.points.push_back(ori_point);

                ppnode.pub.publish(vis_orientation);
                ppnode.pub.publish(vis_points);
                ppnode.pub.publish(vis_lines);
                ppnode.pub.publish(vis_edges);
                return;
            }
        }
        else if (ppnode.global_path_flag == true)
        {
            if (ppnode.realtime_planning_flag == true)
            {
                int num = msg.bboxArray.boxes.size();
                if (num == 0)
                {
                    return;
                }
                Vector2<double> start_pose(msg.odometry.pose.pose.position.x, msg.odometry.pose.pose.position.y);
                double ori_x, ori_y;
                vis_edges.points.clear();
                vis_lines.points.clear();
                vis_orientation.points.clear();
                vis_points.points.clear();
                //set position and orientation
                //start_pose(msg.odometry.pose.pose.position.x,msg.odometry.pose.pose.position.y);
                //set oritenation
                double x, y, z, w;
                std::cout << "num of cones = " << num << std::endl;
                if (num < 20)
                {
                    path_maker.setMinDepth(4);
                    path_maker.setMaxDepth(6);
                }
                else
                {
                    path_maker.setMinDepth(MinDepth);
                    path_maker.setMaxDepth(MaxDepth);
                }
                vis_edges.points.clear();
                vis_lines.points.clear();
                vis_orientation.points.clear();
                vis_points.points.clear();
                //set position and orientation
                //set oritenation
                x = msg.odometry.pose.pose.orientation.x;
                y = msg.odometry.pose.pose.orientation.y;
                z = msg.odometry.pose.pose.orientation.z;
                w = msg.odometry.pose.pose.orientation.w;
                Eigen::Quaterniond quaternion4(w, x, y, z);
                Eigen::Vector3d eulerAngle = quaternion4.matrix().eulerAngles(0, 1, 2);

                ori_y = sin(eulerAngle[2]);
                ori_x = cos(eulerAngle[2]);

                Vector2<double> orientation(ori_x, ori_y);
                path_maker.setStartposition(start_pose, orientation);
                std::cout << "set pose" << std::endl;
                if (ppnode.race_count == 1)
                {
                    //load cones
                    cones.clear();
                    for (int i = 0; i < num; ++i)
                    {
                        Vector2<double> tmp_cone(msg.bboxArray.boxes[i].pose.position.x, msg.bboxArray.boxes[i].pose.position.y);
                        tmp_cone.color = NOCOLOR_CONE;
                        cones.push_back(tmp_cone);
                    }
                }
                std::cout << "load cones" << std::endl;
                path_maker.setVertices(cones);
                path_maker.process();
                std::cout << "generate finished" << std::endl;
                // way = path_maker.getWay();
                way = path_maker.way_splined;
                if (way.size() > 0)
                {
                    //如果是初始化路径
                    std::cout << "generate path size()=" << way.size() << std::endl;
                }
                else
                {
                    std::cout << "Error:way is empty!" << std::endl;
                    return;
                }
                //ppnode.waypoints_record.push_back(way[0]);
                time_end = clock();
                way_points.waypoints.clear();
                way_points.header.frame_id = "map";
                way_points.header.stamp = ros::Time::now();
                autoware_msgs::waypoint tmp_point;
                for (int i = 0; i < way.size(); ++i)
                {
                    tmp_point.pose.pose.position.x = way[i].x;
                    tmp_point.pose.pose.position.y = way[i].y;
                    tmp_point.pose.pose.position.z = 0;
                    way_points.waypoints.push_back(tmp_point);
                }
                ppnode.pub2pure_pursuit.publish(way_points);
                cout << "time cost:" << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << endl;
                std::cout << "---------" << std::endl;

                for (int i = 0; i < way.size() - 1; ++i)
                {
                    geometry_msgs::Point pp1, pp2;
                    pp1.x = way[i].x;
                    pp1.y = way[i].y;
                    pp1.z = 0;
                    pp2.x = way[i + 1].x;
                    pp2.y = way[i + 1].y;
                    pp2.z = 0;
                    vis_lines.points.push_back(pp1);
                    vis_lines.points.push_back(pp2);
                }
                std::vector<Edge<double>> edges = path_maker.triangulation.getEdges();
                for (auto e = begin(edges); e != end(edges); ++e)
                {
                    geometry_msgs::Point p1, p2;
                    p1.x = e->p1.x;
                    p1.y = e->p1.y;
                    p2.x = e->p2.x;
                    p2.y = e->p2.y;
                    p1.z = p2.z = 0;
                    vis_edges.points.push_back(p1);
                    vis_edges.points.push_back(p2);
                }
                for (int i = 0; i < way.size(); ++i)
                {
                    geometry_msgs::Point pp;
                    pp.x = way[i].x;
                    pp.y = way[i].y;
                    pp.z = 0;
                    vis_points.points.push_back(pp);
                }
                geometry_msgs::Point ori_point;
                ori_point.x = start_pose.x + ori_x;
                ori_point.y = start_pose.y + ori_y;
                ori_point.z = 0;
                vis_orientation.points.push_back(ori_point);

                ppnode.pub.publish(vis_orientation);
                ppnode.pub.publish(vis_points);
                ppnode.pub.publish(vis_lines);
                ppnode.pub.publish(vis_edges);
                return;
            }
            else
            {
                int num = msg.bboxArray.boxes.size();
                if (num == 0)
                {
                    return;
                }
                Vector2<double> start_pose(msg.odometry.pose.pose.position.x, msg.odometry.pose.pose.position.y);
                double ori_x, ori_y;
                vis_edges.points.clear();
                vis_lines.points.clear();
                vis_orientation.points.clear();
                vis_points.points.clear();
                //set oritenation
                double x, y, z, w;
                vis_edges.points.clear();
                vis_lines.points.clear();
                vis_orientation.points.clear();
                vis_points.points.clear();
                //set oritenation
                x = msg.odometry.pose.pose.orientation.x;
                y = msg.odometry.pose.pose.orientation.y;
                z = msg.odometry.pose.pose.orientation.z;
                w = msg.odometry.pose.pose.orientation.w;
                Eigen::Quaterniond quaternion4(w, x, y, z);
                Eigen::Vector3d eulerAngle = quaternion4.matrix().eulerAngles(0, 1, 2);

                ori_y = sin(eulerAngle[2]);
                ori_x = cos(eulerAngle[2]);

                Vector2<double> orientation(ori_x, ori_y);
                std::cout << "num of cones = " << num << std::endl;
                if (ppnode.race_count == 1)
                {
                    if (num < 20)
                    {
                        path_maker.setMinDepth(4);
                        path_maker.setMaxDepth(6);
                    }
                    else
                    {
                        path_maker.setMinDepth(MinDepth);
                        path_maker.setMaxDepth(MaxDepth);
                    }

                    path_maker.setStartposition(start_pose, orientation);
                    std::cout << "set pose" << std::endl;
                    if (ppnode.initPath == true)
                    {
                        //load cones
                        cones.clear();
                        for (int i = 0; i < num; ++i)
                        {
                            Vector2<double> tmp_cone(msg.bboxArray.boxes[i].pose.position.x, msg.bboxArray.boxes[i].pose.position.y);
                            tmp_cone.color = NOCOLOR_CONE;
                            cones.push_back(tmp_cone);
                        }
                    }
                    std::cout << "load cones" << std::endl;
                    path_maker.setVertices(cones);
                    path_maker.process();
                    std::cout << "generate finished" << std::endl;
                    // way = path_maker.getWay();
                    way = path_maker.way_splined;
                    // way = path_maker.points_way;
                    if (way.size() > 0)
                    {
                        ppnode.waypoints_record.push_back(path_maker.points_way[1]);
                    }
                    else
                    {
                        return;
                    }
                }
                else
                {
                    int best_mark = 0;
                    double min_dist = std::numeric_limits<double>::max();
                    Vector2<double> pose_ori = start_pose + orientation;
                    for (int i = 0; i < ppnode.waypoints_record.size(); ++i)
                    {
                        double tmp_angle = getAngle<double>(pose_ori, start_pose, ppnode.waypoints_record[i]);
                        if (tmp_angle + EPS < M_PI / 2 || tmp_angle + EPS > M_PI * 3 / 2)
                        {
                            double tmp_dist = start_pose.dist(ppnode.waypoints_record[i]);
                            if (tmp_dist < EPS)
                            {
                                continue;
                            }
                            if (tmp_dist < min_dist)
                            {
                                min_dist = tmp_dist;
                                best_mark = i;
                            }
                        }
                    }
                    way.clear();
                    way.push_back(start_pose);
                    int maxlen = ppnode.waypoints_record.size();
                    for (int i = 0; i < MaxDepth; ++i)
                    {
                        way.push_back(ppnode.waypoints_record[(best_mark + i) % maxlen]);
                    }
                    std::vector<double> s_list, x_list, y_list;
                    s_list.push_back(0);
                    for (int i = 0; i < way.size(); ++i)
                    {
                        x_list.push_back(way[i].x);
                        y_list.push_back(way[i].y);
                        if (i != 0)
                        {
                            double dist = way[i].dist(way[i - 1]);
                            dist += s_list[i - 1];
                            s_list.push_back(dist);
                        }
                    }
                    tk::spline spline_x, spline_y;
                    way.clear();
                    spline_x.set_points(s_list, x_list, true);
                    spline_y.set_points(s_list, y_list, true);
                    double total_len = s_list[int(s_list.size()) - 1];
                    for (double len = 0; len + EPS < total_len; len += SubLength)
                    {
                        Vector2<double> p(spline_x(len), spline_y(len));
                        way.push_back(p);
                    }
                }
                //set position and orientation

                if (way.size() > 0)
                {
                    //如果是初始化路径
                    std::cout << "generate path size()=" << way.size() << std::endl;
                }
                else
                {
                    std::cout << "Error:way is empty!" << std::endl;
                    return;
                }
                //ppnode.waypoints_record.push_back(way[0]);
                time_end = clock();
                way_points.waypoints.clear();
                way_points.header.frame_id = "map";
                way_points.header.stamp = ros::Time::now();
                autoware_msgs::waypoint tmp_point;
                for (int i = 0; i < way.size(); ++i)
                {
                    tmp_point.pose.pose.position.x = way[i].x;
                    tmp_point.pose.pose.position.y = way[i].y;
                    tmp_point.pose.pose.position.z = 0;
                    way_points.waypoints.push_back(tmp_point);
                }
                ppnode.pub2pure_pursuit.publish(way_points);
                cout << "time cost:" << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << endl;
                std::cout << "---------" << std::endl;

                for (int i = 0; i < way.size() - 1; ++i)
                {
                    geometry_msgs::Point pp1, pp2;
                    pp1.x = way[i].x;
                    pp1.y = way[i].y;
                    pp1.z = 0;
                    pp2.x = way[i + 1].x;
                    pp2.y = way[i + 1].y;
                    pp2.z = 0;
                    vis_lines.points.push_back(pp1);
                    vis_lines.points.push_back(pp2);
                }
                std::vector<Edge<double>> edges = path_maker.triangulation.getEdges();
                for (auto e = begin(edges); e != end(edges); ++e)
                {
                    geometry_msgs::Point p1, p2;
                    p1.x = e->p1.x;
                    p1.y = e->p1.y;
                    p2.x = e->p2.x;
                    p2.y = e->p2.y;
                    p1.z = p2.z = 0;
                    vis_edges.points.push_back(p1);
                    vis_edges.points.push_back(p2);
                }
                for (int i = 0; i < way.size(); ++i)
                {
                    geometry_msgs::Point pp;
                    pp.x = way[i].x;
                    pp.y = way[i].y;
                    pp.z = 0;
                    vis_points.points.push_back(pp);
                }
                geometry_msgs::Point ori_point;
                ori_point.x = start_pose.x + ori_x;
                ori_point.y = start_pose.y + ori_y;
                ori_point.z = 0;
                vis_orientation.points.push_back(ori_point);

                ppnode.pub.publish(vis_orientation);
                ppnode.pub.publish(vis_points);
                ppnode.pub.publish(vis_lines);
                ppnode.pub.publish(vis_edges);
                data_fout << time_count << "," << path_maker.num_vertices << "," << path_maker.num_candidate_trees << ","
                          << path_maker.triangulation.time_generateTriangles << "," << path_maker.time_init << "," << path_maker.time_trigraph << "," << path_maker.time_select_startpoints << ","
                          << path_maker.time_generatePath << "," << path_maker.time_dfsTime << "," << path_maker.time_estimateValue << std::endl;
                return;
            }
        }
    }
    catch (...)
    {
        std::cout << "Exception:" << std::endl;
        return;
    }
}

RosPubSub::RosPubSub()
{
    sub_circle_race = nh.subscribe("/CircleIndicator", 1, circle_race_callback);
    sub_back_end = nh.subscribe("/back_end_output", 1, generate_path_callback);
    pub2pure_pursuit = nh.advertise<autoware_msgs::lane>("/path4pure_pursuit", 100);
    pub2lqr = nh.advertise<autoware_msgs::lane>("/path4lqr", 100);
}

#endif