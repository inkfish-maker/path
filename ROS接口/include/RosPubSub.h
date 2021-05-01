#ifndef ROSPUBSUB_H
#define ROSPUBSUN_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <back_end/BackEndOutput.h>
#include <autoware_msgs/lane.h>

class RosPubSub
{
public:
    RosPubSub();
    ros::Subscriber sub_circle_race; //接收跑动的圈数
    ros::Subscriber sub_back_end;    //后端发来的桩筒数据
    ros::Publisher pub2pure_pursuit; //发送给纯追踪控制的路径消息
    ros::Publisher pub2lqr;          //发送给lqr控制的路径消息
    ros::NodeHandle nh;
};
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
    ;
}
RosPubSub::RosPubSub()
{
    sub_circle_race = nh.subscribe("/CircleIndicator", 1, circle_race_callback);
    sub_back_end = nh.subscribe("/back_end_output", 1, generate_path_callback);
    pub2pure_pursuit = nh.advertise<autoware_msgs::lane>("/path4pure_pursuit", 100);
    pub2lqr = nh.advertise<autoware_msgs::lane>("/path4lqr", 100);
}

#endif