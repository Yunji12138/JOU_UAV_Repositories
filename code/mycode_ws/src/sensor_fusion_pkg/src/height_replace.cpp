#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

ros::Publisher fused_odom_pub;
nav_msgs::Odometry latest_odom;
double latest_height = 0.0;
bool odom_received = false;

void imuCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    latest_odom = *msg;
    odom_received = true;
    // std::cout << "latest_odom: " << latest_odom.pose.pose.position.z << std::endl;
}

void heightCallback(const std_msgs::Float64::ConstPtr& msg)
{
    latest_height = msg->data / 1000.0;
    //std::cout << "latest_height: " << latest_height << std::endl;
    if (odom_received)
    {
        nav_msgs::Odometry fused_msg = latest_odom;
        fused_msg.header.stamp = ros::Time::now();

        // 替换 Z 值为定高雷达的高度
        fused_msg.pose.pose.position.z = latest_height;

        // 发布融合后的 odometry
        fused_odom_pub.publish(fused_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_height_fusion_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/vins_fusion/imu_propagate", 10, imuCallback);
    ros::Subscriber height_sub = nh.subscribe("/height_dis", 10, heightCallback);
    fused_odom_pub = nh.advertise<nav_msgs::Odometry>("/vins_fusion/fused_odom", 10);
    ros::spin();
    return 0;
}
