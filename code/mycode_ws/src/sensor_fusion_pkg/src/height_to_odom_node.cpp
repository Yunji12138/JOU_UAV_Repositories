#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher odom_pub;

void heightCallback(const std_msgs::Float64::ConstPtr& msg) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";       // 和 VINS 保持一致

    // 只设置 Z 高度
    odom_msg.pose.pose.position.z = msg->data / 1000.0;
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;

    // 设置单位四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 设置协方差：只有 z 有意义
    for (int i = 0; i < 36; ++i)
        odom_msg.pose.covariance[i] = 99999;

    odom_msg.pose.covariance[2 * 6 + 2] = 0.01;  // z 的协方差

    odom_pub.publish(odom_msg);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "height_to_odom_node");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/height_odom", 10);
    ros::Subscriber height_sub = nh.subscribe("/height_dis", 10, heightCallback);

    ros::spin();
    return 0;
}
