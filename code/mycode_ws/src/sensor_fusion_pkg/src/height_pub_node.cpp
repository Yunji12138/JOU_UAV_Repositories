#include <stdio.h>      //printf()
#include <stdlib.h>     //exit()
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "../lib/DEV_Config.h"
#include "../lib/TOF_Sense.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h> 
#include <mavros_msgs/Altitude.h>
#include <mavros/mavros.h>
#include <sensor_msgs/Range.h>

#define SENSOR_OFFSET_MM 38     // 传感器安装偏移量(mm)
#define MAX_VALID_RANGE_MM 10000  // 有效测量上限

void Handler(int signo)
{
    //System Exit
    printf("\r\nHandler:Program stop\r\n"); 
    DEV_ModuleExit();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "height_pub_node");
    ros::NodeHandle nh;

    ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>("/mavros/distance_sensor/tof", 10);
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("height_dis", 10); 
    // Exception handling:ctrl + c
    signal(SIGINT, Handler);
    DEV_ModuleInit();
    DEV_UART_Init("/dev/ttyS3",921600);
    ros::Rate loop_rate(200);  // 设置频率
    mavros_msgs::Altitude altitude_msg;
    std_msgs::Float64 height_msg;
  
    while (ros::ok())
    {
        // TOF_Inquire_Decoding(0); //Query and decode TOF data 查询获取TOF数据，并进行解码
        // TOF_Active_Decoding(); //Actively acquire TOF data and decode it 主动获取TOF数据，并进行解码
        double raw_mm = static_cast<double>(TOF_Active_Decoding_Return() - SENSOR_OFFSET_MM);
        height_msg.data = raw_mm;
        // std::cout << "height_msg.data: " << height_msg.data << std::endl;
        if(height_msg.data > MAX_VALID_RANGE_MM)
        {
            continue;
        }
        pub.publish(height_msg);

        //test 
        sensor_msgs::Range tof_msg;
        tof_msg.header.stamp = ros::Time::now();
        tof_msg.header.frame_id = "tof_sensor";
        tof_msg.radiation_type = sensor_msgs::Range::INFRARED;
        tof_msg.field_of_view = 0.1;      // 根据传感器实际FOV设置
        tof_msg.min_range = 0.1;          // 最小测量距离（米）
        tof_msg.max_range = MAX_VALID_RANGE_MM / 1000.0;
        tof_msg.range = raw_mm / 1000.0;   // 转换为米

        range_pub.publish(tof_msg);
        loop_rate.sleep(); 
    }
	DEV_ModuleExit();
    return 0; 
}
