#include "../utils/kill_adb_file.h"
#include "../utils/record_voice.h"
#include "../utils/draw_wave.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fstream>
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#define LOCAL_TMP_FILE "/home/orangepi/mycode_ws/src/voice_process_pkg/voice_save_flie/origin_tmp.pcm"
#define REMOTE_FILE "/data/audio.pcm"
#define RECORD_TIME 1.1  //录制时长
#define CHANNELS 6
#define SAMPLE 16


// 需要在打开设备前杀死程序释放设备
#define KILL_1 "demo"
#define KILL_2 "watch"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_voice_node");
    ros::NodeHandle nh;
    kill_adb_process(KILL_1);
    kill_adb_process(KILL_2);
    ros::Rate rate(50);
    PCM_Voice pcm(LOCAL_TMP_FILE, CHANNELS, (SAMPLE / 8));
    ros::Publisher voice_pub = nh.advertise<std_msgs::Float64MultiArray>("origin_voice", 10);
    while(ros::ok())
    {
        char record_cmd[256];
        //同步阻塞
        snprintf(record_cmd, sizeof(record_cmd), "adb shell \"arecord -D hw:1,0 -f S16_LE -r 16000 -c 6 -d %f %s\"",RECORD_TIME,REMOTE_FILE);
        // 处理adb arecord失败
        int record_cmd_ret = system(record_cmd);
        if (record_cmd_ret != 0) 
        {
            ROS_WARN("record failed. Check au-device is busy.");
            rate.sleep();
            return -1;
        }
    
        // 拉取 audio.pcm 到临时文件
        char pull_cmd[256];
        snprintf(pull_cmd, sizeof(pull_cmd), "adb pull %s %s > /dev/null 2>&1", REMOTE_FILE, LOCAL_TMP_FILE);
    
        // 处理adb pull失败
        int pull_cmd_ret = system(pull_cmd);
        if (pull_cmd_ret != 0) {
            ROS_WARN("adb pull failed. Check device connection.");
            rate.sleep();
            return -1;
        }
    
        ROS_INFO("Pull pcm is done.");
    
        char rm_cmd[256];
        snprintf(rm_cmd, sizeof(rm_cmd), "adb shell rm %s",REMOTE_FILE);
        int rm_cmd_ret = system(rm_cmd);
        if (rm_cmd_ret != 0) 
        {
            ROS_WARN("adb rm failed. Please try again.");
            rate.sleep();
            return -1;
        }
        pcm.parsePCM();
        int rows = pcm.pcmNumChannels();  // 通道数
        int cols = pcm.pcmNumFrames();    // 每个通道的采样帧数（列）
        std::vector<std::vector<double>> pcm_vector = pcm.pcmVoiceVector();
        std_msgs::Float64MultiArray pcm_msg;


        pcm_msg.layout.dim.resize(2); //二维数据
        // 第一维：行 → 通道
        pcm_msg.layout.dim[0].label = "channels";
        pcm_msg.layout.dim[0].size = rows;
        pcm_msg.layout.dim[0].stride = rows * cols;

        // 第二维：列 → 每个通道的帧
        pcm_msg.layout.dim[1].label = "frames";
        pcm_msg.layout.dim[1].size = cols;
        pcm_msg.layout.dim[1].stride = cols;

        pcm_msg.layout.data_offset = 0;      

        pcm_msg.data.clear();
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                pcm_msg.data.push_back(pcm_vector[i][j]);
            }
        }
        voice_pub.publish(pcm_msg);
        kill_adb_process(KILL_1);
        kill_adb_process(KILL_2);
    }

}