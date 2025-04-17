#include "matplotlibcpp.h"
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <std_msgs/Float64MultiArray.h>



class MatplotDraw
{
public:
    MatplotDraw()
    {
        voice_sub_ = nh_.subscribe("origin_voice", 1, &MatplotDraw::voiceCallback, this);
        plot_thread_ = std::thread(&MatplotDraw::matplot_draw_thread, this);
    }

    ~MatplotDraw()
    {
        running_ = false;             // 设置停止标志
        plot_cv_.notify_all();        // 唤醒线程以便它检测 running_ 并退出
        if (plot_thread_.joinable())  // 等待线程安全退出
            plot_thread_.join();
    }

    void voiceCallback(const std_msgs::Float64MultiArray::ConstPtr& voice_msg);
    void plotMultiChannel(const std::vector<std::vector<double>>& channelData);
    void matplot_draw_thread();

private:
    ros::Subscriber voice_sub_;
    ros::NodeHandle nh_;
    std::mutex plot_mutex_;
    std::queue<std::vector<std::vector<double>>> plot_queue_;
    std::condition_variable plot_cv_;        //条件变量
    bool running_ = true;                    //控制多线程退出
    std::thread plot_thread_;
    
};
   




