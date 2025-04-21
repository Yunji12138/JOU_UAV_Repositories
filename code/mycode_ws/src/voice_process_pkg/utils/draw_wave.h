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
#include "voice_process_pkg/Pair.h"
#include "voice_process_pkg/PairArray.h"



class MatplotDraw
{
public:
    MatplotDraw()
    {
        running_ = true;
        // voice_sub_ = nh_.subscribe("origin_voice", 1, &MatplotDraw::voiceCallback, this);
        spectrum_vector_sub_ = nh_.subscribe("MUSIC/spectrum_vector", 1, &MatplotDraw::spectrumCallback, this);
        // plot_thread_ = std::thread(&MatplotDraw::matplot_draw_thread, this);
        spectrum_thread_ = std::thread(&MatplotDraw::matplot_draw_Spectrum_thread, this);
    }

    ~MatplotDraw()
    {
        running_ = false;             // 设置停止标志
        // plot_cv_.notify_all();        // 唤醒线程以便它检测 running_ 并退出
        spectrum_cv_.notify_all();

        // if (plot_thread_.joinable())  // 等待线程安全退出
        // {
        //     plot_thread_.join();
        // }
        if(spectrum_thread_.joinable())
        {
            spectrum_thread_.join();
        }
    }

    void voiceCallback(const std_msgs::Float64MultiArray::ConstPtr& voice_msg);
    void spectrumCallback(const voice_process_pkg::PairArray::ConstPtr& spectrum_msg);
    void plotMultiChannel(const std::vector<std::vector<double>>& channelData);
    void matplot_draw_thread();
    void matplot_draw_Spectrum_thread();
    void plotSpectrum(const std::vector<std::pair<double,double>>& spectrum_vector);

private:
    ros::Subscriber voice_sub_;
    ros::Subscriber spectrum_vector_sub_;
    ros::NodeHandle nh_;
    std::mutex plot_mutex_;
    std::mutex spectrum_mutex_;
    std::queue<std::vector<std::vector<double>>> plot_queue_;
    std::queue<std::vector<std::pair<double, double>>> spectrum_queue_;
    std::condition_variable plot_cv_;        //条件变量
    std::condition_variable spectrum_cv_;        //条件变量

    bool running_ = true;                    //控制多线程退出

    std::thread plot_thread_;
    std::thread spectrum_thread_;
    
    
};
   




