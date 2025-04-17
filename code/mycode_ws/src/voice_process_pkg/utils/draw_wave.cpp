#include "draw_wave.h"
namespace plt = matplotlibcpp;  // Define plt as an alias for the matplotlibcpp namespace

void MatplotDraw::plotMultiChannel(const std::vector<std::vector<double>>& channelData) 
{
    int numChannels = channelData.size();
    int numSamples = channelData[0].size();
    if(numChannels == 0) return;

    std::vector<double> x(numSamples);
    for (int i = 0; i < numSamples; ++i)
    {
        x[i] = i;
    }
        

    plt::clf(); // 清空上一次绘图内容
    // 创建子图
    for (int ch = 0; ch < numChannels; ++ch) {
        plt::subplot(numChannels, 1, ch + 1);  // 子图分布：6 行 1 列
        plt::plot(x, channelData[ch]);
        plt::title("Channel " + std::to_string(ch));
    }

    plt::tight_layout();  // 自动调整子图间距
    plt::pause(0.001);  // 避免阻塞主线程
}

void MatplotDraw::voiceCallback(const std_msgs::Float64MultiArray::ConstPtr& voice_msg)
{
    std::lock_guard<std::mutex> lock(plot_mutex_);
    std::vector<std::vector<double>> plot_vector;
    int rows = voice_msg ->layout.dim[0].size;  //行数
    int cols = voice_msg ->layout.dim[1].size;  //列数

    plot_vector.resize(rows, std::vector<double>(cols, 0.0));

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            int index = i * cols + j; 
            plot_vector[i][j] = voice_msg->data[index];
        }
    }
    plot_queue_.push(plot_vector);
    plot_cv_.notify_one();
}

void MatplotDraw::matplot_draw_thread()
{
    while(running_)
    {
        std::unique_lock<std::mutex> lock(plot_mutex_);
        plot_cv_.wait(lock, [this] { return !plot_queue_.empty() || !running_; });
        if(!running_) break;

        auto voice_vector = plot_queue_.front();
        plot_queue_.pop();
        lock.unlock();
        plotMultiChannel(voice_vector);
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"draw_node");
    MatplotDraw draw;

    ros::AsyncSpinner spinner(1);  // 使用异步 spinner 支持多线程 callback
    spinner.start();

    ros::waitForShutdown();  // 等待 ctrl+c 退出
}