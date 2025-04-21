#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h>
#include <complex>
#include "../utils/record_voice.h"
#include <fftw3.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include "voice_process_pkg/Pair.h"
#include "voice_process_pkg/PairArray.h"
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>



#define M_PI  3.14159265358979323846

#define MIC_RADIUS 0.035       // 麦克风半径  单位：m
#define MIC_NUM 6              // 麦克风数量
#define MUSIC_RESOLUTION 1     // music算法分辨率
#define SAMPLE_RATE 16000      // 采样频率
#define FFT_SIZE 512           
#define NUM_SNAPSHOTS 15       // 快照数
#define FRAME_LEN 1024         // 分帧长度
#define HOP_LEN 512            // 滑动窗口移动长度


class MUSIC_solver
{
public:
    MUSIC_solver()
    {
        pcm_sub_ =  nh_.subscribe("origin_voice", 1, &MUSIC_solver::pcmCallback, this);
        spectrum_vector_pub_ = nh_.advertise<voice_process_pkg::PairArray>("MUSIC/spectrum_vector", 10);
        music_thread_ = std::thread(&MUSIC_solver::MUSIC_Solver_thread, this);;
    };

    ~MUSIC_solver()
    {
        is_running = false;
        pcm_buf_cv.notify_one(); // 防止子线程卡死
        if(music_thread_.joinable())
        {
            music_thread_.join(); 
        }
           
    }

    // 计算导向矢量
    std::vector<std::complex<double>> cpSteeringVector(
        double theta_deg,         // 假设声源方位角（角度制）
        double frequency,         // 所选频率
        int num_mics,             // 麦克风数量
        double radius,            // 阵列半径（单位：米）
        double sound_speed = 343  // 默认声速 343 m/s
    );


    // 计算导向向量(eigen)
    Eigen::VectorXcd cpSteeringVectorEigen(
        double theta_deg,
        double frequency,
        int num_mics,
        double radius,
        double sound_speed = 343
    );

    // 计算协方差矩阵
    Eigen::MatrixXcd cpCovarianceMatrix(const std::vector<Eigen::VectorXcd>& snapshots);

    // 判断矩阵是否为Hermitian
    bool isHermitian(const Eigen::MatrixXcd& R, double tol = 1e-10);

    // 特征值分解
    void Decompose(const Eigen::MatrixXcd& R, 
        Eigen::MatrixXcd& signalSubspace, 
        Eigen::MatrixXcd& noiseSubspace,
        int num_signal_sources);

    // 估计方位
    double estimateDOA(const std::vector<std::pair<double, double>>& spectrum_vector);

    // 计算MUSIC谱
    std::vector<std::pair<double, double>> MusicScan(   
        double angle_resolution, 
        int num_mics, 
        double frequency,  
        double radius,
        Eigen::MatrixXcd& signalSubspace, 
        Eigen::MatrixXcd& noiseSubspace
    );

    // 返回hannwindow
    static std::vector<double> HannWindow(int N);

    // 对加窗函数进行fft
    std::vector<std::vector<std::complex<double>>>
    FFTMultiChannel(std::vector<std::vector<double>> AddWinVector);

    Eigen::MatrixXcd ComputeCovarianceMatrix(
        const std::vector<std::vector<std::complex<double>>>& freq_result,
        int freq_index);
    

    // 提取多帧 snapshot
    std::vector<Eigen::VectorXcd> ExtractSnapshotsAll(
        const std::vector<std::vector<std::complex<double>>>& freq_result,
        int bin_index,
        int num_snapshots);

    // 将回调信息加窗返回二维数组
    std::vector<std::vector<double>> AddHannWindow(const std_msgs::Float64MultiArray& msg, int num_channels);

    // pcm文件回调函数
    void pcmCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    // 求解角度器
    void MUSIC_Solve_angle(const std_msgs::Float64MultiArray::ConstPtr& msg);

    void MUSIC_Solve_angle(const std_msgs::Float64MultiArray::ConstPtr& msg, std::vector<std::pair<double, double>> &spectrum_vec_buf);

    void MUSIC_Solve_angle(
        const std::vector<std::vector<double>>& signal, 
        std::vector<std::pair<double, double>> &spectrum_vec_buf);

    // 自动选择bin_index ( 找出人声频段（300–3000 Hz）内能量最大的频点 )
    int AutoSelectBinIndex(
        const std::vector<std::vector<std::complex<double>>>& freq_result,
        double sample_rate,
        int fft_size,
        double min_freq = 300.0,
        double max_freq = 3000.0);

    // 滑动窗口分帧
    void FrameSlip(
    const std::vector<std::vector<double>>& signal, 
    int frame_len, int hop_len);

    // 单通道分帧 (暂时舍弃)
    std::vector<std::vector<double>> FrameSignal(
        const std::vector<double>& signal, 
        int frame_len, 
        int hop_len);
    
    // 解析msg消息
    std::vector<std::vector<double>> 
    RecordMsg(const std_msgs::Float64MultiArray& msg, int num_channels);

    // 加窗
    std::vector<std::vector<double>>
    AddHannWindow(const std::vector<std::vector<double>> signal);
    
    // 求解线程
    void MUSIC_Solver_thread();


private:
    ros::Subscriber pcm_sub_;
    ros::Publisher spectrum_vector_pub_;
    ros::NodeHandle nh_;

    std::mutex silp_window_mutex_;
    std::queue<std::vector<std::vector<double>>> signal_queue;
    std::queue<std::vector<std::vector<double>>> pcm_buf_queue;
    std::mutex pcm_buf_mutex;
    std::condition_variable pcm_buf_cv;
    bool is_running = true;
    std::thread music_thread_;
    

};