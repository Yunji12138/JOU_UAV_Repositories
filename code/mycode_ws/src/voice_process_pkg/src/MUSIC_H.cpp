#include "MUSIC_H.h"

/*
处理流程：PCM → 加窗 → FFT → 频点选择 → 快照 → MUSIC 扫描
*/


std::vector<std::complex<double>> MUSIC_solver::cpSteeringVector(
    double theta_deg,         // 假设声源方位角（角度制）
    double frequency,         // 所选频率
    int num_mics,             // 麦克风数量
    double radius,            // 阵列半径（单位：米）
    double sound_speed  // 默认声速 343 m/s
)
{
    double theta_rad = theta_deg * (M_PI / 180.0);
    std::vector<std::complex<double>> SteeringVector;
    SteeringVector.resize(num_mics);


    for(int i = 0; i < num_mics; i++)
    {
        double Phi = (2 * M_PI * i) / num_mics;
        double X_position = radius * cos(Phi);
        double Y_position = radius * sin(Phi);
        double Tau = (X_position * cos(theta_rad) + Y_position * sin(theta_rad)) / sound_speed;
        double Phase_shift = -2 * M_PI *  frequency * Tau;
        std::complex<double> ejphi = exp(std::complex<double>(0, Phase_shift));
        SteeringVector[i] = ejphi;
    }

    return SteeringVector;
}


Eigen::MatrixXcd MUSIC_solver::cpCovarianceMatrix(const std::vector<Eigen::VectorXcd>& snapshots)
{
    if (snapshots.empty())
    {
        std::cerr << "Error: Snapshot vector is empty!" << std::endl;
        return Eigen::MatrixXcd();  // 返回空矩阵
    }

    int num_mics = snapshots[0].size();
    int num_snapshots = snapshots.size();

    Eigen::MatrixXcd R = Eigen::MatrixXcd::Zero(num_mics,num_mics);

    for(const auto& x : snapshots)
    {
        R += x * x.adjoint(); 
    }

    R = R / static_cast<double>(num_snapshots);

    return R;
}

// 计算特征值，求解声源空间和噪声空间
void MUSIC_solver::Decompose(const Eigen::MatrixXcd& R, 
    Eigen::MatrixXcd& signalSubspace, 
    Eigen::MatrixXcd& noiseSubspace,
    int num_signal_sources)
{
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcd> solver(R);
    if(solver.info() != Eigen::Success)
    {
        std::cerr << "Eigen decomposition failed!" << std::endl;
        return;
    }

    // 所有特征值和特征向量
    Eigen::VectorXd eigenvalues = solver.eigenvalues(); // 实数
    Eigen::MatrixXcd eigenvectors = solver.eigenvectors();

    int num_mics = R.rows();
    int num_noise_sources = num_mics - num_signal_sources;

    signalSubspace = eigenvectors.rightCols(num_signal_sources);  // 取最后几列
    noiseSubspace  = eigenvectors.leftCols(num_noise_sources);   // 取最小几个，对应噪声子空间
}


// 计算导向向量
Eigen::VectorXcd MUSIC_solver::cpSteeringVectorEigen(
    double theta_deg,
    double frequency,
    int num_mics,
    double radius,
    double sound_speed
)
{
    double theta_rad = theta_deg * M_PI / 180.0;
    Eigen::VectorXcd steering(num_mics);

    for (int i = 0; i < num_mics; ++i)
    {
        double phi_i = (2.0 * M_PI * i) / num_mics;

        double delay = radius * cos(theta_rad - phi_i) / sound_speed;

        double phase = -2.0 * M_PI * frequency * delay;

        steering(i) = std::exp(std::complex<double>(0, phase));
    }

    return steering;
}


// 谱扫描
std::vector<std::pair<double, double>> MUSIC_solver::MusicScan
(   double angle_resolution, 
    int num_mics, 
    double frequency,  
    double radius,
    Eigen::MatrixXcd& signalSubspace, 
    Eigen::MatrixXcd& noiseSubspace
)
{
    std::vector<std::pair<double, double>> spectrum_vector;
    for(double theta = 0; theta < 360; theta += angle_resolution)
    {
        Eigen::VectorXcd a_theta = MUSIC_solver::cpSteeringVectorEigen(theta, frequency, num_mics, radius);       //求解导向矢量
        Eigen::VectorXcd EnH_a = noiseSubspace.adjoint() * a_theta;               // 投影到噪声子空间
        double pseudospectrum = 1.0 / EnH_a.squaredNorm();                        //计算谱值
        std::pair<double, double> spectrum_value(theta,pseudospectrum);
        spectrum_vector.push_back(spectrum_value);
    }

    return spectrum_vector;
}


// 估算角度
double MUSIC_solver::estimateDOA(const std::vector<std::pair<double, double>>& spectrum_vector)
{
    double max_value = -1.0;
    double max_theta = 0.0;

    for (const auto& entry : spectrum_vector)
    {
        if (entry.second > max_value)
        {
            max_value = entry.second;
            max_theta = entry.first;
        }
    }

    return max_theta;
}

// debug func
bool MUSIC_solver::isHermitian(const Eigen::MatrixXcd& R, double tol)
{
    int rows = R.rows();
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < rows; ++j)
        {
            if (std::abs(R(i, j) - std::conj(R(j, i))) > tol)
            {
                return false;
            }
        }
    }
    return true;
}



std::vector<double> MUSIC_solver::HannWindow(int N)
{
    std::vector<double> hann_window;
    hann_window.resize(N);
    for(int n = 0; n < N ; n++)
    {
        double tmp = (2 * M_PI * n) / (N - 1);
        hann_window[n] = 0.5 * (1 - std::cos(tmp));
    }

    return hann_window;
}

// 解析msg消息
std::vector<std::vector<double>> 
MUSIC_solver::RecordMsg(const std_msgs::Float64MultiArray& msg, int num_channels)
{
    int total_samples = msg.data.size(); // 获取总数据量，即所有通道的所有帧数
    // 检查消息的布局，期望为2维数据（通道 x 帧数）
    if (msg.layout.dim.size() != 2) {
        std::cerr << "Invalid message layout. Expecting 2 dimensions." << std::endl;
        return {}; // 返回空的二维数组
    }

    // 获取通道数和每个通道的帧数
    int channels = msg.layout.dim[0].size;  // 通道数
    int frames_per_channel = msg.layout.dim[1].size;  // 每个通道的帧数

    // 检查通道数和数据大小是否与期望值一致
    if (channels != num_channels || total_samples != channels * frames_per_channel) {
        std::cerr << "Mismatch in channel/frame sizes." << std::endl;
        return {}; // 返回空的二维数组
    }

    // 初始化一个二维数组，用于存储按通道排列的数据
    std::vector<std::vector<double>> channel_data(num_channels, std::vector<double>(frames_per_channel));

    // 填充二维数组，将msg.data中的数据按照通道和帧划分
    for (int ch = 0; ch < num_channels; ++ch) {
        for (int frame = 0; frame < frames_per_channel; ++frame) {
            channel_data[ch][frame] = msg.data[ch * frames_per_channel + frame]; // 按通道和帧填充数据
        }
    }

    return channel_data;
}


// 加窗 msg版
std::vector<std::vector<double>> 
MUSIC_solver::AddHannWindow(const std_msgs::Float64MultiArray& msg, int num_channels)
{
    int total_samples = msg.data.size(); // 获取总数据量，即所有通道的所有帧数
    // 检查消息的布局，期望为2维数据（通道 x 帧数）
    if (msg.layout.dim.size() != 2) {
        std::cerr << "Invalid message layout. Expecting 2 dimensions." << std::endl;
        return {}; // 返回空的二维数组
    }

    // 获取通道数和每个通道的帧数
    int channels = msg.layout.dim[0].size;  // 通道数
    int frames_per_channel = msg.layout.dim[1].size;  // 每个通道的帧数

    // 检查通道数和数据大小是否与期望值一致
    if (channels != num_channels || total_samples != channels * frames_per_channel) {
        std::cerr << "Mismatch in channel/frame sizes." << std::endl;
        return {}; // 返回空的二维数组
    }

    // 初始化一个二维数组，用于存储按通道排列的数据
    std::vector<std::vector<double>> channel_data(num_channels, std::vector<double>(frames_per_channel));

    // 填充二维数组，将msg.data中的数据按照通道和帧划分
    for (int ch = 0; ch < num_channels; ++ch) {
        for (int frame = 0; frame < frames_per_channel; ++frame) {
            channel_data[ch][frame] = msg.data[ch * frames_per_channel + frame]; // 按通道和帧填充数据
        }
    }

    // 生成Hann窗，大小为每个通道的帧数
    std::vector<double> hann_window = HannWindow(frames_per_channel);
    
    // 对每个通道的数据应用Hann窗
    for (int ch = 0; ch < num_channels; ++ch) {
        for (int n = 0; n < frames_per_channel; ++n) {
            channel_data[ch][n] *= hann_window[n]; // 将每个数据点乘以对应的Hann窗系数
        }
    }

    // 返回加窗后的数据
    return channel_data;
}

// 加窗 信号版
std::vector<std::vector<double>>
MUSIC_solver::AddHannWindow(const std::vector<std::vector<double>> signal)
{
    int num_channels = signal.size();
    int frame_len = signal[0].size();

    // 创建加窗信号
    std::vector<std::vector<double>> add_windows_signal(num_channels, std::vector<double>(frame_len));
    // 生成Hann窗，大小为每个通道的帧数
    std::vector<double> hann_window = HannWindow(frame_len);
    
    // 对每个通道的数据应用Hann窗
    for (int ch = 0; ch < signal.size(); ++ch) {
        for (int n = 0; n < signal[0].size(); ++n)
        {
            add_windows_signal[ch][n] = signal[ch][n] * hann_window[n]; // 将每个数据点乘以对应的Hann窗系数
        }
    }

    return add_windows_signal;
}


// fft
std::vector<std::vector<std::complex<double>>>
MUSIC_solver::FFTMultiChannel(std::vector<std::vector<double>> AddWinVector)
{
    std::vector<std::vector<std::complex<double>>> freq_result;
    int N = AddWinVector[0].size();


    for(int i = 0; i < AddWinVector.size(); i++)
    {
        // 创建空间
        double* in = fftw_alloc_real(N); 
        fftw_complex* out = fftw_alloc_complex(N / 2 + 1);

        // 将单通道数据放入in数组
        std::vector<double> in_real = AddWinVector[i];
        for(int j = 0; j < N; j++)
        {
            in[j] = in_real[j];
        }

        fftw_plan plan = fftw_plan_dft_r2c_1d(N, in, out, FFTW_ESTIMATE);
        fftw_execute(plan);

        std::vector<std::complex<double>> FFTVector(N / 2 + 1);
        for (int index = 0; index < N / 2 + 1; ++index)
        {
            FFTVector[index] = std::complex<double>(out[index][0], out[index][1]);
        }
        
        freq_result.push_back(FFTVector);

        //释放资源
        fftw_destroy_plan(plan);
        fftw_free(in);
        fftw_free(out);
    }

    return freq_result;
}



std::vector<Eigen::VectorXcd> MUSIC_solver::ExtractSnapshotsAll(
    const std::vector<std::vector<std::complex<double>>>& freq_result,
    int bin_index,
    int num_snapshots)
{
    std::vector<Eigen::VectorXcd> snapshots;

    int num_channels = freq_result.size();
    int total_snapshots = freq_result[0].size();  // 每个通道有多少帧

    if (num_snapshots > total_snapshots)
    {
        std::cerr << "Error: Requested snapshots exceed available frames!" << std::endl;
        return snapshots;
    }

    for (int snap = 0; snap < num_snapshots; ++snap)
    {
        Eigen::VectorXcd snapshot(num_channels);
        for (int ch = 0; ch < num_channels; ++ch)
        {
            snapshot[ch] = freq_result[ch][bin_index + snap];  // 滑动窗口取 snapshot
        }
        snapshots.push_back(snapshot);
    }

    return snapshots;
}


// 自动求解bin_index
int MUSIC_solver::AutoSelectBinIndex(
    const std::vector<std::vector<std::complex<double>>>& freq_result,
    double sample_rate,
    int fft_size,
    double min_freq,
    double max_freq)
{
    int num_channels = freq_result.size();
    int num_bin = freq_result[0].size();

    // 求取bin的范围
    int min_bin = min_freq * fft_size / sample_rate;
    int max_bin = max_freq * fft_size / sample_rate;

    double max_energy = 0.0;
    int best_bin = 0;

    for(int bin = min_bin; bin < max_bin; bin++)
    {
        double energy = 0.0;
        for(int ch = 0; ch < num_channels; ch++)
        {
            energy = energy + std::norm(freq_result[ch][bin]);
        }
        if(energy > max_energy)
        {
            max_energy = energy;
            best_bin = bin;
        }

    }

    return best_bin;

}



void MUSIC_solver::MUSIC_Solve_angle(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int bin_index = 0;   // 对应实际频率: Fs / N * 10  ( Fs 采样率 N FFT点数 )
    
    std::vector<std::vector<double>> channels_data = this -> AddHannWindow(*msg,6);
    std::vector<std::vector<std::complex<double>>> freq_result = this -> FFTMultiChannel(channels_data);
    bin_index = AutoSelectBinIndex(freq_result,SAMPLE_RATE,FFT_SIZE);
    auto snapshots = this -> ExtractSnapshotsAll(freq_result,bin_index,NUM_SNAPSHOTS);

    Eigen::MatrixXcd R = this -> cpCovarianceMatrix(snapshots);

    Eigen::MatrixXcd signalSubspace; 
    Eigen::MatrixXcd noiseSubspace;

    this -> Decompose(R, signalSubspace, noiseSubspace,1);
    auto spectrum_vector = this -> MusicScan(MUSIC_RESOLUTION,MIC_NUM,100,MIC_RADIUS,signalSubspace,noiseSubspace);

    double voice_angle = this -> estimateDOA(spectrum_vector);

    std::cout << "声源角度为：" << voice_angle << std::endl;

}

void MUSIC_solver::MUSIC_Solve_angle(const std_msgs::Float64MultiArray::ConstPtr& msg, std::vector<std::pair<double, double>> &spectrum_vec_buf)
{
    int bin_index = 0;   // 对应实际频率: Fs / N * 10  ( Fs 采样率 N FFT点数 )
    std::vector<std::vector<double>> channels_data = this -> AddHannWindow(*msg,6);
    std::vector<std::vector<std::complex<double>>> freq_result = this ->FFTMultiChannel(channels_data);
    bin_index = AutoSelectBinIndex(freq_result,SAMPLE_RATE,FFT_SIZE);
    auto snapshots = this -> ExtractSnapshotsAll(freq_result,bin_index,NUM_SNAPSHOTS);

    Eigen::MatrixXcd R = this ->cpCovarianceMatrix(snapshots);

    Eigen::MatrixXcd signalSubspace; 
    Eigen::MatrixXcd noiseSubspace;

    this -> Decompose(R, signalSubspace, noiseSubspace,1);
    auto spectrum_vector = this -> MusicScan(MUSIC_RESOLUTION,MIC_NUM,100,MIC_RADIUS,signalSubspace,noiseSubspace);
    spectrum_vec_buf = spectrum_vector;
    
    double voice_angle = this -> estimateDOA(spectrum_vector);

    std::cout << "声源角度为：" << voice_angle << std::endl;

}


void MUSIC_solver::MUSIC_Solve_angle(const std::vector<std::vector<double>>& signal, std::vector<std::pair<double, double>> &spectrum_vec_buf)
{
    int bin_index = 0;   // 对应实际频率: Fs / N * 10  ( Fs 采样率 N FFT点数 )
    std::vector<std::vector<double>> channels_data = this -> AddHannWindow(signal);
    std::vector<std::vector<std::complex<double>>> freq_result = this ->FFTMultiChannel(channels_data);
    bin_index = AutoSelectBinIndex(freq_result,SAMPLE_RATE,FFT_SIZE);
    auto snapshots = this -> ExtractSnapshotsAll(freq_result,bin_index,NUM_SNAPSHOTS);

    Eigen::MatrixXcd R = this ->cpCovarianceMatrix(snapshots);

    Eigen::MatrixXcd signalSubspace; 
    Eigen::MatrixXcd noiseSubspace;

    this -> Decompose(R, signalSubspace, noiseSubspace,1);
    auto spectrum_vector = this -> MusicScan(MUSIC_RESOLUTION,MIC_NUM,100,MIC_RADIUS,signalSubspace,noiseSubspace);
    spectrum_vec_buf = spectrum_vector;
    
    double voice_angle = this -> estimateDOA(spectrum_vector);

    std::cout << "声源角度为：" << voice_angle << std::endl;

}

// test
std::vector<std::vector<double>> generateIdealPlaneWave(
    int num_mics,
    double radius,
    double theta_deg,
    double f_signal,
    double fs,
    double duration,
    double sound_speed
) {
    int num_samples = static_cast<int>(duration * fs);
    double theta_rad = theta_deg * M_PI / 180.0;

    std::vector<std::vector<double>> mic_signals(num_mics, std::vector<double>(num_samples));

    for (int mic = 0; mic < num_mics; ++mic) {
        double phi = 2.0 * M_PI * mic / num_mics;
        double delay = (radius * cos(phi - theta_rad)) / sound_speed;

        for (int n = 0; n < num_samples; ++n) {
            double t = n / fs;
            mic_signals[mic][n] = sin(2.0 * M_PI * f_signal * (t - delay));
        }
    }

    return mic_signals;
}

std::vector<std::vector<double>> MUSIC_solver::FrameSignal(const std::vector<double>& signal, int frame_len, int hop_len) 
{
    std::vector<std::vector<double>> frames;
    if(signal.size() == 0 || frame_len > signal.size())
    {
        return frames;
    }

    int num_frames = (signal.size() - frame_len) / hop_len + 1;
    std::vector<double> window = HannWindow(frame_len);

    // frames(num_frames, std::vector<double>(frame_len));

    for (int i = 0; i < num_frames; ++i) 
    {
        int start = i * hop_len;
        for (int j = 0; j < frame_len; ++j) 
        {
            frames[i][j] = signal[start + j] * window[j];
        }
    }

    return frames;
}

// 滑动窗口 - 存入队列
void MUSIC_solver::FrameSlip(
    const std::vector<std::vector<double>>& signal,  
    int frame_len, int hop_len)
{

    if(signal[0].size() == 0 || frame_len > signal[0].size())
    {
        return;
    }
    
    int single_num_frames = (signal[0].size() - frame_len) / hop_len + 1;

    for(int index_all = 0; index_all < single_num_frames; index_all++)
    {
        std::vector<std::vector<double>> silp_signal(signal.size(), std::vector<double>(frame_len));
        int start = index_all * hop_len;
        for(int i = 0; i < signal.size(); i++)
        {
            for(int j = 0; j < frame_len; j++)
            {
                silp_signal[i][j] = signal[i][start + j];
            }
        }
        signal_queue.push(silp_signal);    
    }

}


// 回调函数
void MUSIC_solver::pcmCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::cout << "获得数据，正在解析..." << std::endl;
    std::vector<std::vector<double>> pcm_frame = MUSIC_solver::RecordMsg(*msg,6);
    {
        std::unique_lock<std::mutex> lock(pcm_buf_mutex);
        pcm_buf_queue.push(pcm_frame);
    }
    pcm_buf_cv.notify_one();

}



void MUSIC_solver::MUSIC_Solver_thread()
{
    while (is_running)
    {
        std::vector<std::vector<double>> frame;
        {
            std::unique_lock<std::mutex> lock(pcm_buf_mutex);
            pcm_buf_cv.wait(lock, [this] { return !pcm_buf_queue.empty() || !is_running; });

            if (!is_running && pcm_buf_queue.empty()) break;

            frame = pcm_buf_queue.front();
            pcm_buf_queue.pop();
        }

        // 分帧 + 加窗
        this->FrameSlip(frame, FRAME_LEN, HOP_LEN);

        while (!signal_queue.empty())
        {
            auto ori_signal = signal_queue.front();
            std::vector<std::pair<double, double>> spectrum_buf;
            voice_process_pkg::PairArray spectrum_msg;

            this->MUSIC_Solve_angle(ori_signal,spectrum_buf); // 输出角度,并将谱图数据存入spectrum_buf
            signal_queue.pop();

            for (const auto& p : spectrum_buf)
            {
                voice_process_pkg::Pair one;
                one.first = p.first;
                one.second = p.second;
                spectrum_msg.data.push_back(one);
            }
            spectrum_vector_pub_.publish(spectrum_msg);
        }
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"music_test_node");
    MUSIC_solver m_solver;
    ROS_INFO("进入声源识别线程...");

    ros::spin();
    return 0;

}


    // test music code
    /*
    int num_mics = 6;
    double radius = 0.05;  // 阵列半径 5cm
    double theta_deg = 60; // 入射角
    double f_signal = 2000; // 信号频率 2kHz
    double fs = 16000;     // 采样率 16kHz
    double duration = 0.02; // 采样时长 20ms
    double sound_speed = 343.0;

    auto mic_signals = generateIdealPlaneWave(
        num_mics, radius, theta_deg, f_signal, fs, duration, sound_speed
    );

    int N = mic_signals[0].size();
    auto fft_result = m_solver.FFTMultiChannel(mic_signals);

    int bin_index = m_solver.AutoSelectBinIndex(fft_result,SAMPLE_RATE,FFT_SIZE);
    auto snapshots = m_solver.ExtractSnapshotsAll(fft_result,bin_index,NUM_SNAPSHOTS);

    Eigen::MatrixXcd R = m_solver.cpCovarianceMatrix(snapshots);

    Eigen::MatrixXcd signalSubspace; 
    Eigen::MatrixXcd noiseSubspace;

    m_solver.Decompose(R, signalSubspace, noiseSubspace,1);
    auto spectrum_vector = m_solver.MusicScan(MUSIC_RESOLUTION,MIC_NUM,100,MIC_RADIUS,signalSubspace,noiseSubspace);
    
    double voice_angle = m_solver.estimateDOA(spectrum_vector);

    std::cout << "声源角度为：" << voice_angle << std::endl;
    */


