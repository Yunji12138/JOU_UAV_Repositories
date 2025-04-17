#include "record_voice.h"

// 读取PCM文件并返回二维数组
void PCM_Voice::parsePCM(std::vector<std::vector<double>>& channelData) 
{
    std::ifstream pcm_file(pcmFileName_, std::ios::binary);
    if (!pcm_file.is_open()) {
        std::cerr << "无法打开文件: " << pcmFileName_ << std::endl;
        return;
    }

    // 读取文件大小
    pcm_file.seekg(0,std::ios::end);     //先将指针移动到最后
    size_t file_size = pcm_file.tellg(); //使用tellg计算文件大小
    pcm_file.seekg(0,std::ios::beg);     //再将指针移动回文件开头

    size_t numSamplesTotal = file_size / pcmSample_;
    size_t numFrames = numSamplesTotal / pcmNumChannels_;

    channelData.resize(pcmNumChannels_);
    for (int ch = 0; ch < pcmNumChannels_; ++ch) {
        channelData[ch].resize(numFrames);
    }


    // 从文件中按帧读取数据，每帧包含 numChannels 个通道样本
    for (size_t i = 0; i < numFrames; ++i) {
        for (int ch = 0; ch < pcmNumChannels_; ++ch) {
            int16_t sample;  // 临时变量用于存储读取的 16-bit 整数样本
            pcm_file.read(reinterpret_cast<char*>(&sample), sizeof(int16_t)); // 读取一个样本
            channelData[ch][i] = static_cast<double>(sample); // 转换为 double 存储到对应通道
        }
    }
    
}

void PCM_Voice::parsePCM()
{
    std::ifstream pcm_file(pcmFileName_, std::ios::binary);
    if (!pcm_file.is_open()) {
        std::cerr << "无法打开文件: " << pcmFileName_ << std::endl;
        return;
    }

    // 读取文件大小
    pcm_file.seekg(0,std::ios::end);     //先将指针移动到最后
    size_t file_size = pcm_file.tellg(); //使用tellg计算文件大小
    pcm_file.seekg(0,std::ios::beg);     //再将指针移动回文件开头

    size_t numSamplesTotal = file_size / pcmSample_;
    size_t numFrames = numSamplesTotal / pcmNumChannels_;
    pcmNumFrames_ = numFrames;

    pcmVoiceVector_.resize(pcmNumChannels_);
    for (int ch = 0; ch < pcmNumChannels_; ++ch) {
        pcmVoiceVector_[ch].resize(numFrames);
    }

    // 从文件中按帧读取数据，每帧包含 numChannels 个通道样本
    for (size_t i = 0; i < numFrames; ++i) {
        for (int ch = 0; ch < pcmNumChannels_; ++ch) {
            int16_t sample;  // 临时变量用于存储读取的 16-bit 整数样本
            pcm_file.read(reinterpret_cast<char*>(&sample), sizeof(int16_t)); // 读取一个样本
            pcmVoiceVector_[ch][i] = static_cast<double>(sample); // 转换为 double 存储到对应通道
        }
    }
     // 输出读取完成的信息
     std::cout << "Read " << numFrames << " frames, " << pcmNumChannels_ << " channels." << std::endl;
}
