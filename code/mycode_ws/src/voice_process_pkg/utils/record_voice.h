#include <fstream>
#include <vector>
#include <iostream>

class PCM_Voice
{
public:
    PCM_Voice(char* pcmFileName, int pcmNumChannels, int pcmSample)
    {
        pcmFileName_ = pcmFileName;
        pcmNumChannels_ = pcmNumChannels;
        pcmSample_ = pcmSample;
    }
    void parsePCM(std::vector<std::vector<double>>& channelData);
    void parsePCM();

    std::vector<std::vector<double>> pcmVoiceVector()
    {
        return pcmVoiceVector_;
    }
    int pcmNumChannels()
    {
        return pcmNumChannels_;
    }

    int pcmSample()
    {
        return pcmSample_;
    }

    int pcmNumFrames()
    {
        return pcmNumFrames_;
    }

private:
    int pcmNumChannels_; //通道数（行数据）
    char* pcmFileName_;
    int pcmSample_;  // 单位：字节   eg: 16bit --> 2
    int pcmNumFrames_; //读取文件帧数（列数据）
    std::vector<std::vector<double>> pcmVoiceVector_;
};