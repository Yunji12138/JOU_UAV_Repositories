#pragma once
#include <ros/ros.h>
#include "MUSIC.hpp"

using namespace Eigen;

class voice_algorithm
{
public:
    voice_algorithm() : MicoNum_(6),SamplingNum_(1000),DOA(0),Phi(0)
    {}

    void Output_angle()
    {
        std::cout << "DOA:" << DOA << "Phi: " << Phi << std::endl;
    }

    void set_Mico_signals(const MatrixXcd& signals)
    {
        signals_ = signals;
    }

    void T_MUSIC_algorithm(int signalCount, float ArraryRadius, float frequency)
    {
        // MatrixXcd R = vMUSIC::computeCovarianceMatrix(signals_);
        // std::pair<float,float> doa = vMUSIC::findDOA(R, signalCount, ArraryRadius, frequency, MicoNum_);
        // DOA = doa.first;
        // Phi = doa.second;
    }

private:
    ros::NodeHandle nh_;
    float DOA;
    float Phi;
    int MicoNum_;
    int SamplingNum_;
    Eigen::MatrixXcd signals_; //麦克风信号矩阵

};