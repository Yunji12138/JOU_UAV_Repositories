#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h>
#include <complex>

using namespace Eigen;

namespace vMUSIC
{
    constexpr double PI = 3.141592653589793;
    constexpr double C = 343.0; // 声速（单位：m/s）

    // 计算协方差矩阵
    MatrixXf computeCovarianceMatrix(const MatrixXcd& signals);

    // 获取噪声子空间
    MatrixXcd getNoiseSubspace(const MatrixXcd& R, int signalCount);

    // 计算导向矢量
    VectorXcd computeSteeringVector(float theta_s, float phi_s, float r, float f, int M);

    // 计算MUSIC伪谱值
    double computeMUSICScore(const MatrixXcd& noiseSubspace, const VectorXcd& steeringVector);

    // 估计声源方向（方位角、俯仰角）
    std::pair<float, float> findDOA(const MatrixXcd& R, int signalCount, float r, float f, int M);
};



