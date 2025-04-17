#include "mycode_ws/src/voice_process_pkg/src/MUSIC.hpp"
#include "MUSIC.hpp"

using Eigen::MatrixXf;
using Eigen::MatrixXcd;
using Eigen::VectorXcd;

/*
    R = 1/N * X * X^H
*/
MatrixXcd  vMUSIC::computeCovarianceMatrix(const MatrixXcd& signals)
{
    int N = signals.cols();
    return (signals * signals.adjoint()) / N;
}

MatrixXcd vMUSIC::getNoiseSubspace(const MatrixXcd& R, int signalCount)
{
    Eigen::SelfAdjointEigenSolver<MatrixXcd> eigensolver(R);
    MatrixXcd eigenvectors = eigensolver.eigenvectors();
    return eigenvectors.rightCols(R.cols() - signalCount);
}


VectorXcd vMUSIC::computeSteeringVector(float theta_s, float phi_s, float r, float f, int M)
{
     
    VectorXcd steeringVector(M);  // 创建复数导向矢量(M维)
    double lambda = C / f;       // 计算波长
    double k = 2 * PI / lambda;  // 波数(wavenumber)
    
    for (int m = 0; m < M; ++m) {
        // 计算第m个阵元的位置(假设均匀圆形阵列)
        double angle = (2 * PI * m) / M;  // 阵元的角度位置
        double xm = r * cos(angle);       // x坐标
        double ym = r * sin(angle);       // y坐标
        double zm = 0;                    // z坐标(2D阵列)
        
        // 计算波达时间差(tau)
        double tau = (xm * cos(theta_s * PI / 180) * cos(phi_s * PI / 180) +
                     ym * sin(theta_s * PI / 180) * cos(phi_s * PI / 180) +
                     zm * sin(phi_s * PI / 180)) / C;
        
        // 计算导向矢量的第m个元素
        steeringVector(m) = exp(complex<double>(0, -2 * PI * f * tau));
    }
    return steeringVector;
}

double vMUSIC::computeMUSICScore(const MatrixXcd& noiseSubspace, const VectorXcd& steeringVector)
{
    VectorXcd projection = noiseSubspace.adjoint() * steeringVector;
    return 1.0 / (projection.squaredNorm());
}

std::pair<float, float> vMUSIC::findDOA(const MatrixXcd& R, int signalCount, float r, float f, int M)
{
    MatrixXcd noiseSubspace = getNoiseSubspace(R, signalCount);
    float bestTheta = -180, bestPhi = -90;
    double bestValue = 0;

    for (float theta = -180; theta <= 180; theta += 5.0) {
        for (float phi = -90; phi <= 90; phi += 5.0) {
            VectorXcd steeringVector = computeSteeringVector(theta, phi, r, f, M);
            double value = computeMUSICScore(noiseSubspace, steeringVector);
            if (value > bestValue) {
                bestValue = value;
                bestTheta = theta;
                bestPhi = phi;
            }
        }
    }
    return {bestTheta, bestPhi};
}
