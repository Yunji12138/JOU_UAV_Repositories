#include "voice_node.hpp"


int main(int argc,char** argv)
{
    int signalCount = 1;
    float ArraryRadius = 0.1;
    float frequency = 1000.0;
    MatrixXcd signals = MatrixXcd::Random(6, 1000);
    voice_algorithm va;
    va.set_Mico_signals(signals);
    va.T_MUSIC_algorithm(signalCount,ArraryRadius,frequency);
    va.Output_angle();
}