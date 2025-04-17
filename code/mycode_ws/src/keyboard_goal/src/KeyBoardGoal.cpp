#include "../include/keyboard_goal/KeyBoardGoal.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboardctrl");
    // KeyBoradCtrl kbc;
    // kbc.run();
    
    DroneFSM fsm;
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        fsm.process();
        ros::spinOnce();
        rate.sleep();
    }
    
   
    return 0;
}
