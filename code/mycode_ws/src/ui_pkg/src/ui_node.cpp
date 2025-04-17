#include "../include/ui_pkg/ui_node.hpp"

using namespace ftxui;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_ui_node");
    
    UAVMessageSub sub;
    UINode ui;
    ui.startDisplay();

    ros::spin();
    ui.stopDisplay();

    return 0;
}