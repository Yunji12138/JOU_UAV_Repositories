#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include <memory>                   
#include "ftxui/dom/node.hpp" 
#include "ftxui/screen/color.hpp"  
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace ftxui;

#define OdomTopic "/vins_fusion/fused_odom"         


struct UAVMessage
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
}; 
UAVMessage g_uav_message;
std::mutex g_message_mutex;

class UINode
{
public:
    UINode() {}

    void startDisplay()
    {
        ui_thread_ = std::thread(&UINode::displayLoop, this);
    }

    void stopDisplay()
    {
        if (ui_thread_.joinable()) {
            ui_thread_.join();
        }
    }

private:
    std::thread ui_thread_;

    void displayLoop()
    {
        auto screen = ScreenInteractive::Fullscreen();
        auto render_func = [&] {
            std::lock_guard<std::mutex> lock(g_message_mutex);

            auto format_value = [](double value) {
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(2) << value;
                return stream.str();
            };

            return vbox({
                text(" 🛸 无人机状态监控 🛸 ") | bold | center | borderDouble | color(Color::Khaki1),
                separator(),

                hbox({
                    // 位置信息（左列）
                    vbox({
                        text(" 📍 位置信息 ") | bold | color(Color::Blue) | center,
                        separator(),
                        hbox({ text("🔹 X  : ") | bold, text(format_value(g_uav_message.x)) | bold | color(Color::Cyan) }),
                        hbox({ text("🔹 Y  : ") | bold, text(format_value(g_uav_message.y)) | bold | color(Color::Cyan) }),
                        hbox({ text("🔹 Z  : ") | bold, text(format_value(g_uav_message.z)) | bold | color(Color::Cyan) }),
                    }) | borderRounded | flex,

                    separator(),

                    // 姿态信息（右列）
                    vbox({
                        text(" 🎯 姿态信息 ") | bold | color(Color::Yellow) | center,
                        separator(),
                        hbox({ text("🔄 Roll  : ") | bold, text(format_value(g_uav_message.roll)) | bold | color(Color::YellowLight) }),
                        hbox({ text("🔄 Pitch : ") | bold, text(format_value(g_uav_message.pitch)) | bold | color(Color::YellowLight) }),
                        hbox({ text("🔄 Yaw   : ") | bold, text(format_value(g_uav_message.yaw)) | bold | color(Color::YellowLight) }),
                    }) | borderRounded | flex,
                }),

                separator(),
                text(" 🚀 小飞机，启动！ 爱来自江海大 ") | dim | center | color(Color::GrayLight)
            });
        };

        auto component = Renderer(render_func);
        
        std::thread refresh_thread([&] {
            while (ros::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                screen.PostEvent(Event::Custom);
            }
        });

        screen.Loop(component);
        refresh_thread.join();
    }
};
class UAVMessageSub
{
public:
    UAVMessageSub()
    {
        odom_sub_ = nh_.subscribe(OdomTopic, 1, &UAVMessageSub::odomCallback, this);
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(g_message_mutex);
        
        g_uav_message.x = msg->pose.pose.position.x;
        g_uav_message.y = msg->pose.pose.position.y;
        g_uav_message.z = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        g_uav_message.roll = roll;
        g_uav_message.pitch = pitch;
        g_uav_message.yaw = yaw;
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
};