#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>

#include <thread>
#include <atomic>
#include <algorithm> 
#include <queue>
#include <mutex>



#define MAX(a,b) a > b ? a : b
#define MIN(a,b) a < b ? a : b

#define OdomTopic "/vins_fusion/imu_propagate"              //位姿话题
#define GoalTopic "/move_base_simple/goal"  //egoplanner接受目标点话题
#define TakeoffTopic "/px4ctrl/takeoff_land"

class DroneFSM;

class KeyBoradCtrl
{
public:
    KeyBoradCtrl() : step_(1.0) ,is_running_(true), key_char_('\0')
    {
        // 启动键盘监听线程
        key_thread_ = std::thread(&KeyBoradCtrl::keyboardListener, this);
    }

    ~KeyBoradCtrl()
    {
        // 停止键盘监听
        is_running_ = false;
        if (key_thread_.joinable())
        {
            key_thread_.join();
        }
    }

    geometry_msgs::PoseStamped run_t(geometry_msgs::PoseStamped current_pose_msg)
    {
        if(current_pose_msg.header.frame_id.empty())
        {
            return current_pose_msg;
        }
        geometry_msgs::PoseStamped goal = current_pose_msg;
        char key = getNextKey();
        switch (key) 
        {
            case 'w': goal.pose.position.x += step_; std::cout << "Press W" << std::endl ; break;  // X+方向
            case 's': goal.pose.position.x -= step_; break;  // X-方向
            case 'a': goal.pose.position.y += step_; break;  // Y+方向（左移）
            case 'd': goal.pose.position.y -= step_; break;  // Y-方向（右移）
            case 'q': goal.pose.position.z += step_; break;  // Z+方向（上升）
            case 'e': goal.pose.position.z -= step_; break;  // Z-方向（下降）
            case 'x': goal.pose = current_pose_msg.pose; break; // x 急停
            case '+': step_ = std::max(step_ + 0.01, 5.0);break; // + 步长增加0.01
            case '-': step_ = std::min(step_ + 0.01, 5.0);break; // + 步长减少0.01
            case ' ': goal = current_pose_msg; break;  // 重置为当前位姿
            case 27: 
                ros::shutdown(); 
                break; // 退出程序    
            default: break; 
        }

        key_char_.store('\0');
        return goal;
    }

    geometry_msgs::PoseStamped test_one_meter_x(geometry_msgs::PoseStamped current_pose_msg)
    {
        geometry_msgs::PoseStamped goal = current_pose_msg;
        goal.pose.position.y += 1.0;
        return goal;
    }

    char getKey()
    {
        return key_char_.load();
    }

    char getNextKey()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!key_queue_.empty()) 
        {
            char key = key_queue_.front();
            key_queue_.pop();
            return key;
        }
        return '\0';
    }

    // 键盘监听线程
    void keyboardListener()
    {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
        int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // 设置非阻塞输入
    
        while (is_running_)
        {
            char ch;
            ssize_t bytesRead = read(STDIN_FILENO, &ch, 1); // 读取 1 个字节
            if (bytesRead > 0) 
            {
                // key_char_.store(ch);
                std::lock_guard<std::mutex> lock(queue_mutex_);
                key_queue_.push(ch);
                // test
                std::cout << "key: " << key_queue_.front() << std::endl;
            }
            usleep(10000);  // 10ms 延迟，降低CPU占用
        }
    
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf); // 还原文件状态
    }
    

private:
    double step_;

    std::atomic<bool> is_running_;    // 线程运行标志
    std::atomic<char> key_char_;
    std::thread key_thread_;

    std::queue<char> key_queue_;  
    std::mutex queue_mutex_;  

};

class DroneFSM
{
public:
    DroneFSM()
    {
        current_state_ = CURRENT_FLY_STATE::LAND;
        // 订阅当前位姿
        odom_sub_ = nh_.subscribe(OdomTopic, 1, &DroneFSM::odomCallback, this);
        // 发布目标点
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(GoalTopic, 10);
        // 发布起飞/降落命令
        flycmd_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>(TakeoffTopic, 10);
        // 订阅遥控器指令
        rc_sub_ = nh_.subscribe("/mavros/rc/in", 1, &DroneFSM::rcCallback, this);
    }

    geometry_msgs::PoseStamped getCurrentPose()
    {
        return current_pose_msg_;
    }

    void process()
    {
        switch (current_state_)
        {
        case CURRENT_FLY_STATE::LAND:
        {
            if(LAND_FLAG == 0)
            {
                std::cout << "按下 <-- (遥控器左侧按键)起飞,请注意周围环境安全...(L进入调试模式)" << std::endl;
                LAND_FLAG = 1;
            }

            if(kbc_.getNextKey() == 'L' || kbc_.getNextKey() == 'l' )
            {
                current_state_ = CURRENT_FLY_STATE::FLY;
            }
            // char key = kbc_.getKey();
            if(rc_channel_9_ > 1800)
            {                
                Takeoff_cmd_msg_.takeoff_land_cmd =  quadrotor_msgs::TakeoffLand::TAKEOFF; 
                flycmd_pub_.publish(Takeoff_cmd_msg_);
                ROS_WARN("Takeoff,Please be careful!");
                current_state_ = CURRENT_FLY_STATE::FLY;
            }
            FLY_FLAG = 0;
            break;
        }

        case CURRENT_FLY_STATE::FLY:
        {
            if(FLY_FLAG == 0)
            {
                std::cout << "WASD 控制方向, QE 升降, 空格重置目标:" << std::endl;
                FLY_FLAG = 1;
            }
            if(rc_channel_10_ > 1800)
            {
                Takeoff_cmd_msg_.takeoff_land_cmd =  quadrotor_msgs::TakeoffLand::LAND;
                flycmd_pub_.publish(Takeoff_cmd_msg_);
                current_state_ = CURRENT_FLY_STATE::LAND;
            }

            if(rc_channel_9_ > 1800)
            {
                publish_pose_ = kbc_.test_one_meter_x(current_pose_msg_);
                goal_pub_.publish(publish_pose_);
            }

            // publish_pose_ = kbc_.test_one_meter_x(current_pose_msg_);
            // publish_pose_ = kbc_.run_t(current_pose_msg_);
            // goal_pub_.publish(publish_pose_);
            
            break;
        }

        case CURRENT_FLY_STATE::ERROR:
        {
            std::cout << "错误！尝试进行迫降！请尽快切换手动模式！" << std::endl;
            Takeoff_cmd_msg_.takeoff_land_cmd =  quadrotor_msgs::TakeoffLand::LAND; 
            flycmd_pub_.publish(Takeoff_cmd_msg_);
            break;
        }

        default:
            break;
        }
    }
    enum CURRENT_FLY_STATE
    {
        LAND = 1,
        FLY = 2,
        ERROR = -1,
    };
    
    size_t getCurrentState()
    {
        return current_state_;
    }

    void publishTakeoff()
    {
        Takeoff_cmd_msg_.takeoff_land_cmd =  quadrotor_msgs::TakeoffLand::TAKEOFF;
        flycmd_pub_.publish(Takeoff_cmd_msg_);
    }
private:
    ros::NodeHandle nh_;
    size_t current_state_;
    ros::Publisher goal_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber rc_sub_;
    ros::Publisher flycmd_pub_;
    KeyBoradCtrl kbc_;

    nav_msgs::Odometry current_odom_;
    geometry_msgs::PoseStamped current_pose_msg_;
    quadrotor_msgs::TakeoffLand Takeoff_cmd_msg_;
    geometry_msgs::PoseStamped publish_pose_;

    size_t rc_channel_9_ = 0;
    size_t rc_channel_10_ = 0;

    size_t FLY_FLAG = 0;
    size_t LAND_FLAG = 0;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_pose_msg_.header = msg -> header;
        current_pose_msg_.pose = msg -> pose.pose;
        current_odom_.twist.twist = msg -> twist.twist;
        if
        (std::fabs(current_odom_.pose.pose.position.x) > 20.0 || 
         std::fabs(current_odom_.pose.pose.position.y) > 20.0 ||
         std::fabs(current_odom_.pose.pose.position.z) > 10.0
        )
        {
            current_state_ = CURRENT_FLY_STATE::ERROR;
        }
    }
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& rc_msg)
    {
        rc_channel_9_ = rc_msg -> channels[8];
        rc_channel_10_ = rc_msg -> channels[9];
    }
};