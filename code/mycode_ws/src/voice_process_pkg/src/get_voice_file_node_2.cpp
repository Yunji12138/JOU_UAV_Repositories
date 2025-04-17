#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fstream>
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>


#define LOCAL_TMP_FILE "/home/orangepi/mycode_ws/src/voice_process_pkg/voice_save_flie/origin_tmp.pcm"
#define LOCAL_FULL_FILE "/home/orangepi/mycode_ws/src/voice_process_pkg/voice_save_flie/origin_full.pcm"
#define REMOTE_FILE "/data/build/origin.pcm"
#define CHECK_INTERVAL_MS 500


std::atomic<bool> is_exit(false);
std::mutex data_mutex;
std::condition_variable data_cv;
bool new_data_ready = false;
std::vector<int16_t> audio_buffer;


// 获取本地文件大小
long get_local_file_size(const char *filename) {
    struct stat st;
    if (stat(filename, &st) != 0) return 0;
    return st.st_size;
}

// 异步执行adb pull的线程
void adb_pull_worker() {
    while (!is_exit) {
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "timeout 1 adb pull %s %s", REMOTE_FILE, LOCAL_TMP_FILE);
        
        int ret = system(cmd);
        if (ret != 0) {
            ROS_WARN("adb pull failed (code:%d)", ret);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_INTERVAL_MS));
    }
}

// 非阻塞文件读取线程
void file_process_worker(ros::Publisher& pub) 
{
    long last_size = 0;
    FILE* fp_full = fopen(LOCAL_FULL_FILE, "ab+");
    
    while (!is_exit) {
        // 获取文件大小
        long current_size = get_local_file_size(LOCAL_TMP_FILE);
        if (current_size <= last_size) continue;

        // 非阻塞读取增量数据
        FILE* fp_tmp = fopen(LOCAL_TMP_FILE, "rb");
        if (fp_tmp) {
            fseek(fp_tmp, last_size, SEEK_SET);
            char chunk[1024];
            size_t bytes_read = fread(chunk, 1, sizeof(chunk), fp_tmp);
            
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                audio_buffer.insert(audio_buffer.end(), 
                    reinterpret_cast<int16_t*>(chunk),
                    reinterpret_cast<int16_t*>(chunk + bytes_read)
                );
                new_data_ready = true;
            }
            data_cv.notify_one();
            
            fwrite(chunk, 1, bytes_read, fp_full);
            last_size = current_size;
            fclose(fp_tmp);
        }
    }
    fclose(fp_full);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_voice_file_node");
    ros::NodeHandle nh;
    ros::Publisher voice_pub = nh.advertise<std_msgs::Int16MultiArray>("origin_voice", 10);

    // 启动工作线程
    std::thread adb_thread(adb_pull_worker);
    std::thread file_thread(file_process_worker, std::ref(voice_pub));

    // 主线程负责ROS消息发布
    while (ros::ok() && !is_exit) {
        std::unique_lock<std::mutex> lock(data_mutex);
        data_cv.wait(lock, []{ return new_data_ready; });
        
        // 构造并发布消息
        std_msgs::Int16MultiArray msg;
        msg.data.swap(audio_buffer); // 零拷贝交换数据
        voice_pub.publish(msg);
        
        new_data_ready = false;
        lock.unlock();
        
        ros::spinOnce();
    }

    is_exit = true;
    adb_thread.join();
    file_thread.join();
    return 0;
}