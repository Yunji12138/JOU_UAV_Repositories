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

#define CHANNELS 8
#define SAMPLE_BYTES 2
#define FRAME_SIZE (CHANNELS * SAMPLE_BYTES)

volatile sig_atomic_t is_exit = false;

// 结束程序时候删除文件以防下次运行错误
void delete_files_on_exit() 
{
    remove(LOCAL_TMP_FILE);
    remove(LOCAL_FULL_FILE);
    ROS_INFO("Temporary files deleted.");
}

// ctrl + c退出逻辑
void Handler(int signo)
{
    //System Exit
    is_exit = true;
    delete_files_on_exit();
    printf("\r\nHandler:Program stop,Success remove file!\r\n"); 
}




// 获取本地文件大小
long get_local_file_size(const char *filename) {
    struct stat st;
    if (stat(filename, &st) != 0) return 0;
    return st.st_size;
}

// 发布PCM数据(处理增量数据并发布数组)
void process_pcm_chunk(const char *buffer, size_t len, ros::Publisher& pub) 
{
    // 处理新增的PCM数据
    size_t bytes_read = len;
    size_t total_samples = bytes_read / SAMPLE_BYTES;
    const int16_t* samples = reinterpret_cast<const int16_t*>(buffer);

    std_msgs::Int16MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "frame";
    msg.layout.dim[0].size = total_samples / CHANNELS;
    msg.layout.dim[1].label = "channel";
    msg.layout.dim[1].size = CHANNELS;

    msg.data.resize(total_samples);
    for (size_t i = 0; i < total_samples; ++i)
    {
        msg.data[i] = samples[i];
    }
    
    pub.publish(msg);
}

void sleep_ms(int ms) 
{
    usleep(ms * 1000);
}

int main(int argc, char **argv) 
{
    ros::init(argc,argv,"get_voice_file_node");
    ros::NodeHandle nh;
    ros::Publisher voice_pub = nh.advertise<std_msgs::Int16MultiArray>("origin_voice", 10); 

    signal(SIGINT, Handler);
    atexit(delete_files_on_exit); //安全退出调用删除文件程序

    long last_size = 0;
    FILE *fp_full = fopen(LOCAL_FULL_FILE, "ab+");
    if (!fp_full) {
        perror("Failed to open full pcm file");
        return 1;
    }

    ros::Rate rate(2);

    while (!is_exit && ros::ok()) 
    {
        // 拉取整个 origin.pcm 到临时文件（不覆盖已有文件）
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "adb pull %s %s > /dev/null 2>&1", REMOTE_FILE, LOCAL_TMP_FILE);

        // 处理adb pull失败
        int ret = system(cmd);
        if (ret != 0) {
            ROS_WARN("adb pull failed. Check device connection.");
            rate.sleep();
            continue;
        }

        long new_size = get_local_file_size(LOCAL_TMP_FILE);
        if (new_size < 0) {
            ROS_WARN("Failed to get temp file size.");
            rate.sleep();
            continue;
        }


        // 处理文件重置情况（远程文件被覆盖）
        if (new_size < last_size) {
            ROS_INFO("Remote file reset detected. Resetting last_size.");
            last_size = 0;
            fclose(fp_full);
            fp_full = fopen(LOCAL_FULL_FILE, "wb+"); // 清空本地完整文件
            if (!fp_full) {
                ROS_ERROR("Failed to reset full PCM file.");
                break;
            }
        }

        // 处理新增数据
        if (new_size > last_size)
        {
            FILE *fp_tmp = fopen(LOCAL_TMP_FILE, "rb");
            if (!fp_tmp) {
                perror("Failed to open temp file");
                sleep_ms(CHECK_INTERVAL_MS);
                continue;
            }

            fseek(fp_tmp, last_size, SEEK_SET);
            char buffer[4096];
            size_t read_bytes;
            while ((read_bytes = fread(buffer, 1, sizeof(buffer), fp_tmp)) > 0) 
            {
                process_pcm_chunk(buffer, read_bytes,voice_pub);
                fwrite(buffer, 1, read_bytes, fp_full);
            }
            fflush(fp_full);
            last_size = new_size;
            fclose(fp_tmp);
        }
        else 
        {
            ROS_WARN("Failed to open temp file.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    fclose(fp_full);
    return 0;
}