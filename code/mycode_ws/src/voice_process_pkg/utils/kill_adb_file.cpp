#include "kill_adb_file.h"

//process_name为进程名，不是路径
bool kill_adb_process(const std::string &process_name)
{
    char pid_cmd[128];
    snprintf(pid_cmd, sizeof(pid_cmd), "adb shell pidof %s", process_name.c_str());
    FILE *fp = popen(pid_cmd, "r"); //这里通过打开文件来捕获标准输出（程序pid）

    if(!fp)
    {
        std::cerr << "无法执行 adb shell pidof" << std::endl;
        perror("popen");
        return false;
    }

    char pid_buffer[64];
    if(!fgets(pid_buffer,sizeof(pid_buffer),fp))
    {
        std::cerr << "未能获取到 PID." << std::endl;
        pclose(fp);
        return false;
    }


    //去除/n构造命令
    pid_buffer[strcspn(pid_buffer, "\n")] = 0;

    char pid_buf_2[4];
    for(int i = 0; i < 4; i++)
    {
        pid_buf_2[i] = pid_buffer[i];
    }

    // debug
    // for(int i = 0; i < sizeof(pid_buf_2);i++)
    // {
    //     std::cerr << pid_buf_2[i] << std::endl;
    // }

    char kill_cmd[128];
    snprintf(kill_cmd, sizeof(kill_cmd), "adb shell kill -9 %s", pid_buf_2);
    int ret = system(kill_cmd);

    if (ret == 0) 
    {
        std::cout << "成功杀死进程: " << process_name << " (PID: " << pid_buffer << ")" << std::endl;
        return true;
    } 
    else 
    {
        std::cerr << "kill 失败" << std::endl;
        return false;
    }
}