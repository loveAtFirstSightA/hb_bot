#ifndef _READER_
#define _READER_

#include "parser.h"

// 定义最大激光雷达数量
#define MAX_LIDARS 8

// 激光雷达信息结构体
struct LidarInfo {
    HParser parser;         // 解析器句柄
    HPublish pub;           // 发布器句柄
    char lidar_ip[32];      // 激光雷达IP地址
    int lidar_port;         // 激光雷达端口
};

// 发布数据函数
void PublishData(HPublish, int, RawData**);

// 启动串口读取器
HReader StartUartReader(const char* port, int baudrate, int* rate_list, HParser, HPublish);

// 发送串口命令
bool SendUartCmd(HReader, int len, char*);

// 启动UDP读取器（通过激光雷达IP和端口）
HReader StartUDPReader(const char* lidar_ip, unsigned short lidar_port, unsigned short listen_port, 
                        bool is_group_listener, const char* group_ip, 
                        HParser hParser, HPublish hPub);

// 启动UDP读取器（通过多个激光雷达信息）
HReader StartUDPReader(unsigned short listen_port, bool is_group_listener, const char* group_ip,
                       int lidar_count, const LidarInfo* lidars);

// 发送UDP命令
bool SendUdpCmd(HReader hr, int id, int len, char* cmd);

// 启动TCP读取器
HReader StartTCPReader(const char* lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub);

// 发送TCP命令
bool SendTcpCmd(HReader hr, int len, char* cmd);

// 停止串口读取器
void StopUartReader(HReader hr);

// 停止UDP读取器
void StopUDPReader(HReader hr);

// 停止TCP读取器
void StopTCPReader(HReader hr);

#endif
