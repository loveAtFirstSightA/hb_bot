#ifndef _PARSER_
#define _PARSER_

// 定义数据标志位
#define DF_UNIT_IS_MM         0x0001   // 单位是毫米
#define DF_WITH_INTENSITY     0X0002   // 包含强度信息
#define DF_DESHADOWED         0x0004   // 去阴影处理
#define DF_SMOOTHED           0x0008   // 平滑处理
#define DF_FAN_90             0x0020   // 风扇90度
#define DF_WITH_RESAMPLE      0X0080   // 包含重采样
#define DF_MOTOR_REVERSE      0x0100   // 电机反向
#define DF_WITH_UUID          0X1000   // 包含UUID

#define EF_ENABLE_ALARM_MSG   0X10000  // 启用报警消息

// 定义最大风扇数量和最大数据点数量
#define MAX_FANS              120
#define MAX_POINTS            1000

// 定义任意值
//#define ANYONE 0x1234abcd
#define ANYONE -1

#include <arpa/inet.h>

// 数据点结构体
struct DataPoint {
    uint16_t idx;              // 数据点索引
    // int angle;               // 角度（注释掉）
    double degree;             // 角度（度）
    uint16_t distance;         // 距离（毫米）
    uint8_t confidence;        // 数据点置信度
};

// 原始数据结构体
struct RawData {
    unsigned short code;       // 数据码
    unsigned short N;          // 数据点数量
    unsigned short angle;      // 角度（0.1度）
    unsigned short span;       // 角度跨度（0.1度）
    unsigned short fbase;      // 基础数据
    unsigned short first;      // 第一数据点
    unsigned short last;       // 最后一数据点
    unsigned short fend;       // 结束数据点
    // short ros_angle;        // ROS角度（0.1度，注释掉）
    DataPoint points[MAX_POINTS];  // 数据点数组
    uint32_t ts[2];            // 时间戳
    uint8_t counterclockwise;  // 是否逆时针旋转
};

// 定义句柄类型
typedef void* HParser;       // 解析器句柄
typedef void* HReader;       // 读取器句柄
typedef void* HPublish;      // 发布器句柄

// 激光雷达节点结构体
struct LidarNode {
    HParser hParser;          // 解析器句柄
    HPublish hPublish;        // 发布器句柄
    char ip[30];              // IP地址
    int port;                 // 端口
    in_addr_t s_addr;         // 地址
};

// 打开解析器
HParser ParserOpen(int raw_bytes, 
                   uint32_t device_ability,
                   uint32_t flags, 
                   int init_rpm,
                   double resample_res,
                   bool with_chk, 
                   uint32_t dev_id);

// 脚本处理函数类型
typedef bool (*Script)(void*, int cmd_len, const char* cmd_str, 
                       int pattern_len, const char* pattern_str, 
                       int nfetch, char* fetch);

// 执行脚本
bool ParserScript(HParser, Script, void*);

// 关闭解析器
int ParserClose(HParser);

// 解析流数据
int ParserRunStream(HParser, int len, unsigned char* buf, RawData* fans[]);

// 解析数据
int ParserRun(LidarNode, int len, unsigned char* buf, RawData* fans[]);

// 设置时间戳
void SetTimeStamp(RawData*);

// 全局UUID
extern char g_uuid[32];

#endif
