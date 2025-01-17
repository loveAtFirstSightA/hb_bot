#ifndef _ALARM_
#define _ALARM_

#include <cstdint>

// 定义日志等级
#define LMSG_ERROR      1
#define LMSG_INFO       2
#define LMSG_DEBUG      4
#define LMSG_ALARM      0x100

// 定义错误码
#define LERR_LOW_POWER      1
#define LERR_MOTOR          2
#define LERR_RANGER_HI      4
#define LERR_NETWORK        8
#define LERR_RANGER_IDLE    0x10
#define LERR_ZERO_MISS      0x20

// 激光雷达消息头结构体
struct LidarMsgHdr {
    char sign[4];             // 必须是 "LMSG"
    uint32_t proto_version;   // 协议版本，0x101
    char dev_sn[20];          // 设备序列号
    uint32_t dev_id;          // 设备ID
    uint32_t timestamp;       // 时间戳
    uint32_t type;            // 消息类型
    uint32_t data;            // 消息数据
    uint16_t id;              // 消息ID
    uint16_t extra;           // 扩展字段
};

// 激光雷达报警结构体
struct LidarAlarm {
    LidarMsgHdr hdr;          // 消息头
    uint32_t zone_actived;    // 区域激活标志
    uint8_t all_states[32];   // 所有状态
    uint32_t reserved[11];    // 保留字段
};

// 获取指定位置的比特位
#define getbit(x, y)     ((x) >> (y) & 1)

// 设置指定位置的比特位为1
#define setbit(x, y)     (x |= (1 << y))

#endif
