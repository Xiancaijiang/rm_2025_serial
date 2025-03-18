// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacketVision
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));

//比赛状态结构体
struct GameStatusPacket
{
  uint8_t header = 0xA8;
  uint8_t game_type : 4;          // 比赛类型
  uint8_t game_progress : 4;      // 比赛阶段
  uint16_t stage_remain_time;     // 当前阶段剩余时间，单位s
  uint64_t sync_time_stamp;       // 机器人接收到该指令的精确 Unix 时间
} __attribute__((packed)); 

// 比赛机器人血量数据结构体
struct RobotStatusPacket
{ 
uint8_t header = 0xA7;
 uint8_t robot_id; 
 uint8_t robot_level; 
 uint16_t current_hp; 
 uint16_t maximum_hp; 
 uint16_t shooter_barrel_cooling_value; 
 uint16_t shooter_barrel_heat_limit; 
 uint16_t chassis_power_limit; 
 uint8_t power_management_gimbal_output : 1; 
 uint8_t power_management_chassis_output : 1; 
 uint8_t power_management_shooter_output : 1; 
} __attribute__((packed));  

// 定义射击数据包结构体  
struct ShootDataPacket  
{  
  uint8_t header = 0xA6; // 你可以根据需要设置一个合适的头部  
  uint8_t bullet_type;    // 子弹类型: 1(17mm弹丸), 2(42mm弹丸)  
  uint8_t shooter_id;     // 发射机构ID: 1,2为17mm, 3为42mm  
  uint8_t bullet_freq;    // 射频(Hz)  
  float bullet_speed;      // 射速(m/s)  
  uint16_t checksum = 0;  // 校验和  
} __attribute__((packed));  


struct SendPacketVision
{
  uint8_t header = 0xA5;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacketTwist
{
  uint8_t header = 0xA4;
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
  uint16_t checksum = 0;
} __attribute__((packed));

// inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
// {
//   ReceivePacket packet;
//   std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
//   return packet;
// }

// inline std::vector<uint8_t> toVector(const SendPacket & data)
// {
//   std::vector<uint8_t> packet(sizeof(SendPacket));
//   std::copy(
//     reinterpret_cast<const uint8_t *>(&data),
//     reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
//   return packet;
// }

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
