// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

struct SendPacket
{
  uint8_t header = 0xA5;
  //uint8_t star_number;
  float red_point_x;
  float red_point_y;

  //bool star_position;
  //uint8_t reserved : 1;
  //uint16_t checksum = 0;     //表示校验和
  uint8_t rear = 0xFE;
} __attribute__((packed));

//将SendPacket对象转换为一个存储其二进制表示的std::vector<uint8_t>对象，并返回该向量
inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  //从源位置复制数据，直到结束位置，并将其存储到目标位置开始的位置
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}


#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
