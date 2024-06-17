// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

struct ReceivePacket        //接收到的数据包
{
  uint8_t header = 0x5A;     //头部标识
  //bool star_inside;
  uint8_t star_index;
  //uint16_t checksum = 0;      //检查完整性
  
} __attribute__((packed));    //告诉编译器不要进行字节对齐

struct SendPacket
{
  uint8_t header = 0xA5;
  //uint8_t star_number;
  float star_x;
  float star_y;
  float star_z;
  //bool star_position;
  //uint8_t reserved : 1;
  //uint16_t checksum = 0;     //表示校验和
  uint8_t rear = 0xFE;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  //使用reinterpret_cast<uint8_t *>(&packet)将packet对象的地址转换为uint8_t类型的指针，
  //从而可以直接将data中的字节按顺序复制到packet对象的内存中
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

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
