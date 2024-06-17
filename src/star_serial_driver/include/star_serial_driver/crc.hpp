// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <cstdint>


/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
 ///计算给定消息的CRC16校验和，并与消息中的校验和进行比较，以验证消息是否完整和未被篡改
 //要验证的消息的起始位置   消息的长度
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);

/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
 //在给定的消息末尾附加一个 CRC16 校验和
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);



#endif  // RM_SERIAL_DRIVER__CRC_HPP_
