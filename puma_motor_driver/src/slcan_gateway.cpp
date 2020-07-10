/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include <string>
#include <stdio.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

#include "puma_motor_driver/slcan_gateway.h"
using boost::asio::ip::udp;
using boost::asio::ip::address;

template<typename IntT>
static bool fromHex(const char* buffer, int8_t len, IntT* out)
{
  *out = 0;
  for (; len > 0; len--)
  {
    *out <<= 4;
    char c = *buffer;

    if (c >= '0' && c <= '9')
    {
      *out += c - '0';
    }
    else if (c >= 'A' && c <= 'F')
    {
      *out += 10 + c - 'A';
    }
    else if (c >= 'a' && c <= 'f')
    {
      *out += 10 + c - 'a';
    }
    else
    {
      // Bad character in the string to parse.
      return false;
    }

    buffer++;
  }
  return true;
}


template<typename IntT>
void toHex(IntT val, char* out)
{
  static const char* hex_set = "0123456789ABCDEF";

  out += sizeof(IntT) * 2 - 1;
  for (uint8_t i = 0; i < (sizeof(IntT) * 2); i++)
  {
    *out-- = hex_set[val & 0xf];
    val >>= 4;
  }
}

/**
 * Templated to avoid a direct header dependency on stm32f4xx_can.h, for testability.
 *
 * See: https://github.com/andysworkshop/stm32plus/blob/master/lib/fwlib/f4/stdperiph/inc/stm32f4xx_can.h#L137
 */
template<class CANMsgT>
bool decodeSLCAN(const SLCANMsg& slcan_msg, CANMsgT* can_msg_out)
{
  if (slcan_msg.type != 'R' && slcan_msg.type != 'T')
  {
    LOG_WARN("Bad SLCAN message type.", slcan_msg.type);
    return false;
  }

  // Hard code this to extended identifier; we don't support 11-bit identifiers.
  can_msg_out->IDE = 0x4;

  // Set the RTR bit according to whether the type byte was R or T.
  can_msg_out->RTR = slcan_msg.type == 'R' ? 0x2 : 0x0;

  // Read in the 29-bit ID.
  if (!fromHex(slcan_msg.id, 8, &can_msg_out->ExtId))
  {
    LOG_WARN("Bad SLCAN message id.");
    return false;
  }

  // Read in the data length and data bytes.
  if (!fromHex(slcan_msg.len, 1, &can_msg_out->DLC))
  {
    LOG_WARN("Bad SLCAN message data length.");
    return false;
  }

  if (can_msg_out->DLC > 8)
  {
    LOG_WARN("Too-high SLCAN message data length.");
    return false;
  }

  for (int i = 0; i < can_msg_out->DLC; i++)
  {
    if (!fromHex(&slcan_msg.data[i*2], 2, &can_msg_out->Data[i]))
    {
      LOG_WARN("Bad SLCAN message data.");
      return false;
    }
  }

  return true;
}


/**
 * Returns the total length of the SLCAN string produced.
 */
template<class CANMsgT>
int encodeSLCAN(const CANMsgT& can_msg, SLCANMsg* slcan_msg_out)
{
  static const char* dlc_set = "012345678";
  slcan_msg_out->len[0] = dlc_set[can_msg.DLC];
  slcan_msg_out->type = can_msg.RTR & 0x2 ? 'R' : 'T';
  toHex(can_msg.ExtId, slcan_msg_out->id);
  for (int i = 0; i < can_msg.DLC; i++)
  {
    toHex(can_msg.Data[i], &slcan_msg_out->data[i * 2]);
  }

  // Add terminating CR. This either lives in the delim field, or somewhere in the
  // data field, depending how long the data was.
  slcan_msg_out->data[can_msg.DLC * 2] = '\r';

  // 1 char header + 8 chars ID + 1 char length + 1 char delimiter = 11 chars
  int total_slcan_length = 11 + (can_msg.DLC * 2);
  return total_slcan_length;
}

namespace puma_motor_driver
{

SLCANGateway::SLCANGateway(std::string canbus_dev):
  canbus_dev_(canbus_dev),
  is_connected_(false)
{
}

bool SLCANGateway::connect()
{
    socket_ = new boost::asio::ip::udp::socket(io_service_);
    endpoint_ = udp::endpoint(address::from_string(canbus_dev_), 11412);

    socket_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}


bool SLCANGateway::isConnected()
{
  return is_connected_;
}

bool SLCANGateway::recv(Message* msg)
{
  SLCanMsg slCanMsg;
  if (boost::asio::read(socket_, boost::asio::buffer(&slCanMsg, sizeof(SLCanMsg))))
  {
    decodedMsg(*msg, slCanMsg);
  }

  return false;
}

void SLCANGateway::queue(const Message& msg)
{
    ROS_DEBUG("Queuing ID 0x%08x, data (%d)", msg.id, msg.len);
    write_frames_[write_frames_index_].id = msg.id;
    write_frames_[write_frames_index_].len = msg.len;

    memmove(write_frames_[write_frames_index_].data, msg.data, msg.len);
        
    write_frames_index_++;

    if (write_frames_index_ > sizeof(write_frames_) / sizeof(write_frames_[0]))
    {
        throw std::runtime_error("Overflow of write_frames_index_ in SLCANGateway.");
    }
}

bool SLCANGateway::sendAllQueued()
{
  for (int i = 0; i < write_frames_index_; i++)
  {
    ROS_DEBUG("Writing ID 0x%08x, data (%d)", write_frames_[i].id, write_frames_[i].len);

    SLCanMsg request = {0};

    encodedMsg(request, write_frames_[i]);

    boost::system::error_code err;
    auto sent = socket_.send_to(boost::asio::buffer(&request, sizeof(request)), endpoint_, 0, err);
    // **WARNING* 
    // Return Result Ignored on other implementations
  }
  write_frames_index_ = 0;
  return true;
}

void SLCANGateway::encodedMsg(SLCanMsg& slCanMsg, const Message& msg)
{
  
}

void SLCANGateway::decodedMsg(Message& msg, const SLCanMsg& slCanMsg)
{

}

}
