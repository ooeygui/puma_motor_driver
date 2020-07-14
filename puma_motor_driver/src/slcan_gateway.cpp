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

#define CAN_PORT    11412

namespace puma_motor_driver
{

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

template<class SLCANMsg>
bool decodeSLCAN(const SLCANMsg& slcan_msg, Message* can_msg_out)
{
  if (slcan_msg.type != 'R' && slcan_msg.type != 'T')
  {
    ROS_WARN("Bad SLCAN message type.", slcan_msg.type);
    return false;
  }

  // Read in the 29-bit ID.
  if (!fromHex(slcan_msg.id, 8, &can_msg_out->id))
  {
    ROS_WARN("Bad SLCAN message id.");
    return false;
  }

  // Read in the data length and data bytes.
  if (!fromHex(slcan_msg.len, 1, &can_msg_out->len))
  {
    ROS_WARN("Bad SLCAN message data length.");
    return false;
  }

  if (can_msg_out->len > 8)
  {
    ROS_WARN("Too-high SLCAN message data length.");
    return false;
  }

  for (int i = 0; i < can_msg_out->len; i++)
  {
    if (!fromHex(&slcan_msg.data[i*2], 2, &can_msg_out->data[i]))
    {
      ROS_WARN("Bad SLCAN message data.");
      return false;
    }
  }

  return true;
}


/**
 * Returns the total length of the SLCAN string produced.
 */
template<class Message>
int encodeSLCAN(const Message& can_msg, SLCanMsg* slcan_msg_out)
{
  static const char* dlc_set = "012345678";
  slcan_msg_out->len[0] = dlc_set[can_msg.len];
  slcan_msg_out->type = 'T';
  toHex(can_msg.id, slcan_msg_out->id);
  for (int i = 0; i < can_msg.len; i++)
  {
    toHex(can_msg.data[i], &slcan_msg_out->data[i * 2]);
  }

  // Add terminating CR. This either lives in the delim field, or somewhere in the
  // data field, depending how long the data was.
  slcan_msg_out->data[can_msg.len * 2] = '\r';

  // 1 char header + 8 chars ID + 1 char length + 1 char delimiter = 11 chars
  int total_slcan_length = 11 + (can_msg.len * 2);
  return total_slcan_length;
}


SLCANGateway::SLCANGateway(std::string canbus_dev):
  canbus_dev_(canbus_dev),
  is_connected_(false),
  io_service_(),
  mcu_endpoint_(address::from_string("192.168.131.2"), CAN_PORT),
  local_endpoint_(address::from_string("192.168.131.1"), CAN_PORT)
{
  socket_ = NULL;
  write_frames_index_ = 0;
}

bool SLCANGateway::connect()
{
  if(socket_ != NULL)
  {
    socket_->close();
    delete socket_;
  }

    boost::system::error_code local_err, remote_err;
    socket_ = new boost::asio::ip::udp::socket(io_service_);

    // connect to the remote endpoint
    socket_->open(udp::v4());
    socket_->bind(local_endpoint_, local_err);

    is_connected_ = local_err == boost::system::errc::success;// &&
                    //remote_err == boost::system::errc::success;

    socket_thread_ = boost::thread(boost::bind(&boost::asio::io_context::run, &io_service_));
    socket_->async_receive_from(boost::asio::buffer(&msg_, sizeof(Message)), mcu_endpoint_, 
      std::bind(&SLCANGateway::recv_handler, this, std::placeholders::_1, std::placeholders::_2));

    return is_connected_;
}

bool SLCANGateway::isConnected()
{
  return is_connected_;
}

void SLCANGateway::recv_handler(const boost::system::error_code &err, std::size_t bytes_transferred)
{
  if(bytes_transferred >= sizeof(Message))
  {
   decodeSLCAN(slCanMsg_, &msg_);
  }
  else
  {
    ROS_ERROR("Not enough data read! %d", bytes_transferred);
  }
}

bool SLCANGateway::recv(Message* msg)
{
  return true;
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

    encodeSLCAN(write_frames_[i], &request);

    boost::system::error_code err;
    auto sent = socket_->send_to(boost::asio::buffer(&request, sizeof(request)), mcu_endpoint_, 0, err);
    // **WARNING* 
    // Return Result Ignored on other implementations
  }
  write_frames_index_ = 0;
  return true;
}
}
