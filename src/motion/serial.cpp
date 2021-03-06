// Copyright 2020-2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "motion/serial.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <string>
#include <vector>

namespace motion
{

Serial::Serial(std::string node_name, std::string port_name)
: rclcpp::Node(node_name)
{
    port_handler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0F);
}

Serial::~Serial()
{
    close();
}

bool Serial::open()
{
    if (port_handler->is_using_) {
        RCLCPP_WARN(get_logger(), "the port is currently using!");
        return true;
    }
    if (port_handler->openPort()) {
        RCLCPP_INFO(get_logger(), "succeeded to open the port!");
        if (port_handler->setBaudRate(baudrate)) {
            RCLCPP_INFO(get_logger(), "succeeded to set the baudrate!");
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "failed to set the baudrate!");
        }
    } else {
        RCLCPP_ERROR(get_logger(), "failed to open the port!");
    }

    return false;
}

void Serial::close()
{
    RCLCPP_INFO(get_logger(), "close the port");
    port_handler->closePort();
}

bool Serial::broadcastPing()
{
    std::vector<uint8_t> ids(20);
    std::iota(ids.begin(), ids.end(), 0);
    return broadcastPing(ids);
}

bool Serial::broadcastPing(std::vector<uint8_t> ids)
{
    int dxl_comm_result = COMM_TX_FAIL;

    RCLCPP_DEBUG(get_logger(), "broadcast ping!");
    dxl_comm_result = packet_handler->broadcastPing(port_handler, ids);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_DEBUG(get_logger(), packet_handler->getTxRxResult(dxl_comm_result));

        RCLCPP_DEBUG(get_logger(), "Detected Dynamixel : ");
        for (int i = 0; i < static_cast<int>(ids.size()); i++) {
            RCLCPP_DEBUG(get_logger(), "[ID:%03d]", ids.at(i));
        }

        return true;
    }

    return false;
}

bool Serial::ping(uint8_t id)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    uint16_t dxl_model_number;

    RCLCPP_DEBUG(get_logger(), "ping!");
    dxl_comm_result = packet_handler->ping(port_handler, id, &dxl_model_number, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_DEBUG(get_logger(), packet_handler->getTxRxResult(dxl_comm_result));

        RCLCPP_DEBUG(get_logger(), "[ID:%03d] ping Succeeded. Dynamixel model number : %d",
            id, dxl_model_number);

        return true;
    } else if (dxl_error != 0) {
        RCLCPP_DEBUG(get_logger(), packet_handler->getRxPacketError(dxl_error));
    }

    return false;
}

}   // namespace motion
