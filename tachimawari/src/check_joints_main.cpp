// Copyright (c) 2021 Ichiro ITS
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

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <string>

int main(int argc, char * argv[])
{
  std::string port_name = "/dev/ttyACM0";
  int baudrate = 57600;

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  std::vector<uint8_t> ids(20);
  std::iota(ids.begin(), ids.end(), 1);

  if (argc > 1) {
    port_name = argv[1];
  }

  std::cout << "set the port name as " << port_name << "\n";
  dynamixel::PortHandler * port_handler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
  dynamixel::PacketHandler * packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0F);

  std::cout << "open the port\n";
  if (port_handler->openPort()) {
    std::cout << "succeeded to open the port!\n";
  } else {
    std::cout << "failed to open the port!\n" <<
      "try again!\n";
    return 0;
  }

  if (port_handler->setBaudRate(baudrate)) {
    std::cout << "succeeded to set the baudrate!\n";
  } else {
    std::cout << "failed to set the baudrate!\n" <<
      "try again!\n";
  }

  std::cout << "\033c";

  std::cout << "ping the joints\n\n";
  for (auto id : ids) {
    dxl_comm_result = packet_handler->ping(port_handler, id, NULL, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      std::cout << "[ID: " << std::setfill('0') << std::setw(2) << int(id) << "] ping failed. " <<
        packet_handler->getTxRxResult(dxl_comm_result) << "\n";
    } else if (dxl_error != 0) {
      std::cout << "[ID: " << std::setfill('0') << std::setw(2) << int(id) << "] ping failed. " <<
        packet_handler->getRxPacketError(dxl_comm_result) << "\n";
    } else {
      std::cout << "[ID: " << std::setfill('0') << std::setw(2) << int(id) << "] ping succeeded.\n";
    }
  }

  std::cout << "\nping done\n" <<
    "close the port\n";
  port_handler->closePort();

  return 0;
}
