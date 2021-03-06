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

#include <rclcpp/rclcpp.hpp>
#include <motion/serial.hpp>

#include <memory>
#include <string>
#include <iostream>

int main(int argc, char * argv[])
{
    std::cout << "init rclcpp\n";
    rclcpp::init(argc, argv);

    std::cout << "define node\n";
    auto serial = std::make_shared<motion::Serial>("serial_ping");

    if (argc > 1) {
        std::string port = argv[1];

		std::cout << "set the port name from argument as " << port << "\n";
		serial = std::make_shared<motion::Serial>("serial_ping", port);
    }

    std::cout << "open the port\n";
    if (serial->open()){
		rclcpp::spin(serial);
		serial->broadcastPing();
	}

    rclcpp::shutdown();

    return 0;
}
