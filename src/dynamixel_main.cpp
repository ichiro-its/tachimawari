#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PROTOCOL_VERSION                2.0

#define DXL_ID                          1
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"

#define NODE_NAME                       "dynamixel_ping"
#define TOPIC_NAME                      "dynamixel_topic"

using namespace std;

int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int ping() 
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    int dxl_comm_result = COMM_TX_FAIL;

    uint8_t dxl_error = 0;
    uint16_t dxl_model_number;

    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    dxl_comm_result = packetHandler->ping(portHandler, DXL_ID, &dxl_model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", DXL_ID, dxl_model_number);
    }

    portHandler->closePort();
    return 0;
}

void getMessage(const std_msgs::msg::String::SharedPtr message)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get message: %s", message->data.c_str());
    ping();
}

void publishMessage(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
    auto message = std_msgs::msg::String();
    message.data = "Ping Instruction";
    publisher->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("rcl_cpp"), "Publish a message: %s", message.data.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(NODE_NAME);
    rclcpp::Rate rclcpp_rate(1000ms);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = node->create_publisher<std_msgs::msg::String>(TOPIC_NAME, 10);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber = node->create_subscription<std_msgs::msg::String>(TOPIC_NAME, 10, bind(&getMessage, placeholders::_1));

    while (rclcpp::ok())
    {
        rclcpp_rate.sleep();
        rclcpp::spin_some(node);
        publishMessage(publisher);
    }

    return 0;
}