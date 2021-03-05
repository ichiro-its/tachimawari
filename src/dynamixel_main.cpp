#include <string>
#include <vector>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

namespace Dynamixel
{
    class Serial 
    {
    private:
        dynamixel::PortHandler *_port_handler;
        dynamixel::PacketHandler *_packet_handler;

        std::string _port_name = "/dev/ttyUSB0";
        int _baudrate = 57600;
        bool _init_state = false;
    public:
        Serial();
        Serial(std::string port_name);
        Serial(std::string port_name, int baudrate);
        ~Serial();

        void initHandler();
        void open();
        void close();
        bool broadcastPing(std::vector<uint8_t> ids);
        bool ping(uint8_t id);

        bool _is_open = false;
    };
}

Dynamixel::Serial::Serial()
{
}

Dynamixel::Serial::Serial(std::string port_name)
: _port_name(port_name)
{
}

Dynamixel::Serial::Serial(std::string port_name, int baudrate)
: _port_name(port_name), _baudrate(baudrate)
{
}

Dynamixel::Serial::~Serial()
{
    close();
}

void Dynamixel::Serial::initHandler()
{
    std::cout << "init the port name and protocol version\n";
    _port_handler = dynamixel::PortHandler::getPortHandler(_port_name.c_str());
    _packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0F);
}

void Dynamixel::Serial::open()
{
    if (!_init_state)
        initHandler();
    
    if (_port_handler->is_using_)
        std::cout << "the port is currently using!\n";
    
    if (_port_handler->openPort()) 
    {
        std::cout << "succeeded to open the port!\n";
        if (_port_handler->setBaudRate(_baudrate))
        {
            std::cout << "succeeded to set the baudrate!\n";
            _is_open = true;
        }
        else
            std::cout << "failed to set the baudrate!\n";
    }
    else
        std::cout << "failed to open the port!\n";
}

void Dynamixel::Serial::close()
{
    std::cout << "close the port\n";
    _port_handler->closePort();
}

bool Dynamixel::Serial::broadcastPing(std::vector<uint8_t> ids)
{
    int dxl_comm_result = COMM_TX_FAIL;

    std::cout << "broadcast ping!\n";
    dxl_comm_result = _packet_handler->broadcastPing(_port_handler, ids);
    if (dxl_comm_result != COMM_SUCCESS)
        _packet_handler->getTxRxResult(dxl_comm_result);

    std::cout << "Detected Dynamixel : \n";
    for (int i = 0; i < (int)ids.size(); i++)
        std::cout << "[ID:" << ids.at(i) << "]\n";

    return true;
}

bool Dynamixel::Serial::ping(uint8_t id)
{
    int dxl_comm_result = COMM_TX_FAIL;

    uint8_t dxl_error = 0;
    uint16_t dxl_model_number;

    dxl_comm_result = _packet_handler->ping(_port_handler, id, &dxl_model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        std::cout << _packet_handler->getTxRxResult(dxl_comm_result) << "\n";
    else if (dxl_error != 0)
        std::cout << _packet_handler->getRxPacketError(dxl_error) << "\n";

    std::cout << "[ID:" << id << "] ping Succeeded. Dynamixel model number : " << dxl_model_number << "\n";

    return true;
}

std::vector<uint8_t> getIds()
{
    std::vector<uint8_t> ids;
    for (int i = 0; i < 20; i++) 
       ids.push_back((uint8_t)i);

    return ids;
}

int main(int argc, char* argv[])
{
    std::cout << "init rclcpp\n";
    rclcpp::init(argc, argv);

    std::cout << "define serial\n";
    Dynamixel::Serial *serial;

    if (argc > 1) 
    {
        std::string port = argv[1];

        if (port.find("tty") != std::string::npos)
        {
            std::cout << "set the port name from argument as " << port << "\n";
            serial = new Dynamixel::Serial("/dev/" + port);
        }
        else
        {
            std::cout << "to set the port name : ttyUSB<num>\n";
        }
    }
    else
        serial = new Dynamixel::Serial();

    std::cout << "define node\n";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ping");
    rclcpp::Rate rclcpp_rate(1000);

    std::cout << "open the port\n";
    serial->open();

    while (rclcpp::ok())
    {
        rclcpp_rate.sleep();
        rclcpp::spin_some(node);
        if (serial->_is_open)
            serial->broadcastPing(getIds());
    }

    serial->close();

    return 0;
}