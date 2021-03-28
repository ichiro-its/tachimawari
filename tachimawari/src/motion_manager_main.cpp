#include <rclcpp/rclcpp.hpp>

#include <motion_opencr/motion_manager.hpp>

#include <iostream>
#include <memory>

int main (int argc, char[] * argv)
{
  rclcpp::init(argc, argv);

  auto motion_manager = std::make_shared<tachimawari::MotionManager>("motion_manager");
  
  motion_manager.start();

  rclcpp::rate rcl_rate(8ms);

  while (rclcpp::ok) {
    rcl_rate.sleep();
    rclcpp::spin(motion_manager);
  }

  motion_manager.stop();
}