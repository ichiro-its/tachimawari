#include <rclcpp/rclcpp.hpp>

#include <tachimawari/motion_manager.hpp>

#include <iostream>
#include <memory>

int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto motion_manager = std::make_shared<tachimawari::MotionManager>("motion_manager");

  if (motion_manager->start()) {
    rclcpp::spin(motion_manager);
  }

  motion_manager->stop();

  rclcpp::shutdown();
}