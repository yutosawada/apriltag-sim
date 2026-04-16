#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "apriltag_sim/apriltag_sim_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("apriltag_sim");
  apriltag_sim::AprilTagSimNode sim(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
