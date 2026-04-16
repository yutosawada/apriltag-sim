#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <amr_ex_action/srv/detect_parameter.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sim_world_msgs/msg/sim_tag_array.hpp>

namespace apriltag_sim
{

struct CameraCfg
{
  std::string name;
  std::string optical_frame;
  double hfov_rad = 1.21;
  double vfov_rad = 0.75;
  double max_range_m = 3.0;
  double min_range_m = 0.2;
  tf2::Transform T_base_opt;
  bool tf_cached = false;
};

struct TagCfg
{
  std::string name;
  tf2::Transform T_map_tag;
};

class AprilTagSimNode
{
public:
  explicit AprilTagSimNode(const rclcpp::Node::SharedPtr & node);

private:
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void tick();

  void loadCameras();
  void loadTags();
  void createEnableServices();
  void onSimWorldTags(const sim_world_msgs::msg::SimTagArray::SharedPtr msg);
  bool ensureCameraTf(CameraCfg & cam);

  rclcpp::Node::SharedPtr node_;

  std::string pose_topic_;
  std::string base_frame_;

  std::vector<CameraCfg> cameras_;
  std::vector<TagCfg> tags_;

  std::mutex mtx_;
  bool has_pose_ = false;
  tf2::Transform T_map_base_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> enable_services_;
  std::vector<rclcpp::Service<amr_ex_action::srv::DetectParameter>::SharedPtr> detect_param_services_;
  rclcpp::Subscription<sim_world_msgs::msg::SimTagArray>::SharedPtr sub_sim_world_tags_;
  bool use_sim_world_{false};

  // Tag alias: set_parameter maps target_id → alias frame name (e.g. "reference_target").
  // tick() publishes the matching tag TF with this alias as child_frame_id.
  struct TagAlias {
    int target_id = -1;
    std::string frame_name;
  };
  std::mutex alias_mtx_;
  TagAlias active_alias_;
};

}  // namespace apriltag_sim
