#include "ros2.h"
#include "grid_map.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
void sigint_handler(int sig) {
    // 在这里添加你的清理工作
    rclcpp::shutdown(); // 关键步骤：请求关闭节点
}
int main(int argc, char **argv) {
  // 初始化ROS2
  rclcpp::init(argc, argv);
  signal(SIGINT, sigint_handler);

  // 创建节点
  auto node = rclcpp::Node::make_shared("x_map_node");

  // 配置地图参数
  map::GridMapCofig map_config;
    std::string odom_sub_name;
  std::string cloud_sub_name;
  // 从参数服务器读取参数，如果不存在则使用默认值
  node->declare_parameter("voxScale", 0.1);
  node->declare_parameter("detection_range", 10.0);
  node->declare_parameter("global_x_lower", -50.0);
  node->declare_parameter("global_x_upper", 50.0);
  node->declare_parameter("global_y_lower", -50.0);
  node->declare_parameter("global_y_upper", 50.0);
  node->declare_parameter("p_hit", 0.7);
  node->declare_parameter("p_miss", 0.3);
  node->declare_parameter("p_min", 0.1);
  node->declare_parameter("p_max", 0.9);
  node->declare_parameter("p_occ", 0.5);
  node->declare_parameter("cloud_sub_name", "cloud_map");
  node->declare_parameter("odom_sub_name", "odom");

  // 获取参数值
  node->get_parameter("voxScale", map_config.voxScale);
  node->get_parameter("detection_range", map_config.detection_range);
  node->get_parameter("global_x_lower", map_config.global_x_lower_);
  node->get_parameter("global_x_upper", map_config.global_x_upper_);
  node->get_parameter("global_y_lower", map_config.global_y_lower_);
  node->get_parameter("global_y_upper", map_config.global_y_upper_);
  node->get_parameter("p_hit", map_config.p_hit);
  node->get_parameter("p_miss", map_config.p_miss);
  node->get_parameter("p_min", map_config.p_min);
  node->get_parameter("p_max", map_config.p_max);
  node->get_parameter("p_occ", map_config.p_occ);
  node->get_parameter("cloud_sub_name", cloud_sub_name);
  node->get_parameter("odom_sub_name", odom_sub_name);

  // 4. 创建XPlannerROS2实例
  auto planner = std::make_shared<ros2::Ros2>(node, map_config, odom_sub_name, cloud_sub_name);

  // 5. 打印启动信息
  RCLCPP_INFO(node->get_logger(), "XPlannerROS2 started!");
  RCLCPP_INFO(node->get_logger(), "Grid map interval: %.2f m", map_config.voxScale);
  RCLCPP_INFO(node->get_logger(), "Detection range: %.2f m", map_config.detection_range);

  // 6. 进入spin循环，保持节点运行
  rclcpp::spin(node);

  // 7. 关闭ROS2
  rclcpp::shutdown();
  
  return 0;
}