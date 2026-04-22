#pragma once
#include "grid_map.hpp"
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ros2 {
class Ros2 {
public:
  Ros2(const rclcpp::Node::SharedPtr &node, const map::GridMapCofig &config,
       std::string odom_sub_name, std::string cloud_sub_name)
      : node_(node), map_config_(config), odom_sub_name_(odom_sub_name),
        cloud_sub_name_(cloud_sub_name) {
    grid_map_ = std::make_shared<map::GridMap>(config);
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_sub_name_, 10,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          cloud_callback(msg);
        });
    pub_gridmap = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "Xmap/gridmap", 10);
    pub_occgridmap = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "Xmap/occgridmap", 10);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_sub_name_, 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });
    occ_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                          [this]() { grid_map_->update(); });
    vis_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500),
                                          [this]() { visCallback(); });
  };

private:
  std::string odom_sub_name_;
  std::string cloud_sub_name_;
  std::shared_ptr<map::GridMap> grid_map_;

  map::GridMapCofig map_config_;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr occ_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gridmap;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occgridmap;
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, grid_map_->get_cloud());
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto yaw = tf2::getYaw(msg->pose.pose.orientation);
    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y, yaw);
    grid_map_->update_odom(position);
  }

  void publish_gridmap() {
    pcl::PointCloud<pcl::PointXYZI> cloud_vis;
    sensor_msgs::msg::PointCloud2 map_vis;
    for (int idx = 1; idx < grid_map_->get_global_size(); idx++) {
      // if(gridmap_[idx]==Unoccupied){
      //   Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
      //   pcl::PointXYZI pt;
      //   pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
      //   pt.intensity = 8.0;
      //   cloud_vis.points.push_back(pt);
      // }
      if (grid_map_->gridmap_[idx] == map::Occupied) {
        Eigen::Vector2d corrd =
            grid_map_->gridIndex2coordd(grid_map_->vectornum2gridIndex(idx));
        pcl::PointXYZI pt;
        pt.x = corrd.x();
        pt.y = corrd.y();
        pt.z = 0.1;
        pt.intensity = 0.0;
        cloud_vis.points.push_back(pt);
      }
      if (grid_map_->gridmap_[idx] == map::Unknown) {
        Eigen::Vector2d corrd =
            grid_map_->gridIndex2coordd(grid_map_->vectornum2gridIndex(idx));
        pcl::PointXYZI pt;
        pt.x = corrd.x();
        pt.y = corrd.y();
        pt.z = 0.1;
        pt.intensity = 8.0;
        cloud_vis.points.push_back(pt);
      }
    }

    pcl::PointXYZI pt;
    pt.x = 100.0;
    pt.y = 100.0;
    pt.z = 0.1;
    pt.intensity = 10.0;
    cloud_vis.points.push_back(pt);

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;
    pcl::toROSMsg(cloud_vis, map_vis);
    map_vis.header.frame_id = "world";
    this->pub_gridmap->publish(map_vis);
  }
void visCallback() {
    // 添加调试信息
    static int call_count = 0;
    call_count++;
    
    if (call_count % 10 == 0) {  // 每5秒打印一次
        int occupied = 0, unknown = 0, unoccupied = 0;
        for (int idx = 0; idx < grid_map_->get_global_size(); idx++) {
            if (grid_map_->gridmap_[idx] == map::Occupied) occupied++;
            else if (grid_map_->gridmap_[idx] == map::Unknown) unknown++;
            else if (grid_map_->gridmap_[idx] == map::Unoccupied) unoccupied++;
        }
        RCLCPP_INFO(node_->get_logger(), 
                    "Map stats - Occ:%d, Unocc:%d, Unknown:%d, Total:%d",
                    occupied, unoccupied, unknown, 
                    grid_map_->get_global_size());
    }
    
    publish_gridmap();
    // publish_occgridmap();
}
};
} // namespace ros2