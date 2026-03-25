#pragma once

#include "protocal.h"
#include "socket_server.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <thread>
#include <unistd.h>
#include <cmath>

using namespace std::chrono_literals;

class CmdVelSubscriber : public rclcpp::Node {
public:

  static void handle_sigint(int signum);
  static void send_control(double x, double y);
  static std::shared_ptr<CmdVelSubscriber> get_instance();

  void init_finished();

  CmdVelSubscriber();

  CmdVelSubscriber(const CmdVelSubscriber&) = delete;
  CmdVelSubscriber(CmdVelSubscriber&&) = delete;
  CmdVelSubscriber& operator=(const CmdVelSubscriber&) = delete;
  CmdVelSubscriber& operator=(CmdVelSubscriber&&) = delete;
  ~CmdVelSubscriber();


private:

  static std::atomic<bool> stop_requested_;
  static std::shared_ptr<CmdVelSubscriber> cmd_vel_subscriber_instance_;

  std::shared_ptr<SocketServer<ReceiveNavigationInfo>> ser_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      subscription_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr check_zero_timer_;
  std::thread send_control_thread_;
  std::mutex mutex_;

  std::shared_ptr<geometry_msgs::msg::Twist> latest_msg_;
  rclcpp::Time latest_time_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hp_publisher_;
  std::string cmd_vel_topic_name_;
  std::atomic<bool> initialised_ = false;
  std::atomic<bool> respawn_init_ = false;
  rclcpp::TimerBase::SharedPtr respawn_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr flag_timer_; // 临时测试用
  rclcpp::TimerBase::SharedPtr game_timer_;
  static std::atomic<bool> auto_aim_captured_;
  static std::atomic<int> game_progress_;
  static std::atomic<bool> startup_finished_;
  static std::atomic<bool> startup_timer_created_;
  static std::atomic<bool> hit_to_start_;

  // 用于存储 nav2 odometry 的 yaw（来自MID360 IMU，比较稳定）
  static std::atomic<double> nav2_yaw_;
  double initial_yaw_ = 0.0;       // 初始化时记录的 nav2 yaw（基准）
  bool initial_yaw_recorded_ = false; // 是否已记录初始 yaw
  double yaw_offset_ = 0.0;        // 初始 yaw 与 startup 时 yaw 的差值

  



  void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void check_map_callback(const nav_msgs::msg::OccupancyGrid msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void check_zero();
  void send_control_task();
  void startup();

  // 从四元数提取 yaw 的辅助函数
  static double quaternion_to_yaw(double x, double y, double z, double w);


};
