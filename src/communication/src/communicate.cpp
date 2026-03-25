#include "communicate.hpp"
#include "consts.hpp"
#include "protocal.h"
#include "socket_server.hpp"
#include <cstdlib>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>


// 临时测试用
bool flag = true;
bool flag_timer_created_ = false;


std::atomic<bool> CmdVelSubscriber::stop_requested_ = false;
std::atomic<bool> CmdVelSubscriber::auto_aim_captured_ = false;
std::atomic<int> CmdVelSubscriber::game_progress_ = false;
std::atomic<bool> CmdVelSubscriber::hit_to_start_ = false;
std::atomic<bool> CmdVelSubscriber::startup_finished_ = false;
std::atomic<bool> CmdVelSubscriber::startup_timer_created_ = false;
std::atomic<double> CmdVelSubscriber::nav2_yaw_ = 0.0;
bool dead = false;

std::shared_ptr<CmdVelSubscriber>
    CmdVelSubscriber::cmd_vel_subscriber_instance_ = nullptr;

CmdVelSubscriber::CmdVelSubscriber() : Node("cmd_vel_subscriber") {

  hp_publisher_ =
      rclcpp::Node::create_publisher<std_msgs::msg::Int32>("robot_hp", 10);

  ser_ = std::make_shared<SocketServer<ReceiveNavigationInfo>>(
      11456, [this](const ReceiveNavigationInfo &info) {
        auto_aim_captured_ = info.auto_aim_captured;
        int hp = info.hp;
        int is_dead = info.is_dead;

        if (hp < HP_MAX) {
          hit_to_start_ = true;
          // RCLCPP_INFO(rclcpp::get_logger("socket_server"),
          //             "Received from UDP: yaw=%.2f, hp=%.2f, start=%d, "
          //             "auto_aim_captured=%d, hit_to_start_ = %d",
          //             info.yaw, info.hp, info.game_progress,
          //             info.auto_aim_captured, hit_to_start_ == true);
        }

        // RCLCPP_INFO(rclcpp::get_logger("socket_server"),
        //             "Received from UDP: yaw=%.2f, hp=%.2f, start=%d, "
        //             "auto_aim_captured=%d, hit_to_start_ = %d",
        //             info.yaw, info.hp, info.game_progress,
        //             info.auto_aim_captured, hit_to_start_ == true);

        game_progress_ = info.game_progress;

        auto hp_msg = std_msgs::msg::Int32();

        hp_msg.data = int(hp);

        if (hp_publisher_) {
          hp_publisher_->publish(hp_msg);
        }

        if (is_dead) {
          dead = true;
          respawn_init_ = true;
          RCLCPP_INFO(this->get_logger(),
                      "Received is_dead = true, set dead = true");
        } else if (dead) {
          dead = false;

          respawn_timer_ = this->create_wall_timer(4s, [this]() {
            if (respawn_timer_) {
              respawn_timer_->cancel();
            }

            respawn_init_ = false;
            RCLCPP_INFO(this->get_logger(), "reset respawn_init_ to false");
          });
        }
      });

  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/red_standard_robot1/cmd_vel", rclcpp::QoS(1),
      std::bind(&CmdVelSubscriber::listener_callback, this,
                std::placeholders::_1));

  subscription_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/red_standard_robot1/map", rclcpp::QoS(1),
      std::bind(&CmdVelSubscriber::check_map_callback, this,
                std::placeholders::_1));

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/red_standard_robot1/odometry", rclcpp::QoS(1),
      std::bind(&CmdVelSubscriber::odom_callback, this, std::placeholders::_1));

  check_zero_timer_ = this->create_wall_timer(
      100ms, std::bind(&CmdVelSubscriber::check_zero, this));

  latest_msg_ = nullptr;
  latest_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME); // 无效时间

  send_control_thread_ =
      std::thread(&CmdVelSubscriber::send_control_task, this);
}

CmdVelSubscriber::~CmdVelSubscriber() {
  if (send_control_thread_.joinable()) {
    send_control_thread_.join();
  }
}

void CmdVelSubscriber::init_finished() { initialised_ = true; }

std::shared_ptr<CmdVelSubscriber> CmdVelSubscriber::get_instance() {
  if (!CmdVelSubscriber::cmd_vel_subscriber_instance_) {
    auto instance = std::make_shared<CmdVelSubscriber>();
    cmd_vel_subscriber_instance_ = instance;
    return instance;
  } else {
    return CmdVelSubscriber::cmd_vel_subscriber_instance_;
  }
}

void CmdVelSubscriber::handle_sigint(int signum) {
  if (signum == SIGINT) {
    if (!cmd_vel_subscriber_instance_) {
      std::cout << "received sigint but cmd_vel_subscriver is nullptr"
                << std::endl;
    } else {
      cmd_vel_subscriber_instance_->stop_requested_ = true;
      send_control(0, 0);
      std::cout << "received sigint, terminating..." << std::endl;
    }
  }
}

void CmdVelSubscriber::listener_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_time_ = this->now();
  latest_msg_ = std::make_shared<geometry_msgs::msg::Twist>(*msg);
}

void CmdVelSubscriber::check_map_callback(
    const nav_msgs::msg::OccupancyGrid msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg.data.empty()) {
    // RCLCPP_INFO(this->get_logger(), "map msg received");
    init_finished();
  } else {
    RCLCPP_INFO(this->get_logger(), "no map msg");
  }
}

double CmdVelSubscriber::quaternion_to_yaw(double x, double y, double z,
                                           double w) {
  // 从四元数提取 yaw (绕 z 轴旋转)
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void CmdVelSubscriber::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // 获取 nav2 的 yaw（来自激光雷达 IMU）
  const auto &q = msg->pose.pose.orientation;
  double yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);
  nav2_yaw_.store(yaw);

  // 在第一次收到 odometry 时记录初始 yaw（基准方向）
  if (!initial_yaw_recorded_) {
    initial_yaw_ = yaw;
    initial_yaw_recorded_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial nav2 yaw recorded: %.2f",
                initial_yaw_);
  }
}

void CmdVelSubscriber::check_zero() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_time_.nanoseconds() == 0) {
    return;
  }
  rclcpp::Duration diff = this->now() - latest_time_;
  if (diff.seconds() > 0.3) {
    latest_msg_.reset();
  }
}

void CmdVelSubscriber::send_control(double x, double y) {
  if (flag_timer_created_ == false) {
    RCLCPP_INFO(cmd_vel_subscriber_instance_->get_logger(),
                "Creating flag reset timer");
    flag_timer_created_ = true;
    cmd_vel_subscriber_instance_->flag_timer_ = cmd_vel_subscriber_instance_->create_wall_timer(2s, [=]() {
      flag = false;
    });
  }

  if ((game_progress_ != 4 and !hit_to_start_) or game_progress_ == 5) {
    SendNavigationInfo pkg;
    pkg.vx = pkg.vy = 0;
    pkg.enable_rotate = false;
    pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;
    pkg.enable_auto_aim = false;

    cmd_vel_subscriber_instance_->ser_->send<SendNavigationInfo>(pkg);
    return;
  }
  if (!startup_finished_) {
    if (!startup_timer_created_) {
      if (!cmd_vel_subscriber_instance_) {
        std::cout << "cmd_vel_subscriver_instane_ = nullptr" << std::endl;
      } else {
        cmd_vel_subscriber_instance_->startup();
      }
    } else {
      // 使用 yaw 的变化量来修正冲刺方向
      double current_nav2_yaw = nav2_yaw_.load();
      double delta_yaw = current_nav2_yaw - cmd_vel_subscriber_instance_->initial_yaw_;
      double speed = 1.5;
      // 沿着初始方向（世界坐标系 x 轴）冲刺，补偿 yaw 变化
      double vy = speed * std::cos(delta_yaw);
      double vx = -speed * std::sin(delta_yaw);

      RCLCPP_INFO(rclcpp::get_logger("startup"),
                  "Startup: initial_yaw=%.2f, current_yaw=%.2f, delta_yaw=%.2f, vx=%.2f, vy=%.2f",
                  cmd_vel_subscriber_instance_->initial_yaw_, current_nav2_yaw, delta_yaw, vx, vy);

      SendNavigationInfo pkg;
      pkg.vx = vx;
      pkg.vy = vy;
      pkg.header = 0x37;
      pkg.enable_rotate = false;
      pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;
      pkg.enable_auto_aim = false;

      cmd_vel_subscriber_instance_->ser_->send<SendNavigationInfo>(pkg);
    }
    return;
  }

  if (!cmd_vel_subscriber_instance_) {
    std::cout << "cmd_vel_subscriver_instane_ = nullptr" << std::endl;
  } else {
    SendNavigationInfo pkg{};
    pkg.header = 0x37;
    pkg.vx = x;
    pkg.vy = y;
    if (get_instance()->respawn_init_) {
      pkg.vx = 0;
      pkg.vy = 0;
      pkg.enable_rotate = 0;
      pkg.robot_mode = ROBOT_INIT;

    } else if ((CmdVelSubscriber::get_instance()->initialised_)) {
      pkg.enable_rotate = 1;
      pkg.robot_mode = CmdVelSubscriber::auto_aim_captured_
                           ? ROBOT_FOLLOW_GIMBAL
                           : ROBOT_SEARCH;
    } else {
      pkg.enable_rotate = 0;
      pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;
    }

    // pkg.enable_rotate = 0;

    // pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;

    // pkg.robot_mode = ROBOT_SEARCH;

    pkg.enable_auto_aim = cmd_vel_subscriber_instance_->auto_aim_captured_;

    // std::cout << (pkg.robot_mode == ROBOT_FOLLOW_GIMBAL ? "FOLLOW_GIMBAL"
    //                                                     : "SEARCH")
    //           << std::endl
    //           << (pkg.enable_auto_aim ? "ENABLE_AUTOAIM" : "DISABLE_AUTOAIM")
    //           << std::endl;
    cmd_vel_subscriber_instance_->ser_->send<SendNavigationInfo>(pkg);
  }
}

void CmdVelSubscriber::send_control_task() {
  while (!stop_requested_ && rclcpp::ok()) {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::shared_ptr<geometry_msgs::msg::Twist> local_msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_msg = latest_msg_;
    }

    if (!local_msg) {
      send_control(0, 0);
    } else {
      send_control(local_msg->linear.y, local_msg->linear.x);
      // send_control(0, 0);

      // std::cout << "cmd_vel: "
      //           << "(" << local_msg->linear.y << ", " << local_msg->linear.x
      //           << ", "
      //           << ")" << std::endl;
    }

    std::this_thread::sleep_for(
        1ms - (std::chrono::high_resolution_clock::now() - start_time));
  }
  std::cout << "end" << std::endl;
  rclcpp::shutdown();
}

void CmdVelSubscriber::startup() {
  // 计算初始 yaw 与当前 nav2 yaw 的差值，用于修正冲刺方向
  double current_nav2_yaw = nav2_yaw_.load();
  yaw_offset_ = initial_yaw_ - current_nav2_yaw;
  RCLCPP_INFO(
      this->get_logger(),
      "Startup: initial_yaw=%.2f, current_nav2_yaw=%.2f, yaw_offset=%.2f",
      initial_yaw_, current_nav2_yaw, yaw_offset_);

  startup_timer_ = this->create_wall_timer(2s, [this]() {
    startup_finished_ = true;
    startup_timer_->cancel();

    RCLCPP_INFO(this->get_logger(),
                "startup finished, startup_timer cancelled");
  });
  startup_timer_created_ = true;
}

int main() {
  rclcpp::init(0, nullptr);
  signal(SIGINT, &CmdVelSubscriber::handle_sigint);
  auto cmd_vel_subscriber = CmdVelSubscriber::get_instance();

  rclcpp::spin(cmd_vel_subscriber);
  rclcpp::shutdown();

  return 0;
}
