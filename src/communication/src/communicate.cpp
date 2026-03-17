#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include <iomanip>
#include "control.h"
#include "protocal.h"
#include "socket_server.hpp"
#include <atomic>
#include <unistd.h>
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;

std::atomic<bool> stop_requested(false);

class CmdVelSubscriber : public rclcpp::Node
{
public:
    CmdVelSubscriber()
    : Node("cmd_vel_subscriber")
    {
        // 创建订阅（QoS depth=10）
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/red_standard_robot1/cmd_vel",
            rclcpp::QoS(1),
            std::bind(&CmdVelSubscriber::listener_callback, this, std::placeholders::_1)
        );

        // 创建定时器：每 0.1 秒调用 check_zero
        check_zero_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&CmdVelSubscriber::check_zero, this)
        );

        // 初始化状态
        latest_msg_ = nullptr;
        latest_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME); // 无效时间

        hp_publisher_ = rclcpp::Node::create_publisher<std_msgs::msg::Int32>("robot_hp", 10);

        /*
        hp_timer_ = rclcpp::Node::create_wall_timer(
            1s,
            [this]() {
                RCLCPP_INFO(this->get_logger(), "发布血量更新: 100");
                std_msgs::msg::Int32 msg;
                msg.data = 20;
                hp_publisher_->publish(msg);
            }
        );
        */


        // 启动输出线程
        output_thread_ = std::thread(&CmdVelSubscriber::output_task, this);


    }

    ~CmdVelSubscriber()
    {
        if (output_thread_.joinable()) {
            output_thread_.join();
        }
    }

private:
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_time_ = this->now();
        latest_msg_ = std::make_shared<geometry_msgs::msg::Twist>(*msg); // 深拷贝
    }

    void check_zero()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_time_.nanoseconds() == 0) {
            return; // 从未收到消息
        }
        rclcpp::Duration diff = this->now() - latest_time_;
        if (diff.seconds() > 0.3) {
            latest_msg_.reset(); // 等价于 Python 的 = None
        }
    }

    void output_task()
    {
        while (!stop_requested && rclcpp::ok()) {
            auto start_time = std::chrono::high_resolution_clock::now();
            std::shared_ptr<geometry_msgs::msg::Twist> local_msg;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                local_msg = latest_msg_; // 复制智能指针（线程安全读取）
            }

            if (!local_msg) {
                sendControlXY(0, 0);
            } else {
                // std::cout << "Received cmd_vel:\n"
                //           << "  linear.x:  " << std::fixed << std::setprecision(3) << local_msg->linear.x << "\n"
                //           << "  linear.y:  " << local_msg->linear.y << "\n"
                //           << "  angular.z: " << local_msg->angular.z << std::endl;
                sendControlXY(local_msg->linear.y, local_msg->linear.x);
                // sendControlXY( 0.1, 0);


               std::cout << "cmd_vel: " << "(" << -local_msg->linear.y << ", " << -local_msg->linear.x << ", " << ")" << std::endl;
            
            }

            std::this_thread::sleep_for(1ms-(std::chrono::high_resolution_clock::now()-start_time)); // ≈ sleep(0.01)
        }
        std::cout << "end" << std::endl;
        rclcpp::shutdown();
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr check_zero_timer_;
    std::thread output_thread_;
    std::mutex mutex_;

    // 共享状态（受 mutex 保护）
    std::shared_ptr<geometry_msgs::msg::Twist> latest_msg_;
    rclcpp::Time latest_time_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hp_publisher_;
    rclcpp::TimerBase::SharedPtr hp_timer_;

};

void handle_sigint(int signum)
{
    if (signum == SIGINT)
    {
        stop_requested = true;
        sendControlXY(0, 0);
   
    }
}

int main() 
{
    rclcpp::init(0, nullptr);
    signal(SIGINT, handle_sigint);
    auto node = std::make_shared<CmdVelSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}
