#include "control.h"
#include "rotate_fix.h"
#include <iostream>
#include <cmath>
#include "protocal.h"
#include <rclcpp/rclcpp.hpp>
#include <string>



socket_server<ReceiveNavigationInfo> ser(11456, [](const ReceiveNavigationInfo& info) {
    RCLCPP_INFO(rclcpp::get_logger("socket_server"), "Received from UDP: yaw=%.2f, hp=%.2f, start=%d", info.yaw, info.hp, info.start);
});
void sendControl(double angle, double speed)
{
    double x = cos(angle) * speed;
    double y = sin(angle) * speed;
    SendNavigationInfo pkg{};
    pkg.header = 0x37;
    pkg.vx = x;
    pkg.vy = y;
    ser.send<SendNavigationInfo>(pkg);
}

void sendControlXY(double x, double y)
{
    SendNavigationInfo pkg{};
    pkg.header = 0x37;
    pkg.vx = x;
    pkg.vy = y;
    pkg.wz = 1;
    pkg.robot_mode = ROBOT_SEARCH;

    ser.send<SendNavigationInfo>(pkg);
}


