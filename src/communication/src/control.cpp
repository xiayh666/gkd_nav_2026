#include "control.h"
#include "rotate_fix.h"
#include <iostream>
#include <cmath>



socket_server<ReceiveNavigationInfo> ser(11456, get_rotate);
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

    ser.send<SendNavigationInfo>(pkg);
}

