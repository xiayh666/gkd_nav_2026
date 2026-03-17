#include "rotate_fix.h"
#include "game_status.h"
#include <iostream>
using namespace std;

/*
double curyaw;
double inityaw = 10;


void get_rotate(const ReceiveNavigationInfo& pkg)
{
    if (inityaw == 10)
    {
        inityaw = pkg.yaw;
    }
    curyaw = pkg.yaw;
    hp = pkg.hp;
    start_status = pkg.start;
    // std::cout << hp << std::endl;
}

pcl::PointXYZ fixRotate(pcl::PointXYZ point)
{
    double x = point.x;
    double y = point.y;
    double rad = curyaw - inityaw;
    point.x = x * cos(rad) - y * sin(rad);
    point.y = x * sin(rad) + y * cos(rad);
    return point;
}

double getDeltaYaw()
{
    return curyaw - inityaw;
}
    */


