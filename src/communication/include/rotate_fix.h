#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
struct ReceiveNavigationInfo
{
    uint8_t header;
    float yaw;
    float pitch;
    float hp;
    bool start;
} __attribute__((packed));  
pcl::PointXYZ fixRotate(pcl::PointXYZ point);
void get_rotate(const ReceiveNavigationInfo& pkg);
double getDeltaYaw();