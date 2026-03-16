#pragma once

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "socket_server.hpp"

struct SendNavigationInfo
{
    uint8_t header;
    float vx;
    float vy;
} __attribute__((packed));
void sendControl(double angle, double speed);
void sendControlXY(double x, double y);