#pragma once

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "socket_server.hpp"

void sendControl(double angle, double speed);
void sendControlXY(double x, double y);