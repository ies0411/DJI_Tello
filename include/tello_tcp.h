/**
 * @file tello_tcp.h
 * @author Ethan
 * @brief Tcp/Ip communication with station , Header file
 * @version 0.1
 * @date 2021-12-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __TELLO_TCP_H__
#define __TELLO_TCP_H__

#include <netdb.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "std_msgs/String.h"

/**enum**/
enum _Info_Station_Status_Type {
    _TAKEOFF_STATION,
    _LANDING_STATION,
};
class Listener {
   private:
    char topic_message[256] = {0};
    int sockfd, portno;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    bool echoMode = true;
    char server_addr[12] = "192.168.0.7";
    char server_port_addr[3] = "23";

   public:
    void SendPacket(uint8_t type);
    bool StationConnect(void);
    bool station_complete_flag_ = false;
};
#endif
