#ifndef __TELLO_TCP__
#define __TELLO_TCP__
#include <tello_tcp.h>

void error(const char *msg) {
    perror(msg);
    exit(0);
}

/**
 * @brief connecting to socket
 *
 * @return true
 * @return false
 */
bool Listener::StationConnect(void) {
    portno = atoi(server_port_addr);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
        return false;
    }

    server = gethostbyname(server_addr);
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR connecting");
        return false;
    }
    return true;
}

/**
 * @brief send and receive packet protocel
 *
 * @param _TAKEOFF_STATION : requiring takeoff signal to station
 *        _LANDING_STATION : requiring landing signal to station
 *
 * @protocol 'z' : takeoff signal to station , 'x' : landing signal to station
 *           'Z' or 'X' : mission complete signal packet , from station to drone
 */
void Listener::SendPacket(uint8_t type) {
    ros::Rate loop_rate(10);
    const int buf_size = 256;
    char send_command;
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        char buffer[buf_size];
        // choice protocol
        if (type == _TAKEOFF_STATION) {
            send_command = 'z';
        } else if (type == _LANDING_STATION) {
            send_command = 'x';
        }
        // send signal to station
        if (write(sockfd, &send_command, sizeof(send_command)) < 0) {
            error("ERROR writing to socket");
        }
        // take a echo signal from station
        if (echoMode) {
            bzero(buffer, buf_size);
            if (read(sockfd, buffer, buf_size) < 0) {
                error("ERROR reading reply");
            } else {
                if (buffer[0] == 'Z' || buffer[0] == 'X') {
                    station_complete_flag_ = true;
                    return;
                }
            }
        }
    }
}
#endif
