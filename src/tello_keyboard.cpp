
/**
 * @file tello_keyboard.cpp
 * @author Ethan.lim
 * @brief publish command set
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <string>
#include <thread>

#include "ros/ros.h"
const std::string Cmd_help = R"(
    Tello command set
    - takeoff
    - landing
    - manual
    - stop


    ****manual controll****
    q   w   e       r: up
    a       d       f: down
        s           w: forward
                    s: backward
)";

int main(int argc, char **argv) {
    ros::init(argc, argv, "tello_key");
    ros::NodeHandle nh;
    ros::Publisher keyboard_pub;
    keyboard_pub = nh.advertise<std_msgs::String>("tello_keyboard", 1);
    std_msgs::String msg;

    ros::Rate rate(10);
    while (ros::ok()) {
        std::cout << Cmd_help << std::endl;
        std::cout << "command: ";
        std::getline(std::cin, msg.data);
        rate.sleep();
        keyboard_pub.publish(msg);
    }
    return 0;
}
