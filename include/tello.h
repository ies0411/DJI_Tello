/**
 * @file tello.h
 * @author Ethan.lim
 * @brief header file of tello command node
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __TELLO_H__
#define __TELLO_H__
#include <fcntl.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <stdio.h>
#include <tello_pid.h>
#include <tello_tcp.h>
#include <termios.h>
#include <tf/tf.h>
#include <tf2/convert.h>
#include <unistd.h>

#include <../src/tello_pid_library.cpp>
#include <../src/tello_tcp.cpp>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <future>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

/*****preprocessing*****/
// convert radian to degree
#define RAD_TO_DEG(RAD) (RAD) * (180.f) / (M_PI)
// convert degree to radian
#define DEG_TO_RAD(DEG) (DEG) * (M_PI) / (180.f)
// express 360 degree to Radian
#define RAD_360 (2 * M_PI)

enum InfoDirType {
    _X,
    _Y,
    _Z,
    _YAW,
};
/**class**/
class TelloVM {
   private:
    ros::NodeHandle nh_;
    /**publisher**/
    ros::Publisher takeoff_pub_, landing_pub_, vel_cmd_pub_;
    /**subscriber**/
    ros::Subscriber command_sub_, aruco_pose_sub_;

    /**msgs**/
    std_msgs::Empty takeoff_msgs_, landing_msgs_;
    geometry_msgs::PoseStamped aruco_pose_;
    geometry_msgs::Twist vel_msgs_;

    /**variable**/
    bool finish_flag_ = false;
    Eigen::Vector4d vel_input_;

    /**variable**/
    ros::Time key_time_, Aruco_time_, smc_begin_;
    char key_input_;
    bool stop_flag_ = false, current_status_check_ = false;

    /***instance***/
    PID linear_speed_pid;
    PID th_w_pid;
    PID z_linear_speed_pid;
    Listener station_tcp_;

    /**pid parameter set**/
    double_t pid_dt = 0.1, pid_max = 0.5, pid_min = 0.0, pid_Kp = 2, pid_Kd = 5, pid_Ki = 0.0;
    double_t pid_th_dt = 0.1, pid_th_max = 0.6, pid_th_min = 0.0, pid_th_Kp = 2, pid_th_Kd = 0.5, pid_th_Ki = 0.0;

    /**function**/
    void CommandCallback(const std_msgs::String::ConstPtr& msg);
    bool Takeoff(void);
    bool Landing(void);
    bool ManualControl(void);
    int Getch(void);
    void ArucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void TrackingControl(void);
    void TransRadTOPitoPi(double_t& before_rad);
    void ZeroVelocity(void);
    void TwoPointDistance(const std::vector<std::pair<double, double>>& point, double_t& distance);
    void Distance(const std::vector<std::pair<double, double>>& point, double_t& distance);
    bool RotateAlgorithm(double_t& goal_yaw, double_t& yaw, double_t threshold, double_t& yaw_vel);
    void MarkerLanding(double_t x_goal, double_t y_goal, double_t z_goal, double_t yaw_goal);
    void StationTakeOff(void);
    bool CheckException(const uint16_t& param_num, const std::vector<std::string>& param);

   protected:
   public:
    TelloVM();
    ~TelloVM();
    void SendVelToDrone(void);
    bool SetFinishflag(void);
};

TelloVM::TelloVM() {
    /**init**/
    ROS_INFO("init");
    linear_speed_pid.PID_set(pid_dt, pid_max, pid_min, pid_Kp, pid_Kd, pid_Ki);
    th_w_pid.PID_set(pid_th_dt, pid_th_max, pid_th_min, pid_th_Kp, pid_th_Kd, pid_th_Ki);

    key_time_ = ros::Time::now();
    Aruco_time_ = ros::Time::now();
    vel_input_ = Eigen::Vector4d::Zero();

    /**pub**/
    landing_pub_ = nh_.advertise<std_msgs::Empty>("/tello/land", 1);
    takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/tello/takeoff", 1);
    vel_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);

    /**sub**/
    aruco_pose_sub_ = nh_.subscribe("/aruco_single/pose", 1, &TelloVM::ArucoCallback, this);
    command_sub_ = nh_.subscribe("tello_keyboard", 1, &TelloVM::CommandCallback, this);
    ROS_INFO("init_completed");
}

TelloVM::~TelloVM() {
    ROS_INFO_STREAM("terminate");
}
#endif
