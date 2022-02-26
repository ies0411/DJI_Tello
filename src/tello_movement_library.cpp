#ifndef __TELLO_MOVE__
#define __TELLO_MOVE__

/**
 * @file tello_movement_library.cpp
 * @author Ethan.lim
 * @brief library related with movement
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <tello.h>

/**
 * @brief function to calculate distance between two points
 *
 * @param point : vector included in points
 * @param distance : output
 */
void TelloVM::TwoPointDistance(const std::vector<std::pair<double, double>>& point, double_t& distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2));
}
/**
 * @brief interface function publishing velocity value
 *
 */
void TelloVM::SendVelToDrone(void) {
    vel_msgs_.linear.x = vel_input_[InfoDirType::_X];
    vel_msgs_.linear.y = vel_input_[InfoDirType::_Y];
    vel_msgs_.linear.z = vel_input_[InfoDirType::_Z];
    vel_msgs_.angular.z = vel_input_[InfoDirType::_YAW];
    vel_cmd_pub_.publish(vel_msgs_);
}

/**
 * @brief change radian value range from -pi to pi
 *
 * @param before_rad : radian type
 */
void TelloVM::TransRadTOPitoPi(double_t& before_rad) {
    while (true) {
        if (before_rad > M_PI) {
            before_rad = before_rad - (M_PI * 2.0);
        } else if (before_rad < (-1) * M_PI) {
            before_rad = before_rad + (M_PI * 2.0);
        } else {
            return;
        }
    }
}
/**
 * @brief  function to calculate distance between three points
 *
 * @param point : vector included in points
 * @param distance : output
 */
void TelloVM::Distance(const std::vector<std::pair<double, double>>& point, double_t& distance) {
    distance = sqrt(pow(point[0].first - point[0].second, 2) + pow(point[1].first - point[1].second, 2) + pow(point[2].first - point[2].second, 2));
}

/**
 * @brief function making drone to rotate close to the nearest direction
 *
 * @param goal_yaw : goal yaw value
 * @param yaw : current yaw value
 * @param threshold : if drone is in threshold, yaw_vel is '0'
 * @param yaw_vel : ouput
 * @return true : yaw value is in threshold
 * @return false : yaw value is out of threshold
 */
bool TelloVM::RotateAlgorithm(double_t& goal_yaw, double_t& yaw, double_t threshold, double_t& yaw_vel) {
    TransRadTOPitoPi(goal_yaw);
    TransRadTOPitoPi(yaw);

    double_t diff_yaw = goal_yaw - yaw;
    TransRadTOPitoPi(diff_yaw);
    double_t fabs_th = fabs(diff_yaw);
    double_t turn_pid;
    turn_pid = th_w_pid.calculate(0.0, fabs_th);

    if (fabs_th < threshold) {
        yaw_vel = 0;
        return true;
    } else {
        if (diff_yaw < 0) {
            yaw_vel = (-1) * turn_pid;
            return false;
        } else {
            yaw_vel = turn_pid;
            return false;
        }
    }
}
/**
 * @brief takeoff from station
 *
 */
void TelloVM::StationTakeOff(void) {
    station_tcp_.station_complete_flag_ = false;
    if (station_tcp_.StationConnect()) {
        station_tcp_.SendPacket(_TAKEOFF_STATION);
        if (station_tcp_.station_complete_flag_ == true) {
            if (!Takeoff()) {
                StationTakeOff();
                current_status_check_ == false;
                return;
            }

            ros::Duration(5.0).sleep();
            station_tcp_.SendPacket(_LANDING_STATION);
        }
    } else {
        ROS_ERROR_STREAM("connection failed");
    }
}

void TelloVM::MarkerLanding(double_t x_goal, double_t y_goal, double_t z_goal, double_t yaw_goal) {
    station_tcp_.station_complete_flag_ = false;

    if (station_tcp_.StationConnect()) {
        ros::Rate MarkerLanding(10);
        bool station_check = false;
        pid_dt = 0.1, pid_max = 0.4, pid_min = 0.0, pid_Kp = 0.27, pid_Kd = 0.7, pid_Ki = 0.0;
        pid_th_dt = 0.1, pid_th_max = 0.6, pid_th_min = 0.0, pid_th_Kp = 0.1, pid_th_Kd = 0.7, pid_th_Ki = 0.0;
        linear_speed_pid.PID_set(pid_dt, pid_max, pid_min, pid_Kp, pid_Kd, pid_Ki);
        th_w_pid.PID_set(pid_th_dt, pid_th_max, pid_th_min, pid_th_Kp, pid_th_Kd, pid_th_Ki);
        while (ros::ok()) {
            ros::spinOnce();
            MarkerLanding.sleep();
            ros::Duration duration = ros::Time::now() - Aruco_time_;
            if (stop_flag_ == true) {
                ZeroVelocity();
                stop_flag_ = false;
                current_status_check_ == false;
                return;
            }
            if (duration.toSec() < 0.2) {
                tf::Quaternion q(
                    aruco_pose_.pose.orientation.x,
                    aruco_pose_.pose.orientation.y,
                    aruco_pose_.pose.orientation.z,
                    aruco_pose_.pose.orientation.w);
                tf::Matrix3x3 m(q);
                double_t roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                TransRadTOPitoPi(roll);
                TransRadTOPitoPi(pitch);
                TransRadTOPitoPi(yaw);
                yaw = -pitch;

                Eigen::MatrixXf Tr(2, 2), pre_pose(2, 1), Tr_pose(2, 1), pre_goal_pose(2, 1), Tr_goal_pose(2, 1);
                double_t th = -yaw;

                /** coordinate translation**/
                Tr << cos(th), -sin(th),
                    sin(th), cos(th);

                double_t transfer_pose_x = aruco_pose_.pose.position.x;
                double_t transfer_pose_y = aruco_pose_.pose.position.y;

                pre_goal_pose << x_goal, y_goal;
                Tr_goal_pose = Tr * pre_goal_pose;

                double_t transfer_pose_x_goal = Tr_goal_pose(0);
                double_t transfer_pose_y_goal = Tr_goal_pose(1);

                std::vector<std::pair<double_t, double_t>> point;

                point.emplace_back(std::make_pair(transfer_pose_x, transfer_pose_x_goal));
                point.emplace_back(std::make_pair(transfer_pose_y, transfer_pose_y_goal));
                point.emplace_back(std::make_pair(aruco_pose_.pose.position.z, 0));

                double_t diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
                double_t diff_pose_y = transfer_pose_y_goal - transfer_pose_y;
                double_t diff_pose_z = z_goal - aruco_pose_.pose.position.z;

                double_t distance;
                Distance(point, distance);
                double_t x_y_distance;
                TwoPointDistance(point, x_y_distance);

                double_t tan_xy_z = atan2(diff_pose_z, x_y_distance);
                double_t tan_degree = atan2(diff_pose_y, diff_pose_x);
                double_t speed_pid = linear_speed_pid.calculate(0, distance);
                if (x_y_distance < 0.5) {
                    // ZeroVelocity();
                    if (station_check == false) {
                        station_check = true;

                        ROS_INFO("station_thread");
                    } else {
                        // if ((station_tcp_.station_complete_flag_ == true) && (x_y_distance < 0.3)) {
                        if (x_y_distance < 0.15) {
                            ROS_INFO("sendpacket & echo completed");
                            ZeroVelocity();
                            Landing();
                            ros::Duration(5.0).sleep();

                            station_tcp_.station_complete_flag_ = false;
                            current_status_check_ == false;
                            return;
                        }
                        ROS_INFO("inside threshhold");
                        vel_input_[InfoDirType::_Z] = speed_pid * sin(tan_xy_z);
                        vel_input_[InfoDirType::_Y] = (fabs(speed_pid * cos(tan_xy_z))) * sin(tan_degree);
                        vel_input_[InfoDirType::_X] = (fabs(speed_pid * cos(tan_xy_z))) * cos(tan_degree);
                    }
                } else {
                    vel_input_[InfoDirType::_Z] = speed_pid * sin(tan_xy_z);
                    vel_input_[InfoDirType::_Y] = (fabs(speed_pid * cos(tan_xy_z))) * sin(tan_degree);
                    vel_input_[InfoDirType::_X] = (fabs(speed_pid * cos(tan_xy_z))) * cos(tan_degree);
                }
                if (RotateAlgorithm(yaw_goal, yaw, DEG_TO_RAD(1), vel_input_[InfoDirType::_YAW])) {
                    vel_input_[InfoDirType::_YAW] = 0;
                }
                std::cout << aruco_pose_ << std::endl;
            } else {
                ZeroVelocity();
            }
        }
    } else {
        ROS_ERROR_STREAM("connection fail!");
    }
}

/**
 * @brief fucntion of tracking arucomakrer
 *
 */

void TelloVM::TrackingControl(void) {
    ros::Rate tracking_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        tracking_rate.sleep();
        ros::Duration duration = ros::Time::now() - Aruco_time_;
        if (stop_flag_ == true) {
            ZeroVelocity();
            stop_flag_ = false;
            current_status_check_ == false;
            return;
        }

        if (duration.toSec() < 0.2) {
            tf::Quaternion q(
                aruco_pose_.pose.orientation.x,
                aruco_pose_.pose.orientation.y,
                aruco_pose_.pose.orientation.z,
                aruco_pose_.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double_t roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            TransRadTOPitoPi(roll);
            TransRadTOPitoPi(pitch);
            TransRadTOPitoPi(yaw);
            yaw = -pitch;

            Eigen::MatrixXf Tr(2, 2), pre_pose(2, 1), Tr_pose(2, 1), pre_goal_pose(2, 1), Tr_goal_pose(2, 1);
            double_t th = -yaw;

            /** coordinate translation**/
            Tr << cos(th), -sin(th),
                sin(th), cos(th);
            pre_pose << aruco_pose_.pose.position.x, aruco_pose_.pose.position.y;
            Tr_pose = Tr * pre_pose;

            double_t transfer_pose_x = Tr_pose(0);
            double_t transfer_pose_y = Tr_pose(1);
            double_t x_goal = 0.0, y_goal = -0.5, z_goal = 0.0, yaw_goal = 0.0;
            pre_goal_pose << x_goal, y_goal;
            Tr_goal_pose = Tr * pre_goal_pose;

            double_t transfer_pose_x_goal = Tr_goal_pose(0);
            double_t transfer_pose_y_goal = Tr_goal_pose(1);

            std::vector<std::pair<double_t, double_t>> point;

            point.emplace_back(std::make_pair(transfer_pose_x, transfer_pose_x_goal));
            point.emplace_back(std::make_pair(transfer_pose_y, transfer_pose_y_goal));
            point.emplace_back(std::make_pair(aruco_pose_.pose.position.z, 0));

            double_t diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
            double_t diff_pose_y = transfer_pose_y_goal - transfer_pose_y;
            double_t diff_pose_z = z_goal - aruco_pose_.pose.position.z;

            double_t distance;
            Distance(point, distance);
            double_t x_y_distance;
            TwoPointDistance(point, x_y_distance);

            double_t tan_xy_z = atan2(diff_pose_z, x_y_distance);
            double_t tan_degree = atan2(diff_pose_y, diff_pose_x);
            double_t speed_pid = linear_speed_pid.calculate(0, distance);
            if (distance < 0.07) {
                ZeroVelocity();
            } else {
                vel_input_[InfoDirType::_Z] = speed_pid * sin(tan_xy_z);
                vel_input_[InfoDirType::_Y] = (fabs(speed_pid * cos(tan_xy_z))) * sin(tan_degree);
                vel_input_[InfoDirType::_X] = (fabs(speed_pid * cos(tan_xy_z))) * cos(tan_degree);
            }
            if (RotateAlgorithm(yaw_goal, yaw, DEG_TO_RAD(1), vel_input_[InfoDirType::_YAW])) {
                vel_input_[InfoDirType::_YAW] = 0;
            }
        } else {
            ZeroVelocity();
        }
    }
}

/**
 * @brief acquiring the pose filtered by aruco package
 *
 * @param msg : msgs type is posestamped
 */

void TelloVM::ArucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    aruco_pose_ = *msg;
    aruco_pose_.pose.position.x = -msg->pose.position.x;
    aruco_pose_.pose.position.y = -msg->pose.position.z;
    aruco_pose_.pose.position.z = msg->pose.position.y;
    Aruco_time_ = ros::Time::now();
}

/**
 * @brief function getting the keyboard input data for manual control
 *
 * @return int : output
 */
int TelloVM::Getch(void) {
    ros::Rate key_rate(5);
    while (ros::ok()) {
        tcflush(0, TCIFLUSH);
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        // Get the current character
        key_input_ = getchar();
        if (key_input_ == '\x03') {
            key_input_ = '\x03';

            return 1;
        }
        if (stop_flag_ == true) {
            stop_flag_ = false;
            return 1;
        }
        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        key_rate.sleep();
        ros::spinOnce();
        key_time_ = ros::Time::now();
    }
}

/**
 * @brief takeoff function
 *
 * @return true : suceed to takeoff
 * @return false : time out , fail to takeoff
 */
bool TelloVM::Takeoff(void) {
    ros::Rate move_rate(10);
    ros::Time takeoff_time = ros::Time::now();
    ros::Duration duration;
    while (ros::ok()) {
        move_rate.sleep();
        ros::spinOnce();
        duration = ros::Time::now() - takeoff_time;
        if (duration.toSec() > 5) {
            ROS_ERROR_STREAM("takeoff fail");
            current_status_check_ = false;
            return false;
        }
        takeoff_pub_.publish(takeoff_msgs_);

        if (takeoff_pub_.getNumSubscribers() > 0) {
            break;
        }
    }
    ROS_INFO_STREAM("takeoff success!");
    current_status_check_ = false;
    return true;
}
/**
 * @brief landing function
 *
 * @return true : suceed to landing
 * @return false : time out , fail to landing
 */
bool TelloVM::Landing(void) {
    ROS_INFO("Landing");
    ros::Rate move_rate(10);
    ros::Time landing_time = ros::Time::now();
    ros::Duration duration;
    while (ros::ok()) {
        move_rate.sleep();
        ros::spinOnce();
        duration = ros::Time::now() - landing_time;
        if (duration.toSec() > 5) {
            current_status_check_ == false;
            return false;
        }
        landing_pub_.publish(landing_msgs_);
        if (landing_pub_.getNumSubscribers() > 0) {
            break;
        }
    }
    current_status_check_ == false;
    return true;
}
/**
 * @brief input '0' velocity , hovering
 *
 */
void TelloVM::ZeroVelocity(void) {
    vel_input_[InfoDirType::_X] = 0;
    vel_input_[InfoDirType::_Y] = 0;
    vel_input_[InfoDirType::_Z] = 0;
    vel_input_[InfoDirType::_YAW] = 0;
}
/**
 * @brief manual control
 *
 * @return true : terminate normally
 * @return false : time out
 * 'p':stop
 * 'w':forward
 * 's':backward
 * 'a':left
 * 'd':right
 * 'r':turn cw
 * 'q':turn ccw
 * 'r':up
 * 'f':down
 */
bool TelloVM::ManualControl(void) {
    char key(' ');
    ros::Rate mcotrol_rate(5);
    ROS_INFO_STREAM("manual");
    std::thread thread_t1(&TelloVM::Getch, this);
    thread_t1.detach();
    ros::Duration duration;
    while (ros::ok()) {
        std::cout << aruco_pose_ << std::endl;
        duration = ros::Time::now() - key_time_;
        if (duration.toSec() > 0.5) {
            key = 'p';

        } else if (duration.toSec() > 30) {
            current_status_check_ == false;
            return false;
        } else {
            key = key_input_;
        }
        if (stop_flag_ == true) {
            stop_flag_ = false;
            ZeroVelocity();
            break;
        }

        switch (key) {
            case '\x03': {
                ROS_INFO("finish_parent_key");
                ZeroVelocity();
                current_status_check_ == false;
                return true;
            }
            case 'r': {
                vel_input_[InfoDirType::_X] = 0;
                vel_input_[InfoDirType::_Y] = 0;
                vel_input_[InfoDirType::_Z] = 0.5;
                vel_input_[InfoDirType::_YAW] = 0;

                break;
            }
            case 'f':
                vel_input_[InfoDirType::_X] = 0;
                vel_input_[InfoDirType::_Y] = 0;
                vel_input_[InfoDirType::_Z] = -0.5;
                vel_input_[InfoDirType::_YAW] = 0;

                break;
            case 'w':
                vel_input_[InfoDirType::_X] = 0;
                vel_input_[InfoDirType::_Y] = 0.5;
                vel_input_[InfoDirType::_Z] = 0;
                vel_input_[InfoDirType::_YAW] = 0;
                break;
            case 's':
                vel_input_[InfoDirType::_X] = 0;
                vel_input_[InfoDirType::_Y] = -0.5;
                vel_input_[InfoDirType::_Z] = 0;
                vel_input_[InfoDirType::_YAW] = 0;

                break;
            case 'a':
                vel_input_[InfoDirType::_X] = -0.5;
                vel_input_[InfoDirType::_Y] = 0;
                vel_input_[InfoDirType::_Z] = 0;
                vel_input_[InfoDirType::_YAW] = 0;
                break;
            case 'd':
                vel_input_[InfoDirType::_X] = 0.5;
                vel_input_[InfoDirType::_Y] = 0;
                vel_input_[InfoDirType::_Z] = 0;
                vel_input_[InfoDirType::_YAW] = 0;
                break;
            case 'q':
                vel_input_[InfoDirType::_X] = 0;
                vel_input_[InfoDirType::_Y] = 0;
                vel_input_[InfoDirType::_Z] = 0;
                vel_input_[InfoDirType::_YAW] = -0.5;
                break;
            case 'e':
                vel_input_[InfoDirType::_X] = 0;
                vel_input_[InfoDirType::_Y] = 0;
                vel_input_[InfoDirType::_Z] = 0;
                vel_input_[InfoDirType::_YAW] = +0.5;
                break;

            default:
                ZeroVelocity();

                break;
        }

        mcotrol_rate.sleep();
        ros::spinOnce();
    }
    current_status_check_ == false;
    return true;
}
#endif
