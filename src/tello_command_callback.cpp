
#include "tello_movement_library.cpp"

/**
 * @brief before staring mission, checking exception
 *
 * @param param_num number of token
 * @param param command set by token
 * @return true
 * @return false
 */
bool TelloVM::CheckException(const uint16_t& param_num, const std::vector<std::string>& param) {
    if (current_status_check_ == true) {
        ROS_ERROR_STREAM("busy!");
        return false;
    }
    if (param.size() != param_num) {
        ROS_ERROR_STREAM("param number error");
        return false;
    }
    return true;
}

/**
 * @brief callback function of keyboard node
 *
 * @param msg is string type which is command set
 */
void TelloVM::CommandCallback(const std_msgs::String::ConstPtr& msg) {
    std::istringstream iss(msg->data);
    std::string token;
    std::vector<std::string> param;
    /**msg token by space**/
    while (getline(iss, token, ' ')) {
        param.emplace_back(token);
    }
    /**if command is 'takeoff', go to Takeoff function**/
    if (param[0] == "takeoff") {
        if (!CheckException(1, param)) {
            return;
        }
        current_status_check_ = true;
        std::async(std::launch::async, &TelloVM::Takeoff, this);
        ROS_INFO_STREAM("parents finish");
    }
    // takeoff from station
    if (param[0] == "stationtakeoff") {
        if (!CheckException(1, param)) {
            return;
        }
        current_status_check_ = true;
        std::async(std::launch::async, &TelloVM::StationTakeOff, this);
        ROS_INFO_STREAM("parents finish");

    }
    /**if command is 'landing', go to Lakeoff function**/
    else if (param[0] == "landing") {
        if (!CheckException(1, param)) {
            return;
        }
        current_status_check_ = true;
        std::async(std::launch::async, &TelloVM::Landing, this);
        ROS_INFO_STREAM("parents finish");

    }
    /**if command is 'manual', go to ManualControl function**/
    else if (param[0] == "manual") {
        if (!CheckException(1, param)) {
            return;
        }
        current_status_check_ = true;
        std::async(std::launch::async, &TelloVM::ManualControl, this);
        ROS_INFO_STREAM("parents finish");
    }
    /**if command is 'markerlanding', go to ManualControl function**/
    else if (param[0] == "markerlanding") {
        if (!CheckException(5, param)) {
            return;
        }
        current_status_check_ = true;
        // void TelloVM::MarkerLanding(double_t & x_goal, double_t & x_goal, double_t & x_goal, double_t & yaw_goal) {
        std::async(std::launch::async, &TelloVM::MarkerLanding, this, stod(param[1]), stod(param[2]), stod(param[3]), stod(param[4]));
        ROS_INFO_STREAM("parents finish");
    }
    /**if command is 'tracking', go to TrackingControl function**/
    else if (param[0] == "tracking") {
        if (!CheckException(1, param)) {
            return;
        }
        current_status_check_ = true;
        std::async(std::launch::async, &TelloVM::TrackingControl, this);
        ROS_INFO_STREAM("parents finish");
    }
    /**if want drone to stop**/
    else if (param[0] == "stop") {
        if (!CheckException(1, param)) {
            return;
        }
        stop_flag_ = true;
    }
    /**if want whole process to terminate**/
    else if (param[0] == "finish") {
        if (!CheckException(1, param)) {
            return;
        }
        finish_flag_ = true;
    } else {
        ROS_ERROR_STREAM("Incorrect CMD");
    }
}
/**
 * @brief set finish flag function , flag is private valuable so, this is interface of accessing to flag
 *
 * @return true
 * @return false
 */
bool TelloVM::SetFinishflag(void) {
    return finish_flag_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tello_cmd");
    ROS_INFO("start");
    TelloVM tellovm;
    // framerate 10HZ
    ros::Rate rate(10);
    while (ros::ok()) {
        tellovm.SendVelToDrone();
        if (tellovm.SetFinishflag() == true) {
            exit(0);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
