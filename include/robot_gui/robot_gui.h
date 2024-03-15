#pragma once

#include "ros/subscriber.h"
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"

class CVUIROSPublisher {
    public:
        CVUIROSPublisher();
        void run();

    private:
        void infoCallback(const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg);
        ros::Publisher pub_;
        ros::Subscriber info_sub_;
        robotinfo_msgs::RobotInfo10Fields info_msg_;
        const std::string WINDOW_NAME = "Robot Control GUI";
};