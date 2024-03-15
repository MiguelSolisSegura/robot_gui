#pragma once
#define CVUI_IMPLEMENTATION
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "robot_gui/cvui.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"


class CVUIROSPublisher {
    public:
        CVUIROSPublisher();
        void run();

    private:
        void infoCallback(const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg);
        ros::Publisher vel_pub_;
        geometry_msgs::Twist vel_msg_;
        ros::Subscriber info_sub_;
        robotinfo_msgs::RobotInfo10Fields info_msg_;
        const std::string WINDOW_NAME = "Robot Control GUI";
};