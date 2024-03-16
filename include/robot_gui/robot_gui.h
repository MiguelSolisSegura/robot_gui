#pragma once
#define CVUI_IMPLEMENTATION
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "robot_gui/cvui.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"


class CVUIROSPublisher {
    public:
        CVUIROSPublisher();
        void run();

    private:
        // Callbacks
        void infoCallback(const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg);
        void odomCallback(const nav_msgs::OdometryConstPtr &msg);
        // Clients
        ros::ServiceClient get_client_;
        ros::ServiceClient reset_client_;
        std_srvs::Trigger srv_;
        // Publishers
        ros::Publisher vel_pub_;
        // Subscribers
        ros::Subscriber info_sub_;
        ros::Subscriber odom_sub_;
        // Messages
        geometry_msgs::Twist vel_msg_;
        robotinfo_msgs::RobotInfo10Fields info_msg_;
        std::string distance_msg_;
        // Positions
        float pos_x_;
        float pos_y_;
        float pos_z_;
        // GUI variables
        const std::string WINDOW_NAME = "Robot Control GUI";
};