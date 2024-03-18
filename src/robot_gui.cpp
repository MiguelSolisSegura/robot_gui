#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include <string>

// Constructor
CVUIROSPublisher::CVUIROSPublisher() {
    ros::NodeHandle nh;
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    info_sub_ = nh.subscribe("/robot_info", 1, &CVUIROSPublisher::infoCallback, this);
    info_msg_.data_field_01 = "No data received in /robot_info.";
    odom_sub_ = nh.subscribe("/odom", 1, &CVUIROSPublisher::odomCallback, this);
    pos_x_ = 0.0;
    pos_y_ = 0.0;
    pos_z_ = 0.0;
    get_client_ = nh.serviceClient<std_srvs::Trigger>("/get_distance");
    reset_client_ = nh.serviceClient<std_srvs::Trigger>("/reset_distance");
    distance_msg_ = "0.00";
}

// Robot info message callback
void CVUIROSPublisher::infoCallback(const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg) {
    ROS_DEBUG("Info message was updated");
    info_msg_ = *msg;
}

// Odometry message callback
void CVUIROSPublisher::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    ROS_DEBUG("Odom message was updated");
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    pos_z_ = msg->pose.pose.position.z;
}

void CVUIROSPublisher::run() {
    ROS_INFO("Starting GUI");
    cv::Mat frame = cv::Mat(400, 680, CV_8UC3);
    int count = 0;

    // Init a OpenCV window and tell CVUI to use it
    cv::namedWindow(WINDOW_NAME);
    cvui::init(WINDOW_NAME);

    while (ros::ok()) {
        // Fill the frame with color
        frame = cv::Scalar(194, 194, 194);

        // Robot control buttons
        if (cvui::button(frame, 250, 10, 80, 80, "LF")) {
            vel_msg_.linear.x += 0.1;
            vel_msg_.angular.z += 0.1;
        }
        if (cvui::button(frame, 340, 10, 80, 80, "F")) {
            vel_msg_.linear.x += 0.1;
        }
        if (cvui::button(frame, 430, 10, 80, 80, "RF")) {
            vel_msg_.linear.x += 0.1;
            vel_msg_.angular.z += -0.1;
        }
        if (cvui::button(frame, 250, 100, 80, 80, "L")) {
            vel_msg_.angular.z += 0.1;
        }
        if (cvui::button(frame, 340, 100, 80, 80, "S")) {
            vel_msg_.linear.x = 0.0;
            vel_msg_.angular.z = 0.0;
        }
        if (cvui::button(frame, 430, 100, 80, 80, "R")) {
            vel_msg_.angular.z += -0.1;
        }
        if (cvui::button(frame, 250, 190, 80, 80, "LB")) {
            vel_msg_.linear.x += -0.1;
            vel_msg_.angular.z += 0.1;
        }
        if (cvui::button(frame, 340, 190, 80, 80, "B")) {
            vel_msg_.linear.x += -0.1;
        }
        if (cvui::button(frame, 430, 190, 80, 80, "RB")) {
            vel_msg_.linear.x += -0.1;
            vel_msg_.angular.z += -0.1;
        }

        // Window for robot_info messages
        cvui::window(frame, 10, 10, 230, 380, "Robot Information");
        cvui::printf(frame, 15, 35, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_01.c_str());
        cvui::printf(frame, 15, 55, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_02.c_str());
        cvui::printf(frame, 15, 75, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_03.c_str());
        cvui::printf(frame, 15, 95, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_04.c_str());
        cvui::printf(frame, 15, 115, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_05.c_str());
        cvui::printf(frame, 15, 135, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_06.c_str());
        cvui::printf(frame, 15, 155, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_07.c_str());
        cvui::printf(frame, 15, 175, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_08.c_str());
        cvui::printf(frame, 15, 195, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_09.c_str());
        cvui::printf(frame, 15, 215, 0.4, 0xCBCBCB, "%s", info_msg_.data_field_10.c_str());

        // Windows for velocities monitoring
        cvui::window(frame, 250, 280, 125, 110, "Linear Velocity");
        if (vel_msg_.linear.x >= 0) {
            cvui::printf(frame, 265, 320, 0.9, 0xFFB900, "+%.2f", vel_msg_.linear.x);
        }
        else {
            cvui::printf(frame, 265, 320, 0.9, 0x00F0FF, "%.2f", vel_msg_.linear.x);
        }
        cvui::text(frame, 284, 360, "(m/s)", 0.6, 0xCBCBCB);
        
        cvui::window(frame, 385, 280, 125, 110, "Angular Velocity");
        if (vel_msg_.angular.z >= 0) {
            cvui::printf(frame, 400, 320, 0.9, 0xFFB900, "+%.2f", vel_msg_.angular.z);
        }
        else {
            cvui::printf(frame, 400, 320, 0.9, 0x00F0FF, "%.2f", vel_msg_.angular.z);
        }
        cvui::text(frame, 412, 360, "(rad/s)", 0.6, 0xCBCBCB);

        // Window for distance monitoring
        cvui::window(frame, 520, 10, 150, 80, "Traveled Distance");

        // Buttons for distance interaction
        if (cvui::button(frame, 520, 100, 150, 50, "Update Distance")) {
            if (get_client_.call(srv_)) {
                distance_msg_ = srv_.response.message;
            }
            else {
                ROS_ERROR("Failed to call service GET distance_tracker_service");
            }
        }
        cvui::printf(frame, 530, 50, 0.8, 0xCBCBCB, "%s m", distance_msg_.c_str());
        if (cvui::button(frame, 520, 160, 150, 50, "Reset Distance")) {
            if (reset_client_.call(srv_)) {
                distance_msg_ = srv_.response.message;
            }
            else {
                ROS_ERROR("Failed to call service RESET distance_tracker_service");
            }
        }
        

        // Windows for coordinates
        cvui::window(frame, 520, 220, 150, 50, "Coordinate: X");
        cvui::printf(frame, 525, 245, 0.6, 0xFF0000, pos_x_ >= 0 ? "+%.3f m" : "%.3f m", pos_x_);

        cvui::window(frame, 520, 280, 150, 50, "Coordinate: Y");
        cvui::printf(frame, 525, 305, 0.6, 0x00FF00, pos_y_ >= 0 ? "+%.3f m" : "%.3f m", pos_y_);

        cvui::window(frame, 520, 340, 150, 50, "Coordinate: Z");
        cvui::printf(frame, 525, 365, 0.6, 0x0000FF, pos_z_ >= 0 ? "+%.3f m" : "%.3f m", pos_z_);

        // Update CVUI internal state
        cvui::update();

        // Show everything on the screen
        cv::imshow(WINDOW_NAME, frame);

        // Check if ESC key was pressed
        if (cv::waitKey(20) == 27) {
        break;
        }

        // Publish velocities
        vel_pub_.publish(vel_msg_);

        // Update callbacks
        ros::spinOnce();
    }
}
