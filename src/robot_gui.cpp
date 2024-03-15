#include "robot_gui/robot_gui.h"
#include "ros/init.h"
//robotinfo_msgs::RobotInfo10Fields

CVUIROSPublisher::CVUIROSPublisher() {
    // Initialize ROS node
    ros::NodeHandle nh;
    pub_ = nh.advertise<std_msgs::String>("cvui_button_clicks", 10);
    info_sub_ = nh.subscribe("/robot_info", 1, &CVUIROSPublisher::infoCallback, this);
    info_msg_.data_field_01 = "No data received in /robot_info.";
}

void CVUIROSPublisher::infoCallback(const robotinfo_msgs::RobotInfo10FieldsConstPtr &msg) {
    ROS_INFO("Data message was updated");
    info_msg_ = *msg;

}

void CVUIROSPublisher::run() {
    cv::Mat frame = cv::Mat(400, 680, CV_8UC3);
    int count = 0;

    // Init a OpenCV window and tell cvui to use it.
    cv::namedWindow(WINDOW_NAME);
    cvui::init(WINDOW_NAME);

    while (ros::ok()) {
        // Fill the frame with color
        frame = cv::Scalar(194, 194, 194);

        // Robot control buttons
        if (cvui::button(frame, 250, 10, 80, 80, "LF")) {
        // The button was clicked, so let's increment our counter and publish a
        // message.
        count++;
        std_msgs::String msg;
        msg.data = "Button clicked " + std::to_string(count) + " times.";
        pub_.publish(msg);
        }
        if (cvui::button(frame, 340, 10, 80, 80, "F")) {
        }
        if (cvui::button(frame, 430, 10, 80, 80, "RF")) {
        }
        if (cvui::button(frame, 250, 100, 80, 80, "L")) {
        }
        if (cvui::button(frame, 340, 100, 80, 80, "S")) {
        }
        if (cvui::button(frame, 430, 100, 80, 80, "R")) {
        }
        if (cvui::button(frame, 250, 190, 80, 80, "LB")) {
        }
        if (cvui::button(frame, 340, 190, 80, 80, "B")) {
        }
        if (cvui::button(frame, 430, 190, 80, 80, "RB")) {
        }

        // Window for robot_info messages
        cvui::window(frame, 10, 10, 230, 380, "Robot Information");
        cvui::printf(frame, 15, 35, 0.4, 0xFFB900, "%s", info_msg_.data_field_01.c_str());
        cvui::printf(frame, 15, 55, 0.4, 0xFFB900, "%s", info_msg_.data_field_02.c_str());
        cvui::printf(frame, 15, 75, 0.4, 0xFFB900, "%s", info_msg_.data_field_03.c_str());
        cvui::printf(frame, 15, 95, 0.4, 0xFFB900, "%s", info_msg_.data_field_04.c_str());
        cvui::printf(frame, 15, 115, 0.4, 0xFFB900, "%s", info_msg_.data_field_05.c_str());
        cvui::printf(frame, 15, 135, 0.4, 0xFFB900, "%s", info_msg_.data_field_06.c_str());
        cvui::printf(frame, 15, 155, 0.4, 0xFFB900, "%s", info_msg_.data_field_07.c_str());
        cvui::printf(frame, 15, 175, 0.4, 0xFFB900, "%s", info_msg_.data_field_08.c_str());
        cvui::printf(frame, 15, 195, 0.4, 0xFFB900, "%s", info_msg_.data_field_09.c_str());
        cvui::printf(frame, 15, 215, 0.4, 0xFFB900, "%s", info_msg_.data_field_10.c_str());

        // Windows for velocities monitoring
        cvui::window(frame, 250, 280, 125, 110, "Linear Velocity");
        cvui::window(frame, 385, 280, 125, 110, "Angular Velocity");

        // Window for distance monitoring
        cvui::window(frame, 520, 10, 150, 80, "Traveled Distance");

        // Buttons for distance interaction
        if (cvui::button(frame, 520, 100, 150, 50, "Update Distance")) {
        }
        if (cvui::button(frame, 520, 160, 150, 50, "Reset Distance")) {
        }

        // Windows for coordinates
        cvui::window(frame, 520, 220, 150, 50, "Coordinate: X");
        cvui::window(frame, 520, 280, 150, 50, "Coordinate: Y");
        cvui::window(frame, 520, 340, 150, 50, "Coordinate: Z");

        // Update CVUI internal state
        cvui::update();

        // Show everything on the screen
        cv::imshow(WINDOW_NAME, frame);

        // Check if ESC key was pressed
        if (cv::waitKey(20) == 27) {
        break;
        }

        // Update callbacks
        ros::spinOnce();
    }
}
