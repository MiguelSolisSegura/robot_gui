#include "robot_gui/robot_gui.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_button_clicks_publisher");
  CVUIROSPublisher button_clicks_publisher;
  button_clicks_publisher.run();
  return 0;
}