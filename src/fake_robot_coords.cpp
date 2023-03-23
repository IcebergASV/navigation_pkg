#include <ros/ros.h>
#include <navigation_pkg/SimpleGPS.h> //temporary
#include <ros/console.h>

void fake_robot_coords() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation_pkg::SimpleGPS>("/rectbot_coords", 10);
    ros::Rate rate(10);
    navigation_pkg::SimpleGPS msg;
    msg.latitude = 47.0; // Example latitude value
    msg.longitude = 52.0; // Example longitude value
    msg.altitude = 50.0; // Example altitude value

    while (ros::ok()) {
        ROS_DEBUG_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_robot_coords_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    try {
        fake_robot_coords();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the angle_range_finder_node: " << e.what());
    }
    return 0;
}