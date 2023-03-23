#include <ros/ros.h>
#include <navigation_pkg/Compass.h> //temporary
#include <ros/console.h>

void fake_compass_headings() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation_pkg::Compass>("/rectbot_heading", 10);
    ros::Rate rate(10);
    navigation_pkg::Compass msg;
    msg.heading = 5.235987756;

    while (ros::ok()) {
        ROS_DEBUG_STREAM("fake_compass_headings: " << msg.heading );
        pub.publish(msg);
        rate.sleep();
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_compass_headings_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    try {
        fake_compass_headings();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_compass_headings_node: " << e.what());
    }
    return 0;
}