#include <ros/ros.h>
#include <navigation/Compass.h> //temporary

void fake_compass_headings() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation::Compass>("/rectbot_heading", 10);
    ros::Rate rate(10);
    navigation::Compass msg;
    msg.heading = 5.235987756;

    while (ros::ok()) {
        ROS_INFO_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_compass_headings_node");
    try {
        fake_compass_headings();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_compass_headings_node: " << e.what());
    }
    return 0;
}