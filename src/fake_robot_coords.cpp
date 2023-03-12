#include <ros/ros.h>
#include <navigation/SimpleGPS.h> //temporary

void fake_robot_coords() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation::SimpleGPS>("/rectbot_coords", 10);
    ros::Rate rate(10);
    navigation::SimpleGPS msg;
    msg.latitude = 47.0; // Example latitude value
    msg.longitude = 52.0; // Example longitude value
    msg.altitude = 50.0; // Example altitude value

    while (ros::ok()) {
        ROS_INFO_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_robot_coords_node");
    try {
        fake_robot_coords();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the angle_range_finder_node: " << e.what());
    }
    return 0;
}