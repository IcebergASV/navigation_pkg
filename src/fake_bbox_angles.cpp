#include <ros/ros.h>
#include <navigation_pkg/PropInProgress.h>


void fake_bbox_angles() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation_pkg::PropInProgress>("prop_angle_range", 1);
    ros::Rate rate(10);
    navigation_pkg::PropInProgress msg;
    msg.prop_type = "marker";
    msg.theta_1 = 1.0472; //60
    msg.theta_2 = 2.0944; //120

    while (ros::ok()) {
        ROS_INFO_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_bbox_angles_node");
    try {
        fake_bbox_angles();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_bbox_angles_node: " << e.what());
    }
    return 0;
}