#include <ros/ros.h>
#include <navigation_pkg/PropInProgress.h>
#include <cmath>
#include <ros/console.h>



void fake_bbox_angles() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation_pkg::PropInProgress>("prop_angle_range", 1);
    ros::Rate rate(10);
    navigation_pkg::PropInProgress msg;
    bool red_marker = true;//variable used to switch between publishing red and green markers

 

    while (ros::ok()) {
        if (red_marker){
            msg.prop_type = "red_marker";
            msg.theta_1 = 91*(M_PI/180); 
            msg.theta_2 = 140*(M_PI/180);
            red_marker = false;
        }
        else{
            msg.prop_type = "green_marker";
            msg.theta_1 = 45*(M_PI/180); 
            msg.theta_2 = 89*(M_PI/180); 
            red_marker = true;
        }
        ROS_DEBUG_STREAM("fake_bbox_angles: theta_1 - " << msg.theta_1 << "theta_2 - " << msg.theta_2);
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fake_bbox_angles_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    try {
        fake_bbox_angles();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_bbox_angles_node: " << e.what());
    }
    return 0;
}