#include <ros/ros.h>
#include <navigation_pkg/yolo.h> 
#include <ros/console.h>

void fake_yolo_boxes() {
    //for testing angle finder only - remove eventually
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation_pkg::yolo>("/yolo", 10);
    ros::Rate rate(10);
    navigation_pkg::yolo msg;
    msg.label = "red_marker";
    msg.probability = 0.91; 
    msg.xmin = 100;
    msg.xmax = 300;
    msg.ymin = 200;
    msg.ymax = 400;

    while (ros::ok()) {
        ROS_DEBUG_STREAM(msg);
        pub.publish(msg);
        rate.sleep();
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_yolo_boxes_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    try {
        fake_yolo_boxes();
    }
    catch (ros::Exception& e) {
        ROS_ERROR_STREAM("An exception occurred in the fake_yolo node: " << e.what());
    }
    return 0;
}