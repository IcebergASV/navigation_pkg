#include <ros/ros.h>
#include <navigation_pkg/BoundingBox.h> 
#include <navigation_pkg/BoundingBoxes.h>
#include <ros/console.h>

void fake_yolo_boxes() {
    //for testing angle finder only - remove eventually
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<navigation_pkg::BoundingBoxes>("/yolo", 10);
    ros::Rate rate(10);
    navigation_pkg::BoundingBoxes yolo;
    navigation_pkg::BoundingBox box1;
    navigation_pkg::BoundingBox box2;
    box1.label = "red_marker";
    box1.probability = 0.91; 
    box1.xmin = 100;
    box1.xmax = 300;
    box1.ymin = 200;
    box1.ymax = 400;

    box2.label = "green_marker";
    box2.probability = 0.86; 
    box2.xmin = 400;
    box2.xmax = 500;
    box2.ymin = 200;
    box2.ymax = 450;

    yolo.bounding_boxes.push_back(box1);
    yolo.bounding_boxes.push_back(box2);

    while (ros::ok()) {
        ROS_DEBUG_STREAM(yolo);
        pub.publish(yolo);
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