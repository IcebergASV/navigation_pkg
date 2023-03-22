#include <ros/ros.h>
#include <navigation_pkg/yolo.h>
#include <navigation_pkg/PropInProgress.h>
#include <cmath>


class AngleFinder {
public:
    AngleFinder() : nh_(""), private_nh_("~") 
    {
        yolo_sub_ = nh_.subscribe("/yolo", 1, &AngleFinder::yoloCallback, this);
        prop_pub_ = nh_.advertise<navigation_pkg::PropInProgress>("/prop_angles", 1);
        private_nh_.param<double>("realsense_fov", realsense_fov, 0.0);
        private_nh_.param<int>("realsense_res_x", realsense_res_x, 0);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void yoloCallback(const navigation_pkg::yolo::ConstPtr& msg)
    {
        //get the position of the bounding box
        x_min = msg->xmin;
        x_max = msg->xmax;

        // Calculate the angle range for the prop
        double theta_right = fov_end - ((x_max / realsense_res_x) * realsense_fov); 
        double theta_left = fov_end - ((x_min / realsense_res_x) * realsense_fov);
        
        // Create and publish the Prop message with the prop coordinates
        navigation_pkg::PropInProgress prop_msg;
        prop_msg.prop_type = msg->label; //assign object classification label to the prop
        prop_msg.theta_1 = theta_right;
        prop_msg.theta_2 = theta_left;
        prop_pub_.publish(prop_msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber yolo_sub_;
    ros::Publisher prop_pub_;
    double x_min;
    double x_max;
    double realsense_fov;
    double fov_end = (M_PI / 2) + (realsense_fov / 2 );
    int realsense_res_x;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_finder_node");
    AngleFinder angle_finder;
    angle_finder.spin();
    return 0;
}