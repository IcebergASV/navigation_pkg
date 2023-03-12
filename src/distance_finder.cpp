
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <navigation_pkg/PropInProgress.h>
#include <cmath>

class DistanceFinder {
public:
    DistanceFinder() : nh_(""), private_nh_("~") {
        // get ROS parameters
        private_nh_.param<std::string>("prop_topic", prop_topic_, "/prop_angle_range");
        private_nh_.param<std::string>("scan_topic", scan_topic_, "/rect_bot/laser/scan");
        private_nh_.param<double>("max_range", max_range_, 10.0);
        private_nh_.param<double>("laser_angle_min", laser_angle_min, -M_PI/2.0);
        private_nh_.param<double>("laser_angle_max", laser_angle_max, M_PI/2.0);

        sub_scan_ = nh_.subscribe(scan_topic_, 1, &DistanceFinder::scanCallback, this);
        sub_prop_ = nh_.subscribe(prop_topic_, 1, &DistanceFinder::propCallback, this);
        pub_prop_closest_ = nh_.advertise<navigation_pkg::PropInProgress>("/prop_closest_point", 1);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_prop_;
    ros::Publisher pub_prop_closest_;
    std::string prop_topic_;
    std::string scan_topic_;
    double max_range_;
    double laser_angle_min;
    double laser_angle_max;
    double laser_angle_increment;
    navigation_pkg::PropInProgress prop_msg_;
    sensor_msgs::LaserScan scan_msg;

    void propCallback(const navigation_pkg::PropInProgress::ConstPtr& msg) {
        // save the PropInProgress message for later use
        prop_msg_ = *msg;
        ROS_INFO_STREAM("Received PropInProgress message with theta_1=" << prop_msg_.theta_1
            << " and theta_2=" << prop_msg_.theta_2);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // save the scan message for later use
        scan_msg = *msg;
        laser_angle_increment = scan_msg.angle_increment;

        // check if the PropInProgress message is valid
        if (prop_msg_.prop_type.empty() || std::isnan(prop_msg_.theta_1) || std::isnan(prop_msg_.theta_2)) {
            ROS_WARN("Invalid PropInProgress message received");
            return;
        }

        // calculate the range indexes for the given theta angles
        float steps = (laser_angle_max * 2) / laser_angle_increment; 
        int index1 = (int)(((prop_msg_.theta_1 + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);
        int index2 = (int)(((prop_msg_.theta_2 + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);

        // check that the range indexes are within the range of the scan message
        if (index1 < 0 || index2 < 0 || index1 >= scan_msg.ranges.size() || index2 >= scan_msg.ranges.size()) {
            ROS_WARN("PropInProgress message range indexes are out of bounds for the given scan message");
            return;
        }

        // find the closest point and angle within the given range

        int i = 0;
        float closest_pnt = scan_msg.ranges[i];
        int closest_angle_index = 0;
        for (i = index1; i <= index2; ++i) {
            if (std::isnan(scan_msg.ranges[i])) {
                continue; 
            }
            if(scan_msg.ranges[i] < closest_pnt){
                closest_pnt = scan_msg.ranges[i];
                closest_angle_index = i;
                
                
            }
        }

        // convert index to angle, how many degrees from 90.
        float closest_angle = ((closest_angle_index/steps)*(laser_angle_max*2)) - (laser_angle_max);


        navigation_pkg::PropInProgress closest_prop_msg;
        closest_prop_msg.prop_type = prop_msg_.prop_type;
        closest_prop_msg.theta_1 = prop_msg_.theta_1;
        closest_prop_msg.theta_2 = prop_msg_.theta_2;
        closest_prop_msg.closest_pnt_dist = closest_pnt;
        closest_prop_msg.closest_pnt_angle = closest_angle;
        pub_prop_closest_.publish(closest_prop_msg);
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_finder_node");
    DistanceFinder distance_finder;
    distance_finder.spin();
    return 0;
}