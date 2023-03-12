#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <navigation_pkg/PropInProgress.h>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "lidarPoint.h"

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
    float angle_safety_range = 0.0;
    float marker_distance_safety_range = 0.5; //should be slightly larger than radius of a marker
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

        //add a safety range onto the bounding box angles
        float index1_angle = prop_msg_.theta_1 + angle_safety_range;
        float index2_angle = prop_msg_.theta_2 - angle_safety_range
        // calculate the range indexes for the given theta angles
        float steps = (laser_angle_max * 2) / laser_angle_increment; 
        int index1 = (int)(((index1_angle + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);
        int index2 = (int)(((index2_angle + (laser_angle_max - (M_PI/2))) / (laser_angle_max*2))* steps);

        // check that the range indexes are within the range of the scan message and that index1 > index2
        if (index1 < 0 || index2 < 0 || index1 >= scan_msg.ranges.size() || index2 >= scan_msg.ranges.size() || index1 <= index2) {
            ROS_WARN("PropInProgress message range indexes are out of bounds for the given scan message");
            return;
        }

        //create a 2D vector containing distance angle pairs for points detected by lidar within the range provided by yolo
        std::vector<LidarPoint> scanPoints = createLidarPoints(scan_msgs.ranges, index1_angle, laser_angle_increment);
        
        //create a smaller vector of only points within the camera provided range
        std::vector<LidarPoint> selectedPoints;
        for (int i = index1; i <= index2; i++) {

            selectedPoints.push_back(scanPoints[i]);
        }

        //filter out points not likely to be part of the marker
        std::vector<LidarPoint> filteredPoints = filterPoints(selectedPoints, marker_distance_safety_range);
                
        


        //calculate the radius of the prop
        float radius = calculateRadius(filteredPoints);

        //Could add in check here to exclude the prop if the measured radius doesn't match expected radius

        // find the distance from the center of closest point and angle within the given range

        int i = 0;
        float closest_pnt = filteredPoints[i].distance;
        float closest_angle = filteredPoints[i].angle;
        for (int i = 0; i <= filteredPoints.size(); i < ++i) {
            if (std::isnan(filteredPoints[i].distance)) {
                continue; 
            }
            if(filteredPoints[i].distance < closest_pnt){
                closest_distance = filteredPoints[i].distance; 
                closest_angle = filteredPoints[i].angle;       
            }
        }

        navigation_pkg::PropInProgress closest_prop_msg;
        closest_prop_msg.prop_type = prop_msg_.prop_type;
        closest_prop_msg.theta_1 = prop_msg_.theta_1;
        closest_prop_msg.theta_2 = prop_msg_.theta_2;
        closest_prop_msg.closest_pnt_dist = closest_distance;
        closest_prop_msg.closest_pnt_angle = closest_angle;
        pub_prop_closest_.publish(closest_prop_msg);
    }


    std::vector<LidarPoint> createLidarPoints(const std::vector<double>& distances, double startAngle, double angleIncrement) {
        std::vector<LidarPoint> lidarPoints;

        // Add the first Lidar point
        LidarPoint firstPoint(distances[0], startAngle);
        lidarPoints.push_back(firstPoint);

        // Add the remaining Lidar points
        double currentAngle = startAngle + angleIncrement;
        for (size_t i = 1; i < distances.size(); i++) {
            double distance = distances[i];
            LidarPoint point(distance, currentAngle);
            lidarPoints.push_back(point);

            currentAngle += angleIncrement;
        }

    return lidarPoints;
    }

    std::vector<int> smallerVector(const std::vector<int>& largerVector, int startIndex, int endIndex) {
 
        if (startIndex > endIndex) {
            throw std::invalid_argument("Start index cannot be greater than end index");
        }

        std::vector<int> smallerVector;

        // Add the values from startIndex to endIndex (inclusive) to the smaller vector
        for (int i = startIndex; i <= endIndex; i++) {
            smallerVector.push_back(largerVector[i]);
        }

    return smallerVector;
    }

    std::vector<LidarPoint> filterPoints(const std::vector<LidarPoint>& inputVector, double range) {
        std::vector<LidarPoint> outputVector;
        double lowerBound = inputVector[0].distance;
        double upperBound = inputVector[0].distance;

        // Find the bounds of the range
        for (LidarPoint point : inputVector) {
            if (point.distance < lowerBound) {
                lowerBound = point.distance;
            }
            if (point.distance > upperBound) {
                upperBound = point.distance;
            }
        }

        // Add points to the output vector if their distance is within the range
        for (LidarPoint point : inputVector) {
            if (point.distance >= lowerBound && point.distance <= upperBound && std::abs(point.distance - lowerBound) <= range && std::abs(point.distance - upperBound) <= range) {
                outputVector.push_back(point);
            }
        }

    return outputVector;
    }
    
    float calculateRadius(const std::vector<LidarPoint>& points) {
        // Check that we have at least 6 points
        if (points.size() < 6) {
            throw std::runtime_error("At least 6 points are required to calculate the radius of a cylinder.");
        }

        // Convert angles to x,y coordinates on a unit circle
        std::vector<float> x_coords(points.size());
        std::vector<float> y_coords(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            x_coords[i] = std::cos(points[i].angle);
            y_coords[i] = std::sin(points[i].angle);
        }

        // Use linear regression to find the best-fit line for the x,y coordinates
        float x_mean = 0.0f;
        float y_mean = 0.0f;
        for (size_t i = 0; i < points.size(); ++i) {
            x_mean += x_coords[i];
            y_mean += y_coords[i];
        }
        x_mean /= points.size();
        y_mean /= points.size();

        float slope = 0.0f;
        float intercept = 0.0f;
        float numerator = 0.0f;
        float denominator = 0.0f;
        for (size_t i = 0; i < points.size(); ++i) {
            numerator += (x_coords[i] - x_mean) * (y_coords[i] - y_mean);
            denominator += std::pow(x_coords[i] - x_mean, 2);
        }
        slope = numerator / denominator;
        intercept = y_mean - slope * x_mean;

        // Calculate the center of the best-fit circle for the x,y coordinates
        float center_x = -slope / 2.0f;
        float center_y = intercept - slope * center_x;

        float radius = std::sqrt(std::pow(center_x, 2) + std::pow(center_y, 2));

        return radius;
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_finder_node");
    DistanceFinder distance_finder;
    distance_finder.spin();
    return 0;
}