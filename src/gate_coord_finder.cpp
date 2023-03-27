#include <ros/ros.h>
#include <navigation_pkg/Compass.h> 
#include <navigation_pkg/PropInProgress.h>
#include <navigation_pkg/Prop.h>
#include <navigation_pkg/GateInProgress.h>
#include <navigation_pkg/Gate.h>
#include <navigation_pkg/SimpleGPS.h> //temporary
#include <sensor_msgs/NavSatFix.h>
#include <cmath> 
#include <ros/console.h>
#include "gpsPoint.h"
#include "lidarPoint.h"
#include <std_msgs/Float64.h>



class GateCoordFinder {
public:
    GateCoordFinder() : nh_(""), private_nh_("~") 
    {
        gps_sub_ = nh_.subscribe("/mavros/global_position/global", 1, &GateCoordFinder::gpsCallback, this);
        compass_sub_ = nh_.subscribe("/mavros/global_position/compass_hdg", 1, &GateCoordFinder::compassCallback, this );
        prop_sub_ = nh_.subscribe("/gate_closest_point", 1, &GateCoordFinder::propCallback, this);
        prop_pub_ = nh_.advertise<navigation_pkg::Gate>("/completed_gates", 1);
        private_nh_.param<double>("coord_mapping_error_estimation", coord_mapping_error_estimation, 0.0); 
        private_nh_.param<double>("degrees_lat_per_meter", degrees_lat_per_meter, 0.0);
        private_nh_.param<double>("degrees_lon_per_meter", degrees_lon_per_meter, 0.0);
        
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) 
    //void gpsCallback(const navigation_pkg::SimpleGPS::ConstPtr& msg) 
    {
        robot_lat_ = msg->latitude;
        robot_lon_ = msg->longitude;
        robot_alt_ = msg->altitude;
        ROS_DEBUG_STREAM("Robot Latitude: " << robot_lat_ << "Robot Longitude: " << robot_lon_);
    }

    void compassCallback(const std_msgs::Float64::ConstPtr& msg)
    //void compassCallback(const navigation_pkg::Compass::ConstPtr& msg) 
    {
        robot_heading = msg->data;
    }

    void propCallback(const navigation_pkg::GateInProgress::ConstPtr& msg)
    {
        //get lidar points for both markers
        lidarPoint close_marker_lidar_point;
        lidarPoint far_marker_lidar_point;
        close_marker_lidar_point.setDistance(msg->closer_marker.closest_pnt_dist);
        close_marker_lidar_point.setAngle(msg->closer_marker.closest_pnt_angle);
        far_marker_lidar_point.setDistance(msg->farther_marker.closest_pnt_dist);
        far_marker_lidar_point.setAngle(msg->farther_marker.closest_pnt_angle);

        //// Calculate the GPS coordinates of each marker
        gpsPoint close_marker_coords = mapProp(close_marker_lidar_point);
        gpsPoint far_marker_coords = mapProp(far_marker_lidar_point);

        //calculate coordinates of midpoint of each marker - to do
        gpsPoint midpoint = calculateMidpoint(close_marker_coords, far_marker_coords);

        //mid point safety range



        double lat_safety_range = degrees_lat_per_meter * coord_mapping_error_estimation;
        double lon_safety_range = degrees_lon_per_meter * coord_mapping_error_estimation;



        navigation_pkg::Gate gate_msg;

        gate_msg.closer_marker.prop_type = "marker";
        gate_msg.closer_marker.prop_coords.latitude = close_marker_coords.getLatitude();
        gate_msg.closer_marker.prop_coords.longitude = close_marker_coords.getLongitude();
        gate_msg.closer_marker.prop_coords.altitude = robot_alt_;

        gate_msg.farther_marker.prop_type = "marker";
        gate_msg.farther_marker.prop_coords.latitude = far_marker_coords.getLatitude();
        gate_msg.farther_marker.prop_coords.longitude = far_marker_coords.getLongitude();
        gate_msg.farther_marker.prop_coords.altitude = robot_alt_;


        gate_msg.midpoint_coords.latitude = midpoint.getLatitude();
        gate_msg.midpoint_coords.longitude = midpoint.getLongitude();
        gate_msg.midpoint_coords.altitude = robot_alt_;

        gate_msg.midpoint_range.min_latitude = midpoint.getLatitude() - lat_safety_range;
        gate_msg.midpoint_range.max_latitude = midpoint.getLongitude() + lat_safety_range;
        gate_msg.midpoint_range.min_longitude = midpoint.getLatitude() - lon_safety_range;
        gate_msg.midpoint_range.max_longitude = midpoint.getLongitude() + lon_safety_range;

        prop_pub_.publish(gate_msg);
    }

    gpsPoint mapProp(const lidarPoint& marker) {
 
        double dist = marker.getDistance();
        double angle = marker.getAngle();
        double prop_heading;
        
        if ((robot_heading - angle) > (2*M_PI))
            prop_heading = robot_heading - angle - (2* M_PI);
        else 
            prop_heading = robot_heading - angle;

        double north_dist = dist * cos(prop_heading);
        double east_dist = dist * sin(prop_heading);

        double lat_diff = north_dist * degrees_lat_per_meter;
        double lon_diff = east_dist * degrees_lon_per_meter;

        
        double prop_lat = robot_lat_ + lat_diff;
        double prop_lon = robot_lon_ + lon_diff;

        gpsPoint prop_coords;

        prop_coords.setLatitude(prop_lat);
        prop_coords.setLongitude(prop_lon);
        
        return prop_coords;

    }
    double deg_to_rad(double deg){

        double conversion_value_rad = deg*(M_PI/180);
        return conversion_value_rad;

    }



    double rad_to_deg(double rad){

        double conversion_value_deg = rad*(180/M_PI);    
        return conversion_value_deg;

    }



    gpsPoint calculateMidpoint(const gpsPoint& set1, const gpsPoint& set2){

        //Convert latitude and longitude to radians
        double lat1_rad = deg_to_rad(set1.getLatitude());
        double long1_rad = deg_to_rad(set1.getLongitude());
        double lat2_rad = deg_to_rad(set2.getLatitude());
        double long2_rad = deg_to_rad(set2.getLongitude());

        //Get midpoints in radians
        double avg_latitude_rad = (lat1_rad+lat2_rad)/2;
        double avg_longitude_rad = (long1_rad+long2_rad)/2;

        //Convert the midpoint to latitude and longitude back to degrees
        double midpoint_lat = rad_to_deg(avg_latitude_rad);
        double midpoint_long = rad_to_deg(avg_longitude_rad);

        gpsPoint midpoint(midpoint_lat, midpoint_long);
        return midpoint;

    };

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber gps_sub_;
    ros::Subscriber prop_sub_;
    ros::Subscriber compass_sub_;
    ros::Publisher prop_pub_;
    double robot_lat_;
    double robot_lon_;
    double robot_alt_;
    double robot_heading;
    double coord_mapping_error_estimation;
    double degrees_lat_per_meter;
    double degrees_lon_per_meter;




};



int main(int argc, char** argv) {
    ros::init(argc, argv, "gate_coord_finder_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();
    GateCoordFinder gate_coord_finder;
    gate_coord_finder.spin();
    return 0;
}