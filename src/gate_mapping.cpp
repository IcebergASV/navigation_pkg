#include <ros/ros.h>
#include <navigation_pkg/Gate.h>
#include <navigation_pkg/GateArray.h>
#include <ros/console.h>

class GateMapping {
public:
    GateMapping()
    {
        prop_sub_ = nh_.subscribe("/completed_gates", 1, &PropMapping::propCallback, this);
        prop_pub_ = nh_.advertise<navigation_pkg::GateArray>("/gate_array", 1);
        
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    void propCallback(const navigation_pkg::Gate::ConstPtr& msg)
    {
        // get prop info
        navigation_pkg::Gate gate;
        gate.closer_marker.prop_type = msg->closer_marker.prop_type;
        gate.closer_marker.prop_coords.latitude = msg->closer_marker.prop_coords.latitude;
        gate.closer_marker.prop_coords.longitude = msg->closer_marker.prop_coords.longitude;
        gate.closer_marker.prop_coords.altitude = msg->closer_marker.prop_coords.altitude;

        gate.farther_marker.prop_type = msg->farther_marker.prop_type;
        gate.farther_marker.prop_coords.latitude = msg->farther_marker.prop_coords.latitude;
        gate.farther_marker.prop_coords.longitude = msg->farther_marker.prop_coords.longitude;
        gate.farther_marker.prop_coords.altitude = msg->farther_marker.prop_coords.altitude;

        gate.midpoint_coords.latitude = msg->midpoint_coords.latitude;
        gate.midpoint_coords.longitude = msg->midpoint_coords.longitude;
        gate.midpoint_coords.altitude = msg->midpoint_coords.altitude;

        gate.midpoint_range.min_latitude = msg->midpoint_range.min_latitude;
        gate.midpoint_range.max_latitude = msg->midpoint_range.max_latitude;
        gate.midpoint_range.min_longitude = msg->midpoint_range.min_longitude;
        gate.midpoint_range.max_longitude = msg->midpoint_range.max_longitude;
        gate.midpoint_range.min_altitude = msg->midpoint_range.min_altitude;
        gate.midpoint_range.max_altitude = msg->midpoint_range.max_altitugate;




        //make sure prop is not already in array
            
        if (isPropInArray(gate) == false){
            // add prop to array if not already there
            prop_array.props.push_back(gate);
            ROS_INFO_STREAM("New gate identified, adding to map");
        }


        //publish array       
        prop_pub_.publish(gate_array);
    }

    ros::NodeHandle nh_;
    ros::Subscriber prop_sub_;
    ros::Publisher prop_pub_;
    navigation_pkg::PropArray gate_array;

    bool isPropInArray(navigation_pkg::Gate gate){
        //use safety ranges to decide if prop is already in array
        for (int i = 0; i < gate_array.gates.size(); i++) {
            navigation_pkg::Gate checkprop = gate_array.gates[i];
            
            if ( gate.midpoint_coords.latitude <= checkprop.midpoint_range.max_latitude && gate.midpoint_coords.latitude >= checkprop.midpoint_range.min_latitude
            && gate.midpoint_coords.longitude <= checkprop.midpoint_range.max_longitude && gate.midpoint_coords.longitude >= checkprop.midpoint_range.min_longitude) {
                //prop is already in array, don't add it to the array
                return true;
                ROS_DEBUG_STREAM("Prop is already in the array");
            }

            
        }
        return false;       

    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gate_mapping_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();
    GateMapping gate_mapper;
    gate_mapper.spin();
    return 0;
}