#include <iostream>
#include <sstream>
#include <vector>
#include "lidarPoint.h"

// Default constructor
LidarPoint::LidarPoint() : distance_(0.0), angle_(0.0) {}

// Constructor with input parameters
LidarPoint::LidarPoint(double distance, double angle) : distance_(distance), angle_(angle) {}

// Getter for distance
double LidarPoint::getDistance() const {
    return distance_;
}

// Getter for angle
double LidarPoint::getAngle() const {
    return angle_;
}

// Setter for distance
void LidarPoint::setDistance(double distance) {
    distance_ = distance;
}

// Setter for angle
void LidarPoint::setAngle(double angle) {
    angle_ = angle;
}

// Overload << operator for LidarPoint objects
std::ostream& operator<<(std::ostream& os, const LidarPoint& point) {
    os << "(" << point.getDistance() << ", " << point.getAngle() << ")";
    return os;
}