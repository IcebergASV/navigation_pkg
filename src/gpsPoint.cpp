#include <iostream>
#include <sstream>
#include <vector>
#include "gpsPoint.h"

// Default constructor
gpsPoint::gpsPoint() : latitude_(0.0), longitude_(0.0) {}

// Constructor with input parameters
gpsPoint::gpsPoint(double latitude, double longitude) : latitude_(latitude), longitude_(longitude) {}

// Getter for latitude
double gpsPoint::getLatitude() const {
    return latitude_;
}

// Getter for longitude
double gpsPoint::getLongitude() const {
    return longitude_;
}

// Setter for latitude
void gpsPoint::setLatitude(double latitude) {
    latitude_ = latitude;
}

// Setter for longitude
void gpsPoint::setLongitude(double longitude) {
    longitude_ = longitude;
}

// Overload << operator for gpsPoint objects
std::ostream& operator<<(std::ostream& os, const gpsPoint& point) {
    os << "(" << point.getLatitude() << ", " << point.getLongitude() << ")";
    return os;
}