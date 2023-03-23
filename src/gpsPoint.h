#ifndef GPSPOINT_H
#define GPSPOINT_H

class gpsPoint {
public:
    // Constructors
    gpsPoint();
    gpsPoint(double latitude, double longitude);    

    // Getters
    double getLatitude() const;
    double getLongitude() const;

    // Setters
    void setLatitude(double latitude);
    void setLongitude(double longitude);

private:
    double latitude_;
    double longitude_;
};

//Operator Overloads
std::ostream& operator<<(std::ostream& os, const gpsPoint& point);

#endif // LIDARPOINT_H