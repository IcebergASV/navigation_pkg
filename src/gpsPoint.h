#ifndef GPSPOINT_H
#define GPSPOINT_H

class gpsPoint {
public:
    // Constructors
    gpsPoint();
    gpsPoint(double latitude, double longitude);    

    // Getters
    double getDistance() const;
    double getAngle() const;

    // Setters
    void setDistance(double latitude);
    void setAngle(double longitude);

private:
    double latitude_;
    double longitude_;
};

//Operator Overloads
std::ostream& operator<<(std::ostream& os, const gpsPoint& point);

#endif // LIDARPOINT_H