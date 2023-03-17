#ifndef LIDARPOINT_H
#define LIDARPOINT_H

class LidarPoint {
public:
    // Constructors
    LidarPoint();
    LidarPoint(double distance, double angle);    

    // Getters
    double getDistance() const;
    double getAngle() const;

    // Setters
    void setDistance(double distance);
    void setAngle(double angle);

private:
    double distance_;
    double angle_;
};

//Operator Overloads
std::ostream& operator<<(std::ostream& os, const LidarPoint& point);

#endif // LIDARPOINT_H