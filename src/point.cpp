#include "pipe_sim/point.h"

Point::Point() {}
Point::Point(double rssi, double distance, double variance)
    :rssi_(rssi), distance_(distance), variance_(variance)
{}

double Point::getRSSI() const {return rssi_;}
double Point::getDistance() const {return distance_;}
double Point::getVariance() const {return variance_;}

void Point::print()
{
    std::cout << "{ RSSI: " << rssi_ << ", Distance: " << distance_ << ", Variance: " << variance_ << std::endl;
}

