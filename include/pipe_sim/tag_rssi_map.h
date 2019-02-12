#include <list>
#include <iostream>
#include <vector>
#include <math.h>

#include "pipe_sim/point.h"
#include "pipe_sim/Point.h"

#ifndef TAGRSSIMAP_H
#define TAGRSSIMAP_H

//Constants
constexpr double EPSILON = 0.00001;

/* TagRSSIMap is a list of points that contain RSSI Readings
 * Class will check Points to see if a reading exists in the TagRSSIMap*/
class TagRSSIMap
{
private:
    long epc_;                                      //epc of RFID tag
    double interval_;                               //interval for each point
    std::vector<std::pair<double, double>> bounds_; //the bounds of the TagRSSIMap
    std::list<Point> points_;                       //list of Points

    //In Class Methods
    //Calculates the interval of points
    double calculateInterval();
    //calculates the Bounds of the TagRssiMap from points
    std::vector<std::pair<double, double>> calculateBounds();
public:
    //Constructs a TagRSSIMap from a List of Points
    TagRSSIMap(long epc, std::list<Point> points);
    //Constructs a TagRSSIMap using a function and variables
    TagRSSIMap(bool direction, double start_distance, double map_distance, double length, double interval,
               int32_t epc, double variance, double a, double b, double c, double d);

    //Getter Functions
    long getEPC() const;
    std::vector<std::pair<double, double>> getBounds();
    std::list<Point> getPoints() const;

    //Method to Check whether the Distance is within the TagRSSIMap
    bool checkDistance(double distance);
    //Gets a Point at a certain Distance in the TagRSSIMap
    pipe_sim::Point getPointDistance(double distance);
    //Finds all point of Intersections for an RSSI value
    std::vector<pipe_sim::Point> findIntersections(double rssi, double range);

    //Print Function
    void print();
};

#endif // TAGRSSIMAP_H
