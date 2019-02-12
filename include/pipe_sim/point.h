#include <iostream>

#ifndef POINT_H
#define POINT_H

/*Point Class is a class that holds an RSSI Reading*/

class Point
{
private:
    double rssi_;       //RSSI Reading in dBm
    double distance_;   //Distance along Pipeline in metres
    double variance_;   //Variance of Reading in metres
public:
    //Basic Constructor
    Point();
    //Full Constructor
    Point(double rssi, double distance, double variance);

    //Getter Methods
    double getRSSI() const;
    double getDistance() const;
    double getVariance() const;

    //Print Function
    void print();
};

#endif // POINT_H
