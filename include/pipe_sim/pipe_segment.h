#include "angles/angles.h"
#include "pipe_sim/PipeDrawingInstructions.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

#ifndef PIPESEGMENT_H
#define PIPESEGMENT_H

/*Base Class used for segments of a Pipe Map - Robot motion model is convined to a series of segments (arcs/lines)
 * Robot Motion is in distance over time domain (where distance is the distance on the line)
 * Pipe Segment can transform from distance time domain to pose time domain */

class PipeSegment
{
protected:

    double length_;                     //length of pipe segment in metre
    double start_distance_;             //the distance along a series of pipe segments in which the pipe segment starts
    double end_distance_;               //the distance along a series of pipe segments in which the pipe engs
    geometry_msgs::Pose2D start_pose_;  //pose of start of the segment
    geometry_msgs::Pose2D end_pose_;    //pose of end of the segment
    bool set_;

public:
    //method to get the pose at a distance along the pipe segment
    virtual geometry_msgs::Pose2D getPose(geometry_msgs::Pose2D start_pose, double distance) = 0;

    //getter functions
    double getLength() const;
    double getStartDistance() const;
    double getEndDistance() const;
    geometry_msgs::Pose2D getStartPose() const;
    geometry_msgs::Pose2D getEndPose() const;

    //method gets instructions to draw pipe segment for opencv
    virtual pipe_sim::PipeDrawingInstructions getDrawingInstructions(geometry_msgs::Pose2D current) = 0;
    //method finds the Maximum and Minimum height and width points of the segment
    virtual geometry_msgs::Pose2D getMaxMinPoint(geometry_msgs::Pose2D current, bool max) = 0;

    //print function
    virtual void printSegment() = 0;
};

#endif // PIPESEGMENT_H
