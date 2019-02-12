#include "pipe_segment.h"

#ifndef ARC_H
#define ARC_H

/*Arc Pipe Segment Sub class for pipe segments that are arcs
 *radius is a member */

class Arc: public PipeSegment
{
private:
    double radius_;         //radius of arc

    //private function for in class use
    static double getClosestAngle(double start, double end, double target);

public:
    //Constructor functions
    Arc(double length, double radius);
    Arc(geometry_msgs::Pose2D starting_pose, double length, double radius, double starting_distance);

    //Gets Pose of point a given distance along arc
    geometry_msgs::Pose2D getPose(geometry_msgs::Pose2D start_pose, double distance);
    //Gets the Pose of the Centre Point of Arc
    geometry_msgs::Pose2D getCentrePose(geometry_msgs::Pose2D start_pose);
    //Gets the drawing instructions for the arc
    pipe_sim::PipeDrawingInstructions getDrawingInstructions(geometry_msgs::Pose2D current);
    //Finds minimum and maximum height and width along the arc
    geometry_msgs::Pose2D getMaxMinPoint(geometry_msgs::Pose2D current, bool max);

    //getter function(s)
    double getRadius() const;

    //print function
    void printSegment();
};

#endif // ARC_H
