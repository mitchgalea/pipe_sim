#include "pipe_segment.h"

#ifndef LINE_H
#define LINE_H

/*Line Pipe Segment Sub class for pipe segments that are straight Lines*/

class Line: public PipeSegment
{
private:

public:
    //Constructor functions
    Line(double length);
    Line(geometry_msgs::Pose2D starting_pose, double length, double starting_distance);

    //Gets Pose of point a given distance along line
    geometry_msgs::Pose2D getPose(geometry_msgs::Pose2D start_pose, double distance);

    //Gets the drawing instructions for the line
    pipe_sim::PipeDrawingInstructions getDrawingInstructions(geometry_msgs::Pose2D current);
    //Finds minimum and maximum height and width along the line
    geometry_msgs::Pose2D getMaxMinPoint(geometry_msgs::Pose2D current, bool max);

    //print function
    void printSegment();
};

#endif // LINE_H
