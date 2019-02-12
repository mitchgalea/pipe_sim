#include "pipe_sim/line.h"

///////////////////////////////////////////////////////////////////////////////////////
////CONSTRUCTORS

Line::Line(double length)
{
    start_distance_ = 0;
    end_distance_ = start_distance_ + length;
    length_ = length;
    set_ = false;
}

Line::Line(geometry_msgs::Pose2D starting_pose, double length, double starting_distance)
{
    start_pose_ = starting_pose;
    length_ = length;
    end_pose_ = getPose(start_pose_, length_);
    start_distance_ = starting_distance;
    end_distance_ = start_distance_ + length;
    set_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////
////METHODS

geometry_msgs::Pose2D Line::getPose(geometry_msgs::Pose2D start_pose, double distance)
{
    geometry_msgs::Pose2D pose;
    //finds pose of distance along pipe line
    pose.x = start_pose.x + (distance - start_distance_) * cos(start_pose.theta);
    pose.y = start_pose.y + (distance - start_distance_) * sin(start_pose.theta);
    pose.theta = start_pose.theta;

    return pose;
}

pipe_sim::PipeDrawingInstructions Line::getDrawingInstructions(geometry_msgs::Pose2D current)
{
    //sets instructions
    pipe_sim::PipeDrawingInstructions instructions;
    instructions.straight = true;
    instructions.start = current;
    instructions.end = getPose(current, length_);
    instructions.distance = length_;

    return instructions;
}

geometry_msgs::Pose2D Line::getMaxMinPoint(geometry_msgs::Pose2D current, bool max)
{
    //returns either max or min point of line
    geometry_msgs::Pose2D point;
    geometry_msgs::Pose2D end = getPose(current, length_);

    if(max)
    {
        point.x = std::max(end.x, current.x);
        point.y = std::max(end.y, current.y);
    }
    else
    {
        point.x = std::min(end.x, current.x);
        point.y = std::min(end.y, current.y);
    }

    return point;
}

void Line::printSegment()
{
    std::cout << "LINE:" << std::endl;
    std::cout << "Start Pose: (x: " << start_pose_.x << ", y: " << start_pose_.y << ", theta: " << start_pose_.theta << ")" << std::endl;
    std::cout << "End Pose: (x: " << end_pose_.x << ", y: " << end_pose_.y << ", theta: " << end_pose_.theta << ")" << std::endl;
    std::cout << "Start Distance: " << start_distance_ << std::endl;
    std::cout << "End Distance: " << end_distance_ << std::endl;
    std::cout << "Length: " << length_ << std::endl;
}




