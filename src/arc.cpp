#include "pipe_sim/arc.h"

///////////////////////////////////////////////////////////////////////////////////////
////CONSTRUCTORS

Arc::Arc(double length, double radius)
{
    start_distance_ = 0;
    end_distance_ = start_distance_ + length;
    length_ = length;
    radius_ = radius;
    set_ = false;
}

Arc::Arc(geometry_msgs::Pose2D starting_pose, double length, double radius, double starting_distance)
{
    start_distance_ = 0;
    end_distance_ = start_distance_ + length;
    start_pose_ = starting_pose;
    length_ = length;
    radius_ = radius;
    end_pose_ = getPose(start_pose_, length_);
    start_distance_ = starting_distance;
    end_distance_ = start_distance_ + length;
    set_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////
////METHODS

geometry_msgs::Pose2D Arc::getCentrePose(geometry_msgs::Pose2D start_pose)
{
    geometry_msgs::Pose2D centre_pose;

    double beta = angles::normalize_angle_positive(start_pose.theta - (radius_/fabs(radius_)) * M_PI/2);

    centre_pose.x = start_pose.x + (fabs(radius_) * cos(beta));
    centre_pose.y = start_pose.y + (fabs(radius_) * sin(beta));

    return centre_pose;
}

geometry_msgs::Pose2D Arc::getPose(geometry_msgs::Pose2D start_pose, double distance)
{
    geometry_msgs::Pose2D pose;
    geometry_msgs::Pose2D centre_pose = getCentrePose(start_pose);

    double beta = angles::normalize_angle_positive(start_pose.theta - (radius_/fabs(radius_)) * M_PI/2);
    double alpha = angles::normalize_angle_positive(beta + (radius_/fabs(radius_)) * (M_PI - ((distance - start_distance_)/fabs(radius_))));

    pose.x = centre_pose.x + fabs(radius_) * cos(alpha);
    pose.y = centre_pose.y + fabs(radius_) * sin(alpha);
    pose.theta = angles::normalize_angle_positive(alpha - (radius_/fabs(radius_)) * (M_PI/2));
    return pose;
}

double Arc::getRadius() const
{
    return radius_;
}

pipe_sim::PipeDrawingInstructions Arc::getDrawingInstructions(geometry_msgs::Pose2D current)
{
    pipe_sim::PipeDrawingInstructions instructions;
    instructions.straight = false;
    instructions.centre = getCentrePose(current);
    instructions.radius = radius_;
    instructions.distance = length_;
    instructions.start = current;
    instructions.end = getPose(current, length_);
    instructions.angle = angles::normalize_angle_positive(length_/fabs(radius_));

    double beta = angles::normalize_angle_positive(current.theta - (radius_/fabs(radius_)) * M_PI/2);
    double theta = angles::normalize_angle_positive(beta - (radius_/fabs(radius_)) * M_PI);
    double alpha = angles::normalize_angle_positive(beta + (radius_/fabs(radius_)) * (M_PI - (length_/fabs(radius_))));

    if(radius_ < 0)instructions.rotate_angle = (2 * M_PI) - alpha;
    else instructions.rotate_angle = (2 * M_PI) - theta;

    return instructions;
}

geometry_msgs::Pose2D Arc::getMaxMinPoint(geometry_msgs::Pose2D current, bool max)
{
    geometry_msgs::Pose2D point;
    geometry_msgs::Pose2D centre_pose = getCentrePose(current);
    double beta = angles::normalize_angle_positive(current.theta - (radius_/fabs(radius_)) * M_PI/2);
    double theta = angles::normalize_angle_positive(beta - (radius_/fabs(radius_)) * M_PI);
    double alpha = angles::normalize_angle_positive(beta + (radius_/fabs(radius_)) * (M_PI - (length_/fabs(radius_))));

    if(radius_ < 0)
    {
        if(max)
        {
            point.x = centre_pose.x + fabs(radius_) * cos(getClosestAngle(theta, alpha, 0));
            point.y = centre_pose.y + fabs(radius_) * sin(getClosestAngle(theta, alpha, M_PI/2));
        }
        else
        {
            point.x = centre_pose.x + fabs(radius_) * cos(getClosestAngle(theta, alpha, M_PI));
            point.y = centre_pose.y + fabs(radius_) * sin(getClosestAngle(theta, alpha, (3*M_PI)/2));
        }

    }
    else
    {
        if(max)
        {
            point.x = centre_pose.x + fabs(radius_) * cos(getClosestAngle(alpha, theta, 0));
            point.y = centre_pose.y + fabs(radius_) * sin(getClosestAngle(alpha, theta, M_PI/2));
        }
        else
        {
            point.x = centre_pose.x + fabs(radius_) * cos(getClosestAngle(alpha, theta, M_PI));
            point.y = centre_pose.y + fabs(radius_) * sin(getClosestAngle(alpha, theta, (3*M_PI)/2));
        }
    }

    return point;
}

double Arc::getClosestAngle(double start, double end, double target)
{
    if(fabs(angles::shortest_angular_distance(target, start)) < fabs(angles::shortest_angular_distance(target, end)))
    {
        if(angles::shortest_angular_distance(target, start) < 0) return target;
        else return start;
    }
    else
    {
        if(angles::shortest_angular_distance(target, end) < 0) return end;
        else return target;
    }
}

void Arc::printSegment()
{
    std::cout << "ARC:" << std::endl;
    std::cout << "Start Pose: (x: " << start_pose_.x << ", y: " << start_pose_.y << ", theta: " << start_pose_.theta << ")" << std::endl;
    std::cout << "End Pose: (x: " << end_pose_.x << ", y: " << end_pose_.y << ", theta: " << end_pose_.theta << ")" << std::endl;
    std::cout << "Start Distance: " << start_distance_ << std::endl;
    std::cout << "End Distance: " << end_distance_ << std::endl;
    std::cout << "Length: " << length_ << std::endl;
    std::cout << "Radius: " << radius_ << std::endl;
}
