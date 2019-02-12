#include "pipe_sim/pipe_segment.h"

double PipeSegment::getLength() const
{
    return length_;
}

double PipeSegment::getStartDistance() const
{
    return start_distance_;
}

double PipeSegment::getEndDistance() const
{
    return end_distance_;
}

geometry_msgs::Pose2D PipeSegment::getStartPose() const
{
    return start_pose_;
}

geometry_msgs::Pose2D PipeSegment::getEndPose() const
{
    return end_pose_;
}

