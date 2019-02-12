#include "pipe_sim/pipe_map.h"
PipeMap::PipeMap()
    :map_set_ (false)
{

}

PipeMap::PipeMap(pipe_sim::PipeLine pipe_line, std::vector<pipe_sim::RFIDTag> rfid_tags)
    :distance_(0), map_set_(false), rfid_set_(false)
{
    geometry_msgs::Pose2D pose = pipe_line.initial_pose;
    for(auto pipe_segment:pipe_line.pipe_line)
    {
        if(pipe_segment.radius == 0)
        {
            pipe_line_.push_back(new Line(pose, pipe_segment.distance, distance_));
        }
        else
        {
            pipe_line_.push_back(new Arc(pose, pipe_segment.distance, pipe_segment.radius, distance_));
        }
        pose = pipe_line_.back().getEndPose();
        distance_ += pipe_line_.back().getLength();
    }
    map_set_ = true;

    for(auto rfid_tag:rfid_tags)
    {
        addRFIDTag(rfid_tag);
    }
    rfid_set_ = true;
    std::cout << std::endl;

}

void PipeMap::printMap()
{
    double count = 0;
    for(int i = 0; i < pipe_line_.size(); i++)
    {
        std::cout << "SEGMENT " << count << std::endl;
        pipe_line_.at(i).printSegment();
        std::cout << std::endl;
        count ++;
    }
}

geometry_msgs::Pose2D PipeMap::getPoseDistance(double distance)
{
    if(distance <= distance_ && distance >= 0)
    {
        for(int i = 0; i < pipe_line_.size(); i++)
        {
            if(distance >= pipe_line_.at(i).getStartDistance() && distance <= pipe_line_.at(i).getEndDistance())
            {
                return pipe_line_.at(i).getPose(pipe_line_.at(i).getStartPose(), distance);
            }
        }
    }
}

void PipeMap::processRobot(pipe_sim::PipeRobot &robot)
{
    if(robot.distance > distance_) robot.distance = distance_;
    if(robot.distance < 0) robot.distance = 0;
}

pipe_sim::PipeRobots PipeMap::getRobots()
{ 
    if(robots_.simulated_robot_set)
    {
        robots_.simulated_robot.pose = getPoseDistance(robots_.simulated_robot.distance);
    }
    robots_.predicted_robot.pose = getPoseDistance(robots_.predicted_robot.distance);

    return robots_;
}

bool PipeMap::addRFIDTag(pipe_sim::RFIDTag rfid_tag)
{
    if(rfid_tag.distance <= distance_)
    {
       rfid_tag.pose = getPoseDistance(rfid_tag.distance);
       rfid_tags_.push_back(rfid_tag);
       return true;
    }
    else return false;
}

double PipeMap::getMapDistance() const
{
    return distance_;
}

std::vector<pipe_sim::RFIDTag> PipeMap::getRFIDTags() const
{
    return rfid_tags_;
}

bool PipeMap::mapSet() const
{
    return map_set_;
}

bool PipeMap::rfidSet() const
{
    return rfid_set_;
}

void PipeMap::setRobots(pipe_sim::PipeRobots robots)
{
    robots_ = robots;

    for(int i = 0; i < robots_.robots.size(); i ++)
    {
        robots_.robots.at(i).pose = getPoseDistance(robots_.robots.at(i).distance);
    }
}
