#include <vector>
#include <string>
#include <iostream>
#include "pipe_sim/PipeLine.h"
#include "pipe_sim/RFIDTag.h"
#include "pipe_sim/PipeRobot.h"
#include "pipe_sim/PipeRobots.h"
#include "pipe_segment.h"
#include "angles/angles.h"
#include "geometry_msgs/Pose2D.h"
#include "arc.h"
#include "line.h"
#include "boost/ptr_container/ptr_vector.hpp"

#ifndef PIPEMAP_H
#define PIPEMAP_H

/*PipeMaps are a series of connected Pipe Segments that simulate a PipeLine
 *Contain RFID Tags and used to simulate robot motion along a Pipeline */

class PipeMap
{
private:

    boost::ptr_vector<PipeSegment> pipe_line_;  //vector of PipeSegments
    double distance_;                           //distance of PipeMap in metres
    std::vector<pipe_sim::RFIDTag> rfid_tags_;  //RFID Tags located on PipeMap
    pipe_sim::PipeRobots robots_;               //Robots located on PipeMap

    //booleans for in class use
    bool map_set_;
    bool rfid_set_;

public:
    //Constructor Functions
    PipeMap();
    PipeMap(pipe_sim::PipeLine pipe_line, std::vector<pipe_sim::RFIDTag> rfid_tags);

    //Method to Add an RFID Tag
    bool addRFIDTag(pipe_sim::RFIDTag rfid_tag);
    //Method to check robot positions to be within bounds
    void processRobot(pipe_sim::PipeRobot &robot);
    //Method to update robots Pose by transforming from Distance Time to Pose Time Domain
    void setRobots(pipe_sim::PipeRobots robots);
    //Transform from Distance Time Domain to Pose Time Domain
    geometry_msgs::Pose2D getPoseDistance(double distance);

    //Getter Functions
    std::vector<pipe_sim::RFIDTag> getRFIDTags() const;
    pipe_sim::PipeRobots getRobots();
    double getMapDistance() const;

    //Bool Getter Functions
    bool mapSet() const;
    bool rfidSet() const;

    //Print Function
    void printMap();
};

#endif // PIPEMAP_H
