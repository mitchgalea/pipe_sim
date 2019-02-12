#include "pipe_sim/PipeRobots.h"
#include "rfid_reader/RSSIReading.h"
#include "rfid_reader/TagReading.h"
#include "pipe_sim/robot.h"
#include "pipe_sim/Point.h"

#include <cmath>
#include <chrono>

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

//Constant(s)
constexpr double SCALAR = 0.2;

/* ParticleFilter Class for Pipe Sim
 * Takes in vector of robots msg filters a sensor reading
 * Outputs a Resampled vector of robots msg */
class ParticleFilter
{
private:
    //static method that calculates the importance weight of a particle - in class use
    static void weight(pipe_sim::PipeRobot& robot, pipe_sim::Point sense_point);
public:
    //static method that filters particles based on a reading
    static pipe_sim::PipeRobots filterParticles(pipe_sim::PipeRobots &particles, pipe_sim::Point point);
};

#endif // PARTICLEFILTER_H
