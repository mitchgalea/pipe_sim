#include "pipe_sim/robot.h"
#include "pipe_sim/PipeRobot.h"

#include <random>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iostream>

#ifndef MEANSHIFT_H
#define MEANSHIFT_H

//Constants
constexpr double CLUSTER_SIZE = 1.0;
const int MEAN_COUNT = 3;
constexpr double CONVERGED_DISTANCE = 0.01;

/*MeanShift Class to calculate the predicted position of multiple clusters of particles */
class MeanShift
{
public:
    //static Method to find Mean Shift
    static Robot findMeanRobot(std::vector<Robot> robots, double map_distance);
};

#endif // MEANSHIFT_H
