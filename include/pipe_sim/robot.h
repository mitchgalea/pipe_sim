#include "pipe_sim/PipeRobot.h"
#include "pipe_sim/RFIDTag.h"
#include "pipe_sim/PipeRobots.h"
#include "pipe_sim/Point.h"
#include "rssi_map.h"
#include "point.h"
#include "boost/array.hpp"
#include <random>
#include <chrono>
#include <vector>
#include <algorithm>
#include <queue>

#ifndef ROBOT_H
#define ROBOT_H

/*Robot Class*/
class Robot
{
private:
    pipe_sim::PipeRobot robot_;     //contains a robot ros msg from pipe_sim

public:
    //Basic Constructor
    Robot();
    //Constructors that use Variables
    Robot(double velocity, double move_noise, double sense_noise, double map_distance);
    Robot(double distance, bool direction, double start_noise,
          double velocity, double move_noise, double sense_noise, double map_distance);
    //Constructor from a Robot Msg
    Robot(pipe_sim::PipeRobot robot);

    //Method to Update Distance using the velocity
    void updateDistance(double rate, double map_distance);
    //Method to Updates Distance using encoder
    void updateDistance(bool encoder, double delta_distance, double map_distance);
    //Method for Particle Robots to Sense - updates robot sense with multiple points
    void sense(RSSIMap rssi_map, bool sense_noise, bool round_rssi);
    //Method for Simulated Robot to Sense - outputs a Single Point
    pipe_sim::Point simSense(RSSIMap rssi_map, bool sense_noise, bool round_rssi);

    //Setter Functions
    void setVelocity(double velocity);
    void setDistance(double distance);
    void setDirection(bool direction);
    void setPoints(std::vector<pipe_sim::Point> points);

    //static method to transform a vector of robots to a ros robots message
    static pipe_sim::PipeRobots getRobotsMsg(std::vector<Robot> robots);
    //static method to transform a robots_msg to a vector of robots
    static std::vector<Robot> getRobotVector(pipe_sim::PipeRobots robots_msg);
    //static method to transform vecotr of robots to a weight distance function
    static void robotstoCSV(pipe_sim::PipeRobots robots, std::__cxx11::string path);

    //Getter Function for robot msg
    pipe_sim::PipeRobot getRobotMsg();


};

#endif // ROBOT_H
