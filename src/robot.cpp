#include "pipe_sim/robot.h"

Robot::Robot()
{}

Robot::Robot(double velocity, double move_noise, double sense_noise, double map_distance)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> uniform_distribution(0.0, map_distance);

    robot_.distance = uniform_distribution(generator);
    if(uniform_distribution(generator) > (map_distance/2)) robot_.direction = 1;
    else robot_.direction = 0;
    robot_.velocity = velocity;
    robot_.move_noise = move_noise;
    robot_.sense_noise = sense_noise  ;
}


Robot::Robot(double distance, bool direction, double start_noise,
             double velocity, double move_noise, double sense_noise, double map_distance)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> normal_distribution(0.0, start_noise);
    robot_.distance = distance + normal_distribution(generator);
    if(robot_.distance > map_distance) robot_.distance = map_distance;
    if(robot_.distance < 0) robot_.distance = 0;

    robot_.direction = direction;
    robot_.velocity = velocity;
    robot_.move_noise = move_noise;
    robot_.sense_noise = sense_noise;
}

Robot::Robot(pipe_sim::PipeRobot robot)
{
    robot_ = robot;
}

void Robot::updateDistance(double rate, double map_distance)
{
    if(robot_.velocity != 0)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::normal_distribution<double> noise_distribution(0.0, (robot_.velocity * robot_.move_noise) / rate);
        robot_.distance += double(1 / rate) * ((int(robot_.direction) * robot_.velocity) +
                                              ((int(robot_.direction) - 1) * robot_.velocity));
        robot_.distance += noise_distribution(generator);

        if(robot_.distance > map_distance) robot_.distance = map_distance;
        if(robot_.distance < 0) robot_.distance = 0;
    }
}

void Robot::updateDistance(bool encoder, double delta_distance, double map_distance)
{
    if(fabs(delta_distance) > 0)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::normal_distribution<double> noise_distribution(0.0, delta_distance * robot_.move_noise);

        robot_.distance += ((int(robot_.direction) * delta_distance) +
                           ((int(robot_.direction) - 1) * delta_distance));
        robot_.distance += noise_distribution(generator);

        if(robot_.distance > map_distance) robot_.distance = map_distance;
        if(robot_.distance < 0) robot_.distance = 0;
    }
}

void Robot::sense(RSSIMap rssi_map, bool sense_noise, bool round_rssi)
{
    std::vector<pipe_sim::Point> points;
    if(robot_.direction) points = rssi_map.getPointsDistance(robot_.distance, RSSIMap::FORWARD_NAME);
    else points = points = rssi_map.getPointsDistance(robot_.distance, RSSIMap::REVERSE_NAME);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> noise_distribution(0.0, robot_.sense_noise);

    for(auto point_it = points.begin(); point_it < points.end(); point_it++)
    {
        if(sense_noise) point_it->rssi += noise_distribution(generator);
        if(round_rssi)  point_it->rssi = round(point_it->rssi);
    }

    setPoints(points);
}

pipe_sim::Point Robot::simSense(RSSIMap rssi_map, bool sense_noise, bool round_rssi)
{
    std::vector<pipe_sim::Point> points;
    if(robot_.direction) points = rssi_map.getPointsDistance(robot_.distance, RSSIMap::FORWARD_NAME);
    else points = points = rssi_map.getPointsDistance(robot_.distance, RSSIMap::REVERSE_NAME);

    pipe_sim::Point point;
    std::cout << "Sense Points Count: " << points.size() << std::endl;
    if(points.size() > 0)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::uniform_int_distribution<int> point_distribution(0, points.size() - 1);
        point = points[point_distribution(generator)];

        if(sense_noise)
        {
            std::normal_distribution<double> noise_distribution(0.0, robot_.sense_noise);
            point.rssi += noise_distribution(generator);
        }
        if(round_rssi)
        {
            point.rssi = round(point.rssi);
        }
        robot_.points.clear();
        robot_.points.push_back(point);
    }
    else point.epc = -1;
    return point;
}

void Robot::setVelocity(double velocity)
{
    robot_.velocity = velocity;
}

void Robot::setDirection(bool direction)
{
    robot_.direction = direction;
}

void Robot::setDistance(double distance)
{
    robot_.distance = distance;
}

void Robot::setPoints(std::vector<pipe_sim::Point> points)
{
    robot_.points = points;
}

pipe_sim::PipeRobots Robot::getRobotsMsg(std::vector<Robot> robots)
{
    pipe_sim::PipeRobots robots_msg;
    for(auto robot:robots)
    {
        robots_msg.robots.push_back(robot.robot_);
    }
    return robots_msg;
}

std::vector<Robot> Robot::getRobotVector(pipe_sim::PipeRobots robots_msg)
{
    std::vector<Robot> robots;
    for(auto robot_msg:robots_msg.robots)
    {
        Robot robot(robot_msg);
        robots.push_back(robot);
    }
    return robots;
}

pipe_sim::PipeRobot Robot::getRobotMsg()
{
    pipe_sim::PipeRobot robot_msg;
    robot_msg = robot_;

    return robot_msg;
}

void Robot::robotstoCSV(pipe_sim::PipeRobots robots, std::string path)
{
    double count = 0.0;

    for(auto robot_it = robots.robots.begin(); robot_it < robots.robots.end(); robot_it++)
    {
        count += robot_it->weight;
    }
    for(auto robot_it = robots.robots.begin(); robot_it < robots.robots.end(); robot_it++)
    {
        robot_it->weight = (robot_it->weight / count) * 100000;
    }

    std::ofstream myfile;
    myfile.open (path);

    const char delim = ',';
    const std::string newline = "\n";
    
    myfile << "distance" << delim << "weight" << newline;

    for(auto robot_it = robots.robots.begin(); robot_it < robots.robots.end(); robot_it++)
    {
        myfile << robot_it->distance << delim << robot_it->weight << newline;
    }
    myfile.close();
}



