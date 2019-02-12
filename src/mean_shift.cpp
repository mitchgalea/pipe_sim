#include "pipe_sim/mean_shift.h"

Robot MeanShift::findMeanRobot(std::vector<Robot> robots, double map_distance)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> uniform_distribution(0, robots.size() - 1);

    struct Mean{
        double mean;
        bool direction;
        int particles;
        bool start;
    };

    std::vector<Mean> means(MEAN_COUNT);

    for(int i = 0; i < means.size(); i++)
    {
        pipe_sim::PipeRobot msg = robots.at(uniform_distribution(generator)).getRobotMsg();
        means[i].mean = msg.distance;
        means[i].direction = msg.direction;
    }

    for(int i = 0; i < means.size(); i++)
    {
        double mean_distance = 0;
        int count = 0;

        for(auto robot:robots)
        {
            double robot_distance = robot.getRobotMsg().distance;
            if(robot_distance < (means[i].mean + (CLUSTER_SIZE / 2)) && robot_distance > (means[i].mean - (CLUSTER_SIZE / 2)) && robot.getRobotMsg().direction == means[i].direction)
            {
                mean_distance += robot_distance;
                count ++;
            }
        }

        mean_distance = mean_distance / double(count);



        if(fabs(mean_distance - means[i].mean) > CONVERGED_DISTANCE)
        {
            means[i].mean = mean_distance;
            means[i].particles = count;
        }
        else break;
    }

    int max = 0;

    for(int i = 0; i < means.size(); i++)
    {
        if(means[i].particles > means[max].particles)
        {
            max = i;
        }
    }

    Robot robot;
    robot.setDirection(means[max].direction);
    robot.setDistance(means[max].mean);

    return robot;
}

