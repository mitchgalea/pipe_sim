#include "pipe_sim/particle_filter.h"

pipe_sim::PipeRobots ParticleFilter::filterParticles(pipe_sim::PipeRobots &particles, pipe_sim::Point point)
{
    pipe_sim::PipeRobots filtered_particles;

    for(int i = 0; i < particles.robots.size(); i++)
    {
        weight(particles.robots[i], point);
    }

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> index_distribution(0, particles.robots.size());

    int index = index_distribution(generator);

    double beta = 0.0;

    double mw = 0;

    for(int i = 0; i < particles.robots.size(); i++)
    {
        mw = std::max(mw, particles.robots.at(i).weight);
    }

    std::uniform_real_distribution<double> beta_distribution(0.0, 2.0 * mw);

    for(int i = 0; i < particles.robots.size(); i++)
    {
        beta += beta_distribution(generator);
        while(beta > particles.robots.at(index).weight)
        {
            beta -= particles.robots.at(index).weight;
            index = (index + 1) % particles.robots.size();
        }

        filtered_particles.robots.push_back(particles.robots[index]);
    }
    
    return filtered_particles;
}

void ParticleFilter::weight(pipe_sim::PipeRobot& robot, pipe_sim::Point sense_point)
{
    robot.weight = 0.001;
    for(auto point:robot.points)
    {
        if(point.epc == sense_point.epc)
        {
            double difference = fabs(point.rssi - sense_point.rssi);
            if(difference < 0.1) difference = 0.1;
            robot.weight = exp(-1 * ( pow(difference * SCALAR, 2 )) /
                                (sqrt(2 * M_PI * pow( point.variance, 2))));
        }
    }
}


