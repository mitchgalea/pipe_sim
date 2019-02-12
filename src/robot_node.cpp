#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "pipe_sim/PipeLine.h"
#include "pipe_sim/PipeLineSegment.h"
#include "pipe_sim/create_map.h"
#include "pipe_sim/RequestMapDistance.h"
#include "sensor_msgs/Image.h"
#include "pipe_sim/pipe_map.h"
#include "pipe_sim/PipeRobots.h"
#include "pipe_sim/PipeRobot.h"
#include "pipe_sim/SetRobotVelocity.h"
#include "pipe_sim/RequestSense.h"
#include "pipe_sim/robot.h"
#include "pipe_sim/RequestRFIDTags.h"
#include "pipe_sim/particle_filter.h"
#include "pipe_sim/mean_shift.h"
#include "rfid_reader/TagReading.h"
#include "rfid_reader/RSSIReading.h"
#include "pipe_sim/rssi_map.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <random>
#include <unistd.h>

const int ENCODER_BUFFER = 10;
constexpr double ENCODER_MIN_DISTANCE = 0.001;
constexpr double ENCODER_CIRC = 1.0;
const int ENCODER_REVOLUTION = 2;

class RobotNode
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient map_distance_client_;
    ros::Publisher robot_pub_;
    ros::ServiceServer velocity_service_;
    ros::ServiceServer sense_service_;
    ros::Subscriber rfid_reading_sub_;
    ros::Subscriber encoder_sub_;
    ros::ServiceClient rfid_map_client_;

    double map_distance_;
    std::vector<Robot> robots_;
    std::mutex robot_mutex_;
    Robot simulated_robot_;
    bool set_simulated_robot_;
    RSSIMap rssi_map_;
    std::vector<pipe_sim::RFIDTag> rfid_tags_;
    double refresh_rate_;
    double sense_rate_;
    double sense_range_;
    bool encoder_;
    int32_t encoder_ticks_;

public:
    RobotNode(ros::NodeHandle nh)
        :nh_(nh), encoder_ticks_(0)
    {
        map_distance_client_ = nh_.serviceClient<pipe_sim::RequestMapDistance>("request_map_distance");
        rfid_map_client_ = nh_.serviceClient<pipe_sim::RequestRFIDTags>("request_rfid_map");
        robot_pub_ = nh_.advertise<pipe_sim::PipeRobots>("pipe_robots", 5);
        velocity_service_ = nh_.advertiseService("set_robot_velocity", &RobotNode::setRobotVelocity, this);
        sense_service_ = nh_.advertiseService("request_sense", &RobotNode::requestSense, this);
        rfid_reading_sub_ = nh_.subscribe("rfid_reading", 10, &RobotNode::rfidReadingCallback, this);
        encoder_sub_ = nh_.subscribe("encoder", 10, &RobotNode::encoderCallback, this);


        pipe_sim::RequestMapDistance map_distance_srv;
        map_distance_srv.request.req = true;

        if(map_distance_client_.call(map_distance_srv))
        {
            if(map_distance_srv.response.ack)
            {
                ROS_INFO("Map Distance Received");
                map_distance_ = map_distance_srv.response.distance;
            }
            else ROS_INFO("Map Not Set");
        }
        else
        {
            ROS_INFO("Map Distance Service Failed");
        }

        pipe_sim::RequestRFIDTags rfid_map_srv;
        rfid_map_srv.request.req = true;

        if(rfid_map_client_.call(rfid_map_srv))
        {
            if(rfid_map_srv.response.rfid_map_set)
            {
                ROS_INFO("RSSI Map Recieved");
                rssi_map_ = RSSIMap(rfid_map_srv.response.rssi_map_path.data);
                rssi_map_.print();
                rfid_tags_ = rfid_map_srv.response.rfid_tags;
            }
            else ROS_INFO("RFID Map Not Set");
        }
        else
        {
            ROS_INFO("RFID Map Service Failed");
        }
        
        int count;
        double distance;
        double velocity;
        bool direction;
        double start_noise;
        double move_noise;
        double sense_noise;

        ros::NodeHandle pn("~");

        pn.param<int>("count", count, 1000);
        pn.param<double>("distance", distance, -1.0);
        pn.param<double>("velocity", velocity, 0.0);
        pn.param<bool>("direction", direction, true);
        pn.param<double>("start_noise", start_noise, 0.0);
        pn.param<double>("move_noise", move_noise, 0.0);
        pn.param<double>("sense_noise", sense_noise, 0.1);
        pn.param<double>("refresh_rate", refresh_rate_, 10.0);
        pn.param<double>("sense_rate", sense_rate_, 0.5);
        pn.param<double>("sense_range", sense_range_, 1.0);
        pn.param<bool>("simulated_robot", set_simulated_robot_, false);

        if(velocity > 0) encoder_ = false;
        else encoder_ = true;

        if(distance < 0)  
        {
            if(set_simulated_robot_) simulated_robot_ = Robot(velocity, move_noise, sense_noise, map_distance_);

            for(int i = 0; i < count; i++)
            {
                Robot robot(velocity, move_noise, 2 * sense_noise, map_distance_);
                robots_.push_back(robot);

            }
        }
        else
        {
            if(set_simulated_robot_) simulated_robot_ = Robot(distance, direction, 0.0, velocity, move_noise, sense_noise, map_distance_);

            for(int i = 0; i < count; i++)
            {
                Robot robot(distance, direction, start_noise, velocity, move_noise, 2 * sense_noise, map_distance_);
                robots_.push_back(robot);
            }
        }
        ROS_INFO("Robots Created");
    }

    ~RobotNode()
    {}

    bool setRobotVelocity(pipe_sim::SetRobotVelocity::Request &req,
                          pipe_sim::SetRobotVelocity::Response &res)
    {
        if(encoder_) encoder_ = false;
        ROS_INFO("Velocity Set: %f", req.velocity);

        if(set_simulated_robot_) simulated_robot_.setVelocity(req.velocity);


        for(int i = 0; i < robots_.size(); i++)
        {
            robots_.at(i).setVelocity(req.velocity);
        }
        res.ack = true;
        return true;
    }

    bool requestSense(pipe_sim::RequestSense::Request &req,
                          pipe_sim::RequestSense::Response &res)
    {
        pipe_sim::Point sense_point = simulated_robot_.simSense(rssi_map_, true, true);
        std::cout << "EPC: " << sense_point.epc << ", RSSI: " << sense_point.rssi << " , Distance: " << sense_point.distance << std::endl;
        if(sense_point.epc != -1)
        {
            robot_mutex_.lock();
            for(int i = 0; i < robots_.size(); i++) robots_.at(i).sense(rssi_map_, false, true);
            pipe_sim::PipeRobots robots_msg = Robot::getRobotsMsg(robots_);
            robots_ = Robot::getRobotVector(ParticleFilter::filterParticles(robots_msg, sense_point));
            ROS_INFO("Robot Particles Filtered");
            robot_mutex_.unlock();
        }
        return true;
    }

    void rfidReadingCallback(const rfid_reader::RSSIReading& rfid_reading)
    {
        pipe_sim::Point sense_point;
        sense_point.epc = rfid_reading.epc;
        sense_point.rssi = rfid_reading.rssi;

        robot_mutex_.lock();
        for(int i = 0; i < robots_.size(); i++) robots_.at(i).sense(rssi_map_, false, true);
        pipe_sim::PipeRobots robots_msg = Robot::getRobotsMsg(robots_);
        robots_ = Robot::getRobotVector(ParticleFilter::filterParticles(robots_msg, sense_point));
        ROS_INFO("Robot Particles Filtered");
        robot_mutex_.unlock();

    }

    void encoderCallback(const std_msgs::Int32& encoder_reading)
    {
        int32_t current_encoder = encoder_reading.data;

        if((current_encoder - encoder_ticks_) >= ((ENCODER_MIN_DISTANCE * ENCODER_REVOLUTION) / ENCODER_CIRC))
        {

            double delta_distance = (current_encoder - encoder_ticks_) * (ENCODER_CIRC / ENCODER_REVOLUTION);
            encoder_ticks_ = current_encoder;

            robot_mutex_.lock();
            for(int i = 0; i < robots_.size(); i++)
            {
                robots_.at(i).updateDistance(true, delta_distance, map_distance_);
            }
            if(set_simulated_robot_)
            {
                simulated_robot_.updateDistance(true, delta_distance, map_distance_);
            }
            robot_mutex_.unlock();
        }


    }

    void robotDistanceThread()
    {
        ros::Rate rate_limiter(refresh_rate_);
        while(ros::ok())
        {
            if(!encoder_)
            {
                robot_mutex_.lock();
                for(int i = 0; i < robots_.size(); i++)
                {
                    robots_.at(i).updateDistance(refresh_rate_, map_distance_);
                }
                if(set_simulated_robot_)
                {
                    simulated_robot_.updateDistance(refresh_rate_, map_distance_);
                }
                robot_mutex_.unlock();
            }

            Robot predicted_robot;
            predicted_robot = MeanShift::findMeanRobot(robots_, map_distance_);

            pipe_sim::PipeRobots robots_msg = Robot::getRobotsMsg(robots_);
            robots_msg.predicted_robot = predicted_robot.getRobotMsg();
            robots_msg.simulated_robot_set = false;
            if(set_simulated_robot_)
            {
                robots_msg.simulated_robot = simulated_robot_.getRobotMsg();
                robots_msg.simulated_robot_set = true;
            }
            robot_pub_.publish(robots_msg);

            rate_limiter.sleep();
        }
    }
    
    void simSenseThread()
    {
        ros::Rate rate_limiter(sense_rate_);
        rate_limiter.sleep();
        while(ros::ok())
        {
            pipe_sim::Point sense_point = simulated_robot_.simSense(rssi_map_, true, true);
            std::cout << "EPC: " << sense_point.epc << ", RSSI: " << sense_point.rssi << " , Distance: " << sense_point.distance << std::endl;
            if(sense_point.epc != -1)
            {
                robot_mutex_.lock();
                for(int i = 0; i < robots_.size(); i++) robots_.at(i).sense(rssi_map_, false, true);
                pipe_sim::PipeRobots robots_msg = Robot::getRobotsMsg(robots_);
                robots_ = Robot::getRobotVector(ParticleFilter::filterParticles(robots_msg, sense_point));
                ROS_INFO("Robot Particles Filtered");
                robot_mutex_.unlock();
            }
            rate_limiter.sleep();
        }
    }
    
    bool simulated_robot() const
    {
        return set_simulated_robot_;
    }
};

int main(int argc, char **argv)
{
    chdir("/home/mitchell_galea");
    ros::init(argc, argv, "pipe_map_node");

    ros::init(argc, argv, "robot_node");
    
    ros::NodeHandle nh;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    RobotNode robot_node(nh);

    std::thread distance_thread(&RobotNode::robotDistanceThread, std::ref(robot_node));

    std::thread sim_sense_thread;

    if(robot_node.simulated_robot()) sim_sense_thread = std::thread(&RobotNode::simSenseThread, std::ref(robot_node));

    ros::spin();

    ros::shutdown();

    distance_thread.join();

    sim_sense_thread.join();


  return 0;
}


