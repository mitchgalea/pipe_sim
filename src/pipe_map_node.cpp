#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "pipe_sim/PipeLine.h"
#include "pipe_sim/PipeRobots.h"
#include "pipe_sim/PipeLineSegment.h"
#include "pipe_sim/create_map.h"
#include "pipe_sim/RequestMapDistance.h"
#include "sensor_msgs/Image.h"
#include "pipe_sim/pipe_map.h"
#include "pipe_sim/RFIDTag.h"
#include "pipe_sim/RequestRFIDTags.h"
#include "pipe_sim/pipe_sim_drawer.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <unistd.h>

class PipeMapNode
{
private:

    ros::NodeHandle nh_;
    PipeMap* pipe_map_ptr_;
    CreateMap map_;
    cv_bridge::CvImage map_image_;
    PipeSimDrawer pipe_drawer_;
    ros::Subscriber robot_sub_;
    ros::ServiceServer rfid_map_service_;
    ros::ServiceServer distance_service_;
    ros::ServiceServer image_service_;
    ros::Subscriber rfid_tag_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

public:

    PipeMapNode(ros::NodeHandle nh)
        : nh_ (nh), it_(nh)
    {
        rfid_tag_sub_ = nh_.subscribe("rfid_tag", 10, &PipeMapNode::rfidTagCallback, this);
        distance_service_ = nh_.advertiseService("request_map_distance", &PipeMapNode::requestMapDistance, this );
        rfid_map_service_ = nh_.advertiseService("request_rfid_map", &PipeMapNode::requestRFIDMap, this );
        robot_sub_ = nh_.subscribe("pipe_robots", 10, &PipeMapNode::robotCallback, this);
        image_pub_ = it_.advertise("map_image", 1);

        std::string path;
        double offset;
        double resolution;
        double occupied_thresh;
        double free_thresh;

        ros::NodeHandle pn("~");
        pn.param<std::string>("name", path, "PipeMap");
        pn.param<double>("offset", offset, 1.0);
        pn.param<double>("resolution", resolution, 0.001);
        pn.param<double>("occupied_thresh", occupied_thresh, 0.6);
        pn.param<double>("free_thresh", free_thresh, 0.3);

        map_ = CreateMap(path, offset, resolution, occupied_thresh, free_thresh);
        if(map_.success())
        {
            pipe_drawer_ = PipeSimDrawer(map_.getMinPoint(), map_.getMaxPoint(),
                                         map_.getOffset(), resolution);
            map_.createImage(pipe_drawer_);
            ROS_INFO("Map Image Created: %s", map_.getMapName().c_str());

            map_image_.image = cv::imread(map_.getImagePath(), CV_LOAD_IMAGE_COLOR);
            map_image_.encoding = "bgr8";
            map_image_.header = std_msgs::Header();
        }

        pipe_map_ptr_ = new PipeMap(map_.getPipeLine(), map_.getRFIDTags());

        ROS_INFO("Pipe Map Created");

    }

    ~PipeMapNode()
    {
        delete pipe_map_ptr_;
    }

    bool requestMapDistance(pipe_sim::RequestMapDistance::Request &req,
                            pipe_sim::RequestMapDistance::Response &res)
    {
        if(req.req == true && pipe_map_ptr_->mapSet())
        {
            ROS_INFO("Map Distance Requested");
            res.ack = true;
            res.distance = pipe_map_ptr_->getMapDistance();
            ROS_INFO("Sending Map Distance");
        }
        else
        {
            res.ack = false;
        }
        return true;
    }

    bool requestRFIDMap(pipe_sim::RequestRFIDTags::Request &req,
                        pipe_sim::RequestRFIDTags::Response &res)
    {
        {
            if(req.req == true && pipe_map_ptr_->rfidSet())
            {
                ROS_INFO("RFID Map Requested");
                res.rfid_tags = pipe_map_ptr_ ->getRFIDTags();
                res.rssi_map_path.data = map_.getRSSIYamlPath();
                res.rfid_map_set = true;
                ROS_INFO("Sending RFID Map");
            }
            else
            {
                res.rfid_map_set = false;
            }
            return true;
        }
    }

    void robotCallback(const pipe_sim::PipeRobots& robots)
    {
        pipe_map_ptr_->setRobots(robots);
    }

    void rfidTagCallback(const pipe_sim::RFIDTag& rfid_tag)
    {
        if(pipe_map_ptr_->addRFIDTag(rfid_tag) && map_.addRFIDTagNode(rfid_tag))
        {
            ROS_INFO("RFID Tag Added");
        }
        else ROS_INFO("Tag Not Added");
    }

    void imageObjectThread()
    {
        ros::Rate rate_limiter(10);

        while(ros::ok())
        {
            cv_bridge::CvImage image;

            image.encoding = sensor_msgs::image_encodings::RGB8;
            image.image = map_image_.image.clone();

            pipe_drawer_.drawObjects(image.image, pipe_map_ptr_->getRFIDTags(), pipe_map_ptr_->getRobots());
            image_pub_.publish(image.toImageMsg());

            rate_limiter.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    chdir("/home/mitchell_galea");
    ros::init(argc, argv, "pipe_map_node");
    
    ros::NodeHandle nh;

    PipeMapNode pipe_map(nh);

    std::thread image_thread(&PipeMapNode::imageObjectThread, std::ref(pipe_map));

    ros::spin();

    ros::shutdown();

    image_thread.join();


  return 0;
}


