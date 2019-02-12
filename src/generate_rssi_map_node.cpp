#include "ros/ros.h"
#include "std_msgs/String.h"

#include "pipe_sim/rssi_map.h"

#include <sstream>
#include <iostream>
#include <string>
#include <chrono>
#include <random>
#include <unistd.h>

constexpr double MAP_DISTANCE = 40.0;
constexpr double TAG_SPACING = 2.0;
constexpr double RSSI_LENGTH = 4.0;
constexpr double INTERVAL = 0.1;
constexpr double VARIANCE = 2.0;

constexpr double CONSTANT_A = -30;
constexpr double CONSTANT_B = -0.5;
constexpr double CONSTANT_C = 5.0;
constexpr double CONSTANT_D = 2.0;

const std::string YAML_PATH = "catkin_ws/src/pipe_sim/maps/TestMap/TestMap@.yaml";
const std::string CSV_PATH = "catkin_ws/src/pipe_sim/maps/TestMap/TestMap.csv";

class GenerateRSSIMapNode
{
private:
    ros::NodeHandle nh_;
    
    RSSIMap rssi_map_;
    std::vector<pipe_sim::RFIDTag> rfid_tags_;


public:
    GenerateRSSIMapNode(ros::NodeHandle nh)
        :nh_(nh)
    {
        setRFIDTags(makeRFIDTags());
        rssi_map_ = RSSIMap(rfid_tags_, MAP_DISTANCE, RSSI_LENGTH, INTERVAL, VARIANCE, CONSTANT_A, CONSTANT_B, CONSTANT_C, CONSTANT_D);
        rssi_map_.emitYAML(YAML_PATH);
        rssi_map_.creatCSV(CSV_PATH);

    }

    void setRFIDTags(std::vector<pipe_sim::RFIDTag> rfid_tags)
    {
        rfid_tags_ = rfid_tags;
    }
    std::vector<pipe_sim::RFIDTag> makeRFIDTags()
    {
        std::vector<pipe_sim::RFIDTag> rfid_tags;
        for(unsigned distance = 0; distance <= MAP_DISTANCE; distance += TAG_SPACING)
        {
            rfid_tags.push_back(makeRFIDTag(distance));
        }
        return rfid_tags;
    }
    pipe_sim::RFIDTag makeRFIDTag(double distance)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::uniform_int_distribution<int32_t> point_distribution(0, std::numeric_limits<int32_t>::max());

        pipe_sim::RFIDTag rfid_tag;
        rfid_tag.distance = distance;
        rfid_tag.epc = point_distribution(generator);

        return rfid_tag;
    }
    void printRFIDTags()
    {
        std::cout << "RFID Tags:" << std::endl;
        for(auto rfid_tag:rfid_tags_)
        {
            std::cout << "     - {EPC: " << rfid_tag.epc << ", Distance: " << rfid_tag.distance << "}" << std::endl;
        }
    }

    ~GenerateRSSIMapNode()
    {}

};

int main(int argc, char **argv)
{
    chdir("/home/mitchell_galea");
    ros::init(argc, argv, "generate_rssi_map_node");
    
    ros::NodeHandle nh;

    GenerateRSSIMapNode generate_rssi_map_node(nh);

    ros::spinOnce();

    ros::shutdown();

  return 0;
}


