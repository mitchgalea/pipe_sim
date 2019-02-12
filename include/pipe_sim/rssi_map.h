#include <string>
#include "pipe_sim/tag_rssi_map.h"
#include "pipe_sim/Point.h"
#include "pipe_sim/RFIDTag.h"
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#ifndef RSSIMAP_H
#define RSSIMAP_H

//Names of Possible Vectors of Tag RSSI Maps - Usually Forward and Reverse Used
const std::vector<std::string> NAMES = {"TagRSSIMaps", "ForwardTagRSSIMaps", "ReverseTagRSSIMaps"};

/*RSSI Map is a collection of Tag RSSI Maps that maps the RSSI signals of a PipeLine*/
class RSSIMap
{
private:
    std::string name_;                                      //name of RSSI Map
    std::map<std::string, std::vector<TagRSSIMap>> maps_;   //Tag RSSI Maps

public:
    //Basic Constructor
    RSSIMap();
    //Constructor that uses a Yaml File
    RSSIMap(std::string file_path);
    //Constructor that uses RFID Tags and Variables
    RSSIMap(std::vector<pipe_sim::RFIDTag> rfid_tags, double map_distance, double length,
            double interval, double variance, double a, double b, double c, double d, std::__cxx11::string name = "map");

    //Static Const Members for public access
    static const std::string FORWARD_NAME;
    static const std::string REVERSE_NAME;
    static const std::string NAME;

    //checks whether a Point is at a distance within the RSSI Map
    bool checkDistance(double distance, std::__cxx11::string name);
    //sets Name
    void setName(std::string name);
    //Adds a vector of Tag RSSI Map
    void addTagRSSIMaps(std::vector<TagRSSIMap> tag_rssi_maps, std::__cxx11::string name);
    //Finds all Interception points in a TagRSSIMap
    std::vector<pipe_sim::Point> getInterceptions(long epc, double rssi, double range, std::__cxx11::string name);
    //Gets all Points at a certain distance of RSSI Map
    std::vector<pipe_sim::Point> getPointsDistance(double distance, std::__cxx11::string name);
    //Gets a Tag RSSI Map
    TagRSSIMap getTagRssiMap(long epc, std::__cxx11::string name);

    //Emits RSSI Map as Yaml
    void emitYAML(std::__cxx11::string path);
    //Creates a CSV for RSSI Map
    void creatCSV(std::string path);

    //print method
    void print();
};

#endif // RSSIMAP_H
