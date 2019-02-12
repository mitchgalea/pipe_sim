#include "pipe_sim/rssi_map.h"

RSSIMap::RSSIMap() {}

const std::string RSSIMap::FORWARD_NAME = "ForwardTagRSSIMaps";
const std::string RSSIMap::REVERSE_NAME = "ReverseTagRSSIMaps";
const std::string RSSIMap::NAME = "TagRSSIMaps";

RSSIMap::RSSIMap(std::__cxx11::string file_path)
{
    std::cout << file_path << std::endl;
    YAML::Node node = YAML::LoadFile(file_path);

    if(node["Name"])
    {
        YAML::Node name = node["Name"];
        setName(name.as<std::string>());
    }

    for(auto name:NAMES)
        if(node[name])
        {
            YAML::Node tag_rssi_maps_node = node[name];
            std::vector<TagRSSIMap> tag_rssi_maps;
            for(auto i = 0; i < tag_rssi_maps_node.size(); i++)
            {
                std::list<Point> points;
                if(tag_rssi_maps_node[i]["Points"])
                {
                    YAML::Node points_node = tag_rssi_maps_node[i]["Points"];
                    for(auto i = 0; i < points_node.size(); i++)
                    {
                        YAML::Node point_node = points_node[i];
                        if(point_node["RSSI"] && point_node["Distance"] && point_node["Variance"])
                        {
                            Point point(point_node["RSSI"].as<double>(), point_node["Distance"].as<double>(), point_node["Variance"].as<double>());
                            points.push_back(point);
                        }
                    }
                }
                long epc;
                if(tag_rssi_maps_node[i]["EPC"])
                {
                    epc = tag_rssi_maps_node[i]["EPC"].as<long>();
                }
                TagRSSIMap tag_rssi_map(epc, points);
                tag_rssi_maps.push_back(tag_rssi_map);
            }
            addTagRSSIMaps(tag_rssi_maps, name);
        }
}

RSSIMap::RSSIMap(std::vector<pipe_sim::RFIDTag> rfid_tags, double map_distance, double length,
                 double interval, double variance, double a, double b, double c, double d, std::string name)
    :name_(name)
{
    std::vector<TagRSSIMap> forward_maps;
    std::vector<TagRSSIMap> reverse_maps;
    for(auto rfid_tag:rfid_tags)
    {
        forward_maps.push_back(TagRSSIMap(true, rfid_tag.distance, map_distance, length, interval, rfid_tag.epc, variance, a, b, c, d));
        reverse_maps.push_back(TagRSSIMap(false, rfid_tag.distance, map_distance, length, interval, rfid_tag.epc, variance, a, b, c, d));
    }
    addTagRSSIMaps(forward_maps, FORWARD_NAME);
    addTagRSSIMaps(reverse_maps, REVERSE_NAME);
}

void RSSIMap::setName(std::__cxx11::string name)
{
    name_ = name;
}

TagRSSIMap RSSIMap::getTagRssiMap(long epc, std::string name)
{
    if(maps_.find(name) != maps_.end())
    {
        for(auto tag_rssi_map:maps_[name])
        {
            if(tag_rssi_map.getEPC() == epc) return tag_rssi_map;
        }
    }
}

bool RSSIMap::checkDistance(double distance, std::__cxx11::string name)
{
    if(maps_.find(name) != maps_.end())
    {
        for(auto tag_rssi_map:maps_[name])
        {
            if(tag_rssi_map.checkDistance(distance)) return true;
        }
    }
    return false;
}

std::vector<pipe_sim::Point> RSSIMap::getPointsDistance(double distance, std::__cxx11::string name)
{
    std::vector<pipe_sim::Point> points;
    if(maps_.find(name) != maps_.end())
    {
        for(auto tag_rssi_map:maps_[name])
        {
            if(tag_rssi_map.checkDistance(distance))
            {
                points.push_back(tag_rssi_map.getPointDistance(distance));
            }
        }
    }
    return points;
}


void RSSIMap::addTagRSSIMaps(std::vector<TagRSSIMap> tag_rssi_maps, std::__cxx11::string name)
{
    maps_[name] = tag_rssi_maps;
}

std::vector<pipe_sim::Point> RSSIMap::getInterceptions(long epc, double rssi, double range, std::__cxx11::string name)
{
    if(maps_.find(name) != maps_.end())
    {
        return getTagRssiMap(epc, name).findIntersections(rssi, range);
    }
}



void RSSIMap::print()
{
    std::cout << "Name: " << name_ << std::endl;
    for(auto tag_rssi_map:maps_)
    {
        std::cout << tag_rssi_map.first << "size: " << tag_rssi_map.second.size() << std::endl;
        for(unsigned i = 0; i < tag_rssi_map.second.size(); i++)
        {
             tag_rssi_map.second.at(i).print();
        }
    }

}

void RSSIMap::emitYAML(std::string path)
{
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;

    emitter << YAML::Key << "Name" << YAML::Value << name_;

    for(auto map_it = maps_.begin(); map_it != maps_.end(); map_it++)
    {
        emitter << YAML::Key << map_it->first;
        emitter << YAML::Value << YAML::BeginSeq;
        for(auto tag_rssi_map:map_it->second)
        {
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "EPC" << YAML::Value << tag_rssi_map.getEPC();
            emitter << YAML::Key << "Points";
            emitter << YAML::Value << YAML::BeginSeq;
            std::list<Point> points = tag_rssi_map.getPoints();
            for(auto point_it = points.begin(); point_it != points.end(); point_it++)
            {
                emitter << YAML::BeginMap;
                emitter << YAML::Key << "RSSI" << YAML::Value << std::to_string(point_it->getRSSI());
                emitter << YAML::Key << "Distance" << YAML::Value << std::to_string(point_it->getDistance());
                emitter << YAML::Key << "Variance" << YAML::Value << std::to_string(point_it->getVariance());
                emitter << YAML::EndMap;
            }
            emitter << YAML::EndSeq;
            emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;

    }
    emitter << YAML::EndSeq;

    std::ofstream fout(path);
    fout << emitter.c_str();
}

void RSSIMap::creatCSV(std::__cxx11::string path)
{
    std::ofstream myfile;
    myfile.open (path);

    const char delim = ',';
    const std::string newline = "\n";

    for(auto map_it = maps_.begin(); map_it != maps_.end(); map_it++)
    {
        for(auto tag_rssi_map:map_it->second)
        {
            std::list<Point> points = tag_rssi_map.getPoints();
            for(auto point_it = points.begin(); point_it != points.end(); point_it++)
            {
                bool direction;
                if(map_it->first == FORWARD_NAME)direction = true;
                if(map_it->first == REVERSE_NAME)direction = false;
                myfile << direction << delim << point_it->getDistance() << delim << point_it->getRSSI() << delim << tag_rssi_map.getEPC() << newline;
            }
        }
    }
    myfile.close();
}

