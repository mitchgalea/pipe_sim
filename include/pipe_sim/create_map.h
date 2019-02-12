#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <math.h>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include <boost/algorithm/string.hpp>
#include "pipe_sim/PipeDrawingInstructions.h"
#include "pipe_sim/PipeLine.h"
#include "pipe_sim/PipeLineSegment.h"
#include "pipe_sim/RFIDTag.h"
#include "geometry_msgs/Pose2D.h"
#include "arc.h"
#include "line.h"
#include "rssi_map.h"
#include "pipe_sim/pipe_sim_drawer.h"
#include <unistd.h>
#include <opencv2/opencv.hpp>


#ifndef CREATEMAP_H
#define CREATEMAP_H

//Constants
const char MAP_SYMBOL = '~';
const char RSSI_MAP_SYMBOL = '@';
const std::string MAP_PATH = "catkin_ws/src/pipe_sim/maps/";

/*CreatMap Class is used to take a YAML Map instruction file and uses it to
 * create a PipeLine used by PipeMap, Map Image and a Mapping Yaml file */

class CreateMap
{
private:
    //booleans for checking proccesses
    bool pipe_line_set_;
    bool instructions_set_;
    bool image_created_;
    bool yaml_created_;
    bool rfid_created_;
    
    double diameter_;           //diameter of pipe in metres
    double pipe_thickness_;     //thickness of pipe in metres
    double offset_;             //image offset from minimum and maximum points in metres
    double resolution_;         //resolution of map in metres per pixel
    double occupied_thresh_;    //for mapping yaml file
    double free_thresh_;        //for mapping yaml file

    std::string map_name_;                                          //name of map
    std::string map_path_;                                          //path of map file
    std::vector<pipe_sim::PipeDrawingInstructions> instructions_;   //drawing instructions
    geometry_msgs::Pose2D min_;                                     //min pose of pipe segments
    geometry_msgs::Pose2D max_;                                     //max pose of pipe segments
    pipe_sim::PipeLine pipe_line_;                                  //pipeline
    std::vector<pipe_sim::PipeLine> sub_pipe_lines_;                //not in use
    std::vector<pipe_sim::RFIDTag> rfid_tags_;                      //rfid tags in pipe
    YAML::Node node_;                                               //node used for yaml parsing

    //in class methods
    void setNamePath(std::string yamlfile);
    void setInstructions();
    void parseYaml();
    void createYaml();

public:
    //Constructors and Destructors
    CreateMap();
    ~CreateMap();
    CreateMap(std::string yamlfile, double offset, double resolution,
              double occupied_thresh, double free_thresh);

    //print method for Pipe Line
    static void printPipeLine(pipe_sim::PipeLine pipe_line);
    //Method to Create the Image
    void createImage(PipeSimDrawer &drawer);
    //Method to Add an RFID Tag to the Pipe Instructions
    bool addRFIDTagNode(pipe_sim::RFIDTag rfid_tag);

    //Getter Functions
    std::string getImagePath();
    std::string getYamlPath();
    std::string getRSSIYamlPath();
    std::string getMapName();
    pipe_sim::PipeLine getPipeLine();
    geometry_msgs::Pose2D getMinPoint() const;
    geometry_msgs::Pose2D getMaxPoint() const;
    double getOffset() const;
    double getResolution() const;
    std::vector<pipe_sim::RFIDTag> getRFIDTags();

    //Emits the Yaml if new RFID Tags are added to Map
    void emitYaml();

    //Boolean Check Methods
    bool success();
    bool rfid();
};

#endif // CREATEMAP_H
