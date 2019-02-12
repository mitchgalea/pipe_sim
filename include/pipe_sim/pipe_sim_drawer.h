#include "geometry_msgs/Pose2D.h"
#include "pipe_sim/PipeDrawingInstructions.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "pipe_sim/PipeRobot.h"
#include "pipe_sim/PipeRobots.h"
#include "pipe_sim/RFIDTag.h"
#include "angles/angles.h"
#include <vector>

#ifndef PIPESIMDRAWER_H
#define PIPESIMDRAWER_H

//Constants for OpenCV Colours
const cv::Scalar ORANGE = CV_RGB(153, 204, 255);
const cv::Scalar RED = CV_RGB(255, 0, 0);
const cv::Scalar BLUE = CV_RGB(0, 0, 255);
const cv::Scalar BLACK = CV_RGB(0, 0, 0);
const cv::Scalar WHITE = CV_RGB(255, 255, 255);
const cv::Scalar LIGHT_GREY = CV_RGB(220, 220, 220);
const cv::Scalar LIGHT_GREEN = CV_RGB(173,255,47);

//Constance
const int ROBOT_RADIUS = 12;
const int RFID_RADIUS = 6;
constexpr double ARROW_LENGTH = 0.05;

/*Class Used to Draw PipeMaps in OpenCV*/
class PipeSimDrawer
{
private:

    geometry_msgs::Pose2D min_;     //min pose of Map
    geometry_msgs::Pose2D max_;     //max pose of Map
    double resolution_;             //resolution of Map
    double offset_;                 //offset of PipeSegments to Image Border

    //booleans
    bool set_;

public:
    //Constructors
    PipeSimDrawer();
    PipeSimDrawer(geometry_msgs::Pose2D min, geometry_msgs::Pose2D max,
                       double offset, double resolution);

    //transforms a Pose to a CV Point
    cv::Point pointFromPose(geometry_msgs::Pose2D pose);
    //trandsorms a CV Point to a Pose
    geometry_msgs::Pose2D poseFromPoint(cv::Point point);
    //Method Used to Draw PipeLine
    void drawPipeLine(cv::Mat &image, std::vector<pipe_sim::PipeDrawingInstructions> instructions, double diameter, double pipe_thickness);
    //Method Used to Draw a Robot
    void drawRobot(cv::Mat &image, pipe_sim::PipeRobot robot,  cv::Scalar colour, int outline = 0 ,bool arrow = false, int radius = ROBOT_RADIUS);
    //Method Used to Draw an RFID Tag
    void drawRFIDTag(cv::Mat &image, pipe_sim::RFIDTag rfid_tag, cv::Scalar colour);
    //Method Used to Draw Objects including Robots and RFID Tags
    void drawObjects(cv::Mat &image, std::vector<pipe_sim::RFIDTag> rfid_tags, pipe_sim::PipeRobots robots);

    //Print Function
    void printTransformParameters();
};
#endif // PIPESIMDRAWER_H
