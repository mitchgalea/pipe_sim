#include "pipe_sim/pipe_sim_drawer.h"

PipeSimDrawer::PipeSimDrawer()
    :set_(false)
{}

PipeSimDrawer::PipeSimDrawer(geometry_msgs::Pose2D min, geometry_msgs::Pose2D max,
                                       double offset, double resolution)
    :set_(false)
{
    min_.x = min.x - offset;
    max_.x = max.x + offset;
    min_.y = min.y - offset;
    max_.y = max.y + offset;

    resolution_ = resolution;
    offset_ = offset;

    set_ = true;
}

cv::Point PipeSimDrawer::pointFromPose(geometry_msgs::Pose2D pose)
{
    cv::Point point;
    point.x = (int)(( pose.x - min_.x) / resolution_);
    point.y = (int)((max_.y - pose.y)/ resolution_);

    return point;
}

void PipeSimDrawer::printTransformParameters()
{
    std::cout << "Min (x: " << min_.x << ", y: " << min_.y << ")" <<std::endl;
    std::cout << "Max (x: " << max_.x << ", y: " << max_.y << ")" <<std::endl;

    std::cout << "Resolution: " << resolution_ << std::endl;
}

void PipeSimDrawer::drawPipeLine(cv::Mat &image, std::vector<pipe_sim::PipeDrawingInstructions> instructions, double diameter, double pipe_thickness)
{
    std::cout << "Image Size (columns: " <<  image.cols <<  ", rows: " <<image.rows << ")" <<std::endl;
    for(auto instruction:instructions)
    {
        if(instruction.straight)
        {
            cv::line(image, pointFromPose(instruction.start), pointFromPose(instruction.end),
                     CV_RGB(0, 0, 0), int((instruction.diameter + (2 * instruction.pipe_thickness))/resolution_), CV_AA);
        }
        else
        {
            cv::ellipse(image, pointFromPose(instruction.centre), cv::Size(abs(int(instruction.radius/resolution_)), abs(int(instruction.radius/resolution_))),
                       (instruction.rotate_angle * (180/M_PI)), 0, (instruction.angle * (180/M_PI)), CV_RGB(0, 0, 0), int((instruction.diameter + (2 * instruction.pipe_thickness))/resolution_), CV_AA);
        }
    }

    for(auto instruction:instructions)
    {
        if(instruction.straight)
        {
            cv::line(image, pointFromPose(instruction.start), pointFromPose(instruction.end),
                     CV_RGB(255, 255, 255), int(instruction.diameter/resolution_), CV_AA);
        }
        else
        {
            cv::ellipse(image, pointFromPose(instruction.centre), cv::Size(abs(int(instruction.radius/resolution_)), abs(int(instruction.radius/resolution_))),
                       (instruction.rotate_angle * (180/M_PI)), 0, (instruction.angle * (180/M_PI)), CV_RGB(255, 255, 255), int(instruction.diameter/resolution_), CV_AA);
        }
    }

    for(auto instruction:instructions)
    {
        if(instruction.straight)
        {         
            cv::line(image, pointFromPose(instruction.start), pointFromPose(instruction.end),
                     LIGHT_GREEN, 1, CV_AA);
        }
        else
        {
            cv::ellipse(image, pointFromPose(instruction.centre), cv::Size(abs(int(instruction.radius/resolution_)), abs(int(instruction.radius/resolution_))),
                       (instruction.rotate_angle * (180/M_PI)), 0, (instruction.angle * (180/M_PI)), LIGHT_GREEN, 1, CV_AA);
        }
    }
}

void PipeSimDrawer::drawRobot(cv::Mat &image, pipe_sim::PipeRobot robot, cv::Scalar colour, int outline, bool arrow, int radius)
{
    if(arrow)
    {
        double arrow_length = (std::min(image.cols, image.rows) * resolution_) * ARROW_LENGTH;
        geometry_msgs::Pose2D arrow_pose;
        double theta = robot.pose.theta;
        if(robot.direction == false) theta = angles::normalize_angle_positive(theta - M_PI);

        arrow_pose.x = robot.pose.x + arrow_length * cos(theta);
        arrow_pose.y = robot.pose.y + arrow_length * sin(theta);
        cv::arrowedLine(image, pointFromPose(robot.pose), pointFromPose(arrow_pose), CV_RGB(0, 0, 0), 3, CV_AA);
    }
    if(outline > 0)cv::circle(image, pointFromPose(robot.pose), radius + outline, BLACK, -1 , CV_AA);
    cv::circle(image, pointFromPose(robot.pose), radius, colour, -1 , CV_AA);
}

void PipeSimDrawer::drawRFIDTag(cv::Mat &image, pipe_sim::RFIDTag rfid_tag, cv::Scalar colour)
{
    cv::circle(image, pointFromPose(rfid_tag.pose), RFID_RADIUS + 12, LIGHT_GREY, -1 , CV_AA);
    cv::circle(image, pointFromPose(rfid_tag.pose), RFID_RADIUS, colour, -1 , CV_AA);
}

void PipeSimDrawer::drawObjects(cv::Mat &image, std::vector<pipe_sim::RFIDTag> rfid_tags, pipe_sim::PipeRobots robots)
{
    for(auto rfid_tag:rfid_tags)
    {
        drawRFIDTag(image, rfid_tag, BLACK);
    }
    if(robots.robots.size() > 0)
    {
        for(auto robot:robots.robots)
        {
            drawRobot(image, robot, ORANGE);
        }
        //drawRobot(image, robots.predicted_robot, BLUE, 2, true, ROBOT_RADIUS + 2);
        if(robots.simulated_robot_set) drawRobot(image, robots.simulated_robot, RED, 2, true, ROBOT_RADIUS + 2);
    }
}


