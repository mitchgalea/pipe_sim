#include "pipe_sim/create_map.h"

CreateMap::CreateMap()
    :pipe_line_set_(false),
     instructions_set_(false),
     image_created_(false),
     yaml_created_(false),
     rfid_created_(false)
{}

CreateMap::CreateMap(std::string yamlfile, double offset, double resolution,
                     double occupied_thresh, double free_thresh):
    pipe_line_set_(false),
    instructions_set_(false),
    image_created_(false),
    yaml_created_(false),
    rfid_created_(false),
    offset_(offset),
    resolution_(resolution),
    occupied_thresh_(occupied_thresh),
    free_thresh_(free_thresh),
    map_path_(MAP_PATH),
    map_name_(yamlfile)
{
    parseYaml();
    setInstructions();
    createYaml();
}

CreateMap::~CreateMap()
{
    //emitYaml();
}

void CreateMap::parseYaml()
{

    char buff[FILENAME_MAX];
    getcwd( buff, FILENAME_MAX );
    std::string current_working_dir(buff);

    std::cout << current_working_dir << std::endl;

    std::string file_path = map_path_;
    file_path.append(map_name_);
    file_path.append("/");
    file_path.append(map_name_);
    file_path += MAP_SYMBOL;
    file_path.append(".yaml");
    node_ = YAML::LoadFile(file_path);

    if(node_["InitialPose"] && node_["PipeLineSegments"] && node_["Diameter"] && node_["PipeThickness"])
    {

        YAML::Node pose_node = node_["InitialPose"];
        if(pose_node["x"] && pose_node["y"] && pose_node["theta"])
        {
            pipe_line_.initial_pose.x = pose_node["x"].as<double>();
            pipe_line_.initial_pose.y = pose_node["y"].as<double>();
            pipe_line_.initial_pose.theta = pose_node["theta"].as<double>();
        }
        else
        {
            std::cout << "x y theta fail" << std::endl;
            return;
        }

        YAML::Node pipe_line_node = node_["PipeLineSegments"];
        for(auto i = 0; i < pipe_line_node.size(); i++)
        {
            pipe_sim::PipeLineSegment temp;
            if(pipe_line_node[i]["radius"] && pipe_line_node[i]["distance"])
            {
                temp.radius = pipe_line_node[i]["radius"].as<double>();
                temp.distance = pipe_line_node[i]["distance"].as<double>();
                pipe_line_.pipe_line.push_back(temp);
            }
            else
            {
                std::cout << "segment fail" << std::endl;
                return;
            }
        }
        pipe_line_.diameter = node_["Diameter"].as<double>();
        pipe_line_.pipe_thickness = node_["PipeThickness"].as<double>();
        if(node_["ID"])
        {
            pipe_line_.id = node_["ID"].as<int64_t>();
        }
        pipe_line_.sub_pipe_line = false;

        if(pipe_line_.pipe_line.size() > 0)
        {
            pipe_line_set_ = true;
        }
    }
    if(node_["SubPipeLines"])
    {
        for(int i = 0; i < node_["SubPipeLines"].size(); i++)
        {
            pipe_sim::PipeLine sub_pipe_line;

            sub_pipe_line.sub_pipe_line = true;
            sub_pipe_line.diameter = node_["SubPipeLines"][i]["Diameter"].as<double>();
            sub_pipe_line.pipe_thickness = node_["SubPipeLines"][i]["PipeThickness"].as<double>();
            if(node_["SubPipeLines"][i]["ID"])
            {
                sub_pipe_line.id = node_["SubPipeLines"][i]["ID"].as<int64_t>();
            }
            sub_pipe_line.connect_id = node_["SubPipeLines"][i]["Connection"]["ID"].as<int64_t>();
            sub_pipe_line.connect_distance = node_["SubPipeLines"][i]["Connection"]["Distance"].as<int64_t>();
            sub_pipe_line.theta = node_["SubPipeLines"][i]["Connection"]["theta"].as<double>();

            YAML::Node sub_pipe_line_node = node_["SubPipeLines"][i]["PipeLineSegments"];
            for(auto i = 0; i < sub_pipe_line_node.size(); i++)
            {
                pipe_sim::PipeLineSegment temp;
                if(sub_pipe_line_node[i]["radius"] && sub_pipe_line_node[i]["distance"])
                {
                    temp.radius = sub_pipe_line_node[i]["radius"].as<double>();
                    temp.distance = sub_pipe_line_node[i]["distance"].as<double>();
                    sub_pipe_line.pipe_line.push_back(temp);
                }
                else
                {
                    std::cout << "segment fail" << std::endl;
                    return;
                }
            }
            sub_pipe_lines_.push_back(sub_pipe_line);
        }



    }
    if(node_["RFID"])
    {
        YAML::Node rfid_node = node_["RFID"];
        pipe_sim::RFIDTag temp_tag;
        for(auto i = 0; i < rfid_node.size(); i++)
        {
            temp_tag.epc = rfid_node[i]["epc"].as<int64_t>();
            temp_tag.distance = rfid_node[i]["distance"].as<double>();
            rfid_tags_.push_back(temp_tag);
        }
    }
}

void CreateMap::printPipeLine(pipe_sim::PipeLine pipe_line)
{
    std::cout << "PipeLine: " << pipe_line.id << std::endl;
    if(!pipe_line.sub_pipe_line)
    {
        std::cout << "Initial Pose( x: " << pipe_line.initial_pose.x << ", y: " << pipe_line.initial_pose.y << ", theta: " << pipe_line.initial_pose.theta << ")" << std::endl;
        std::cout << "Diameter: " << pipe_line.diameter << std::endl;
        std::cout << "Pipe Thickness: " << pipe_line.pipe_thickness << std::endl;
        std::cout << "Pipe Segments: " << std::endl;
        for(auto pipe_segment:pipe_line.pipe_line)
        {
            std::cout << "    - Segment(radius: " << pipe_segment.radius << ", distance: "  << pipe_segment.distance << ") "<< std::endl;
        }

    }
    else
    {
        std::cout << "Connection( id: " << pipe_line.connect_id << ", distance: " << pipe_line.connect_distance << ", theta: " << pipe_line.theta << ")" << std::endl;
        std::cout << "Diameter: " << pipe_line.diameter << std::endl;
        std::cout << "Pipe Thickness: " << pipe_line.pipe_thickness << std::endl;
        std::cout << "Pipe Segments: " << std::endl;
        for(auto pipe_segment:pipe_line.pipe_line)
        {
            std::cout << "    - Segment(radius: " << pipe_segment.radius << ", distance: "  << pipe_segment.distance << ") "<< std::endl;
        }
    }
    std::cout << std::endl;
}

void CreateMap::setInstructions()
{
    if(pipe_line_set_)
    {
        geometry_msgs::Pose2D current = pipe_line_.initial_pose;

        min_ = current;
        max_ = current;

        double accumulated_distance = 0;
        
        for(auto pipe_segment:pipe_line_.pipe_line)
        {
            pipe_sim::PipeDrawingInstructions instruction;
            geometry_msgs::Pose2D min_point;
            geometry_msgs::Pose2D max_point;
            
            PipeSegment * segment;
            if(fabs(pipe_segment.radius) < std::numeric_limits<double>::epsilon())
            {
                Line line(pipe_segment.distance);
                segment = &line;
            }
            else
            {
                Arc arc(pipe_segment.distance, pipe_segment.radius);
                segment = &arc;
            }
            instruction = segment->getDrawingInstructions(current);
            min_point = segment->getMaxMinPoint(current, false);
            max_point = segment->getMaxMinPoint(current, true);
            min_.x = std::min(min_.x, min_point.x);
            min_.y = std::min(min_.y, min_point.y);
            max_.x = std::max(max_.x, max_point.x);
            max_.y = std::max(max_.y, max_point.y);

            for(int i = 0; i < sub_pipe_lines_.size(); i ++)
            {
                if(accumulated_distance + pipe_segment.distance >= sub_pipe_lines_.at(i).connect_distance && accumulated_distance < sub_pipe_lines_.at(i).connect_distance)
                {
                    sub_pipe_lines_.at(i).initial_pose = segment->getPose(current, sub_pipe_lines_.at(i).connect_distance - accumulated_distance);
                    sub_pipe_lines_.at(i).initial_pose.theta =  sub_pipe_lines_.at(i).theta;
                }

            }
            accumulated_distance += pipe_segment.distance;
            current = instruction.end;
            instruction.diameter = pipe_line_.diameter;
            instruction.pipe_thickness = pipe_line_.pipe_thickness;
            instructions_.push_back(instruction);
        }
        
        for(auto pipe_line:sub_pipe_lines_)
        {
            current = pipe_line.initial_pose;
            for(auto pipe_segment:pipe_line.pipe_line)
            {
                pipe_sim::PipeDrawingInstructions instruction;
                geometry_msgs::Pose2D min_point;
                geometry_msgs::Pose2D max_point;

                PipeSegment * segment;
                if(fabs(pipe_segment.radius) < std::numeric_limits<double>::epsilon())
                {
                    Line line(pipe_segment.distance);
                    segment = &line;
                }
                else
                {
                    Arc arc(pipe_segment.distance, pipe_segment.radius);
                    segment = &arc;
                }
                instruction = segment->getDrawingInstructions(current);
                min_point = segment->getMaxMinPoint(current, false);
                max_point = segment->getMaxMinPoint(current, true);
                min_.x = std::min(min_.x, min_point.x);
                min_.y = std::min(min_.y, min_point.y);
                max_.x = std::max(max_.x, max_point.x);
                max_.y = std::max(max_.y, max_point.y);

                current = instruction.end;
                instruction.diameter = pipe_line.diameter;
                instruction.pipe_thickness = pipe_line.pipe_thickness;
                instructions_.push_back(instruction);
            }
        }

        if(instructions_.size() > 0)
        {
            instructions_set_ = true;
        }
    }
}
    

void CreateMap::createImage(PipeSimDrawer& drawer)
{
    if(instructions_set_)
    {
        int height = (max_.y - min_.y + (2 * offset_)) / resolution_;
        int length = (max_.x - min_.x + (2 * offset_)) / resolution_;

        cv::Mat mapImage(height, length,  CV_8UC3, cv::Scalar(128,128,128));
        drawer.drawPipeLine(mapImage, instructions_, pipe_line_.diameter, pipe_line_.pipe_thickness);

        std::string path = map_path_;

        path.append(map_name_);
        path.append("/");
        path.append(map_name_);
        path.append(".png");

        imwrite( path, mapImage);

        image_created_ = true;
    }
}

void CreateMap::createYaml()
{

    std::string path = map_path_;

    path.append(map_name_);
    path.append("/");
    path.append(map_name_);
    path.append(".yaml");

    std::ofstream yamlfile(path);

    yamlfile << "image: " << map_name_ << ".png" << std::endl;
    yamlfile << "resolution: " << resolution_ << std::endl;
    yamlfile << "origin: [" << min_.x << ", " << min_.y << ", " << min_.theta << "]" << std::endl;
    yamlfile << "occupied_thresh: " << occupied_thresh_ << std::endl;
    yamlfile << "free_thresh: " << free_thresh_ << std::endl;
    yamlfile << "negate: 0" << std::endl;

    yamlfile.close();
    yaml_created_ = true;

}

bool CreateMap::success()
{
    if(pipe_line_set_ && instructions_set_
    && yaml_created_)
    {
        return true;
    }
    return false;
}

bool CreateMap::rfid()
{
    if(rfid_created_)return true;
    return false;
}

std::string CreateMap::getImagePath()
{
    std::string image_path;
    image_path.append(map_path_);
    image_path.append(map_name_);
    image_path.append("/");
    image_path.append(map_name_);
    image_path.append(".png");

    return image_path;
}

std::string CreateMap::getYamlPath()
{
    std::string yaml_path;
    yaml_path.append(map_path_);
    yaml_path.append(map_name_);
    yaml_path.append("/");
    yaml_path.append(map_name_);
    yaml_path.append(".yaml");

    return yaml_path;
}

std::string CreateMap::getRSSIYamlPath()
{
    std::string file_path = map_path_;
    file_path.append(map_name_);
    file_path.append("/");
    file_path.append(map_name_);
    file_path += RSSI_MAP_SYMBOL;
    file_path.append(".yaml");
    return file_path;
}

std::string CreateMap::getMapName()
{
    return map_name_;
}

pipe_sim::PipeLine CreateMap::getPipeLine()
{
    return pipe_line_;
}

std::vector<pipe_sim::RFIDTag> CreateMap::getRFIDTags()
{
    return rfid_tags_;
}

bool CreateMap::addRFIDTagNode(pipe_sim::RFIDTag rfid_tag)
{
    YAML::Node rfid_node;

    std::string epc = std::to_string(rfid_tag.epc);
    std::string distance = std::to_string(rfid_tag.distance);
    rfid_node["epc"] = epc;
    rfid_node["distance"] = distance;

    node_["RFID"].push_back(rfid_node);

    return true;
}

geometry_msgs::Pose2D CreateMap::getMinPoint() const {return min_;}
geometry_msgs::Pose2D CreateMap::getMaxPoint() const {return max_;}
double CreateMap::getOffset() const {return offset_;}
double CreateMap::getResolution() const {return resolution_;}


void CreateMap::emitYaml()
{
    std::string yaml_path;
    yaml_path.append(map_path_);
    yaml_path.append(map_name_);
    yaml_path.append("/");
    yaml_path.append(map_name_);
    yaml_path.append("~.yaml");

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;

    emitter << YAML::Key << "InitialPose";
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << node_["InitialPose"]["x"];
    emitter << YAML::Key << "y" << YAML::Value << node_["InitialPose"]["y"];
    emitter << YAML::Key << "theta" << YAML::Value << node_["InitialPose"]["theta"];
    emitter << YAML::EndMap;

    emitter << YAML::Key << "Diameter" << YAML::Value << node_["Diameter"];

    emitter << YAML::Key << "PipeThickness" << YAML::Value << node_["PipeThickness"];

    emitter << YAML::Key << "PipeLineSegments";
    emitter << YAML::Value << YAML::BeginSeq;
    for(int i = 0; i < node_["PipeLineSegments"].size(); i ++)
    {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "radius" << YAML::Value << node_["PipeLineSegments"][i]["radius"];
        emitter << YAML::Key << "distance" << YAML::Value << node_["PipeLineSegments"][i]["distance"];
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "RFID";
    emitter << YAML::Value << YAML::BeginSeq;
    for(int i = 0; i < node_["RFID"].size(); i ++)
    {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "epc" << YAML::Value << node_["RFID"][i]["epc"];
        emitter << YAML::Key << "distance" << YAML::Value << node_["RFID"][i]["distance"];
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;

    emitter << YAML::EndMap;

    std::ofstream fout(yaml_path);
    fout << emitter.c_str();
}
