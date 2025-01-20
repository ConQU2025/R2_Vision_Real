#include <ros/ros.h>
#include "lines_map.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "load_lines_map");
    ros::NodeHandle nh("~");

    std::string imagePath;
    nh.param<std::string>("lines_file", imagePath, "lines.jpg");

    LinesMap lines_map;
    lines_map.loadImage(imagePath);
    cv::Mat resultImage = lines_map.getLinesMap();

    cv::imshow("lines", resultImage);
    cv::waitKey(0);
    return 0;
}
