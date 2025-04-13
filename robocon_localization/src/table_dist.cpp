#include <ros/ros.h>
#include "distance_lookup.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "table_dist");
    ros::NodeHandle nh("~");

    std::string filename;
    nh.param<std::string>("table_file", filename, "distances.txt");

    DistanceLookup distanceLookup;
    distanceLookup.init(filename);

    for (int pixel = 0; pixel <= distanceLookup.getMaxPixel(); ++pixel) {
        ROS_INFO("间隔像素: %d, 距离: %f", pixel, distanceLookup.getDistance(pixel));
    }

    ros::spin();
    return 0;
}
