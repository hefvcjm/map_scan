#include "map_scan.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_scan");
    map_scan::MapScan map_scan;
    ROS_INFO("map scan node started");
    ros::spin();
    return 0;
}