#ifndef __MAP_SCAN_H__
#define __MAP_SCAN_H__
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>
#include <cmath>
#include "map_scan/GetMapScan.h"
#include "map.h"

namespace map_scan
{

using namespace std;

class MapScan
{
public:
    MapScan();
    ~MapScan()
    {
        map_free(map);
    };

private:
    ros::NodeHandle nh;
    ros::ServiceServer scan_srv;
    ros::ServiceClient get_map_clinet;

    map_t *map;

    /**
     * @brief function for service to get caster scan
     * 
     * @param req laser's pose where you want to get a lasercan
     * @param res laserscan that you have to fill the info except ranges
     * @return true if succeed, false otherwise.
     */
    bool getCasterScan(map_scan::GetMapScan::Request &req, map_scan::GetMapScan::Response &res);

    /**
     * @brief convert OccupancyGrid to map_t
     * 
     * @param map_msg OccupancyGrid massege
     * @return map_t map_t pointer
     */
    map_t *convertMap(const nav_msgs::OccupancyGrid &map_msg) const;
};

}; // namespace map_scan

#endif // !__MAP_SCAN_H__