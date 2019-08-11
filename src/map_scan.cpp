#include "map_scan.h"

namespace map_scan
{

MapScan::MapScan() : map(NULL)
{
    get_map_clinet = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    ROS_WARN("Waiting for GetMap service...");
    get_map_clinet.waitForExistence();
    nav_msgs::GetMap srv;
    // srv.response.map = map;
    if (get_map_clinet.call(srv))
    {
        ROS_INFO("Get the grid map.");
        map = convertMap(srv.response.map);
        scan_srv = nh.advertiseService("map_scan", &MapScan::getCasterScan, this);
    }
    else
    {
        ROS_ERROR("Failed to call service to get grid map");
    }
}

bool MapScan::getCasterScan(map_scan::GetMapScan::Request &req, map_scan::GetMapScan::Response &res)
{
    ROS_INFO("Calling service.");
    res.scan.header.frame_id = req.scan.header.frame_id;
    res.scan.header.stamp = ros::Time::now();
    res.scan.angle_min = req.scan.angle_min;
    res.scan.angle_max = req.scan.angle_max;
    res.scan.angle_increment = req.scan.angle_increment;
    res.scan.time_increment = req.scan.time_increment;
    res.scan.scan_time = req.scan.scan_time;
    res.scan.range_min = req.scan.range_min;
    res.scan.range_max = req.scan.range_max;
    res.scan.ranges.clear();
    ROS_DEBUG("\n\tlaser_info:\n\tangle_min=%f\n\tangle_max=%f\n\tangle_increment=%f\n\trange_min=%f\n\trange_max=%f\n",
              res.scan.angle_min,
              res.scan.angle_max,
              res.scan.angle_increment,
              res.scan.range_min,
              res.scan.range_max);
    int x0, y0, x1, y1;
    double ox, oy, oa;
    ox = req.pose.x;
    oy = req.pose.y;
    oa = req.pose.theta;
    x0 = MAP_GXWX(map, ox);
    y0 = MAP_GYWY(map, oy);
    // ROS_INFO("%d,%d,%d,%d", x0, y0, x1, y1);
    for (float angle = res.scan.angle_min; angle <= res.scan.angle_max + 1e-6; angle += res.scan.angle_increment)
    {
        if (res.scan.angle_increment <= 1e-6)
        {
            ROS_WARN("angle_increment is 0.");
            return false;
        }
        x1 = MAP_GXWX(map, ox + res.scan.range_max * cos(oa + angle));
        y1 = MAP_GYWY(map, oy + res.scan.range_max * sin(oa + angle));
        // ROS_INFO("%d,%d,%d,%d", x0, y0, x1, y1);
        res.scan.ranges.push_back(map_calc_range(map, x0, y0, x1, y1, oa + angle, res.scan.range_max));
        res.scan.intensities.push_back(0.0);
    }
    ROS_INFO("Service done.");
    return true;
}

map_t *MapScan::convertMap(const nav_msgs::OccupancyGrid &map_msg) const
{
    map_t *map = map_alloc();
    ROS_ASSERT(map);

    map->size_x = map_msg.info.width;
    map->size_y = map_msg.info.height;
    map->scale = map_msg.info.resolution;
    map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
    // Convert to player format
    map->cells = (map_cell_t *)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);
    ROS_ASSERT(map->cells);
    for (int i = 0; i < map->size_x * map->size_y; i++)
    {
        if (map_msg.data[i] == 0)
            map->cells[i].occ_state = -1;
        else if (map_msg.data[i] == 100)
            map->cells[i].occ_state = +1;
        else
            map->cells[i].occ_state = 0;
    }

    return map;
}

} // namespace map_scan