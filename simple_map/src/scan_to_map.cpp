#include "simple_map/scan_to_map.h"
#include <ros/ros.h>

//разрешение карты
double map_resolution = 0.1;
//размер карты в клетках
int map_width = 100;
int map_height = 100;

void scan_to_map(const sensor_msgs::LaserScan& scan, nav_msgs::OccupancyGrid& map_msg, const tf::Transform& transform) {
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) 
    {
        double angle = scan.angle_min + scan.angle_increment * i;
        tf::Vector3 v = transform(tf::Vector3(scan.ranges[i]*cos(angle), scan.ranges[i]*sin(angle), 0));
        v -= tf::Vector3(map_msg.info.origin.position.x, map_msg.info.origin.position.y, 0);
        if (v.x() >= 0 && v.y() >= 0 &&
                v.x() < map_msg.info.resolution * map_msg.info.width && v.y() < map_msg.info.resolution * map_msg.info.height) 
        {
            std::size_t x = v.x() / map_msg.info.resolution;
            std::size_t y = v.y() / map_msg.info.resolution;
            
            map_msg.data[ y* map_width + x] = 100;
           
        }
        
        for (std::size_t j = 0; j*map_msg.info.resolution < scan.ranges[i]; ++j)
        {
        
        	tf::Vector3 v_j = transform(tf::Vector3(j*map_msg.info.resolution*cos(angle), j*map_msg.info.resolution*sin(angle), 0));
        	v_j -= tf::Vector3(map_msg.info.origin.position.x, map_msg.info.origin.position.y, 0);
        	
        	if (v_j.x() >= 0 && v_j.y() >= 0 && v_j.x() < map_msg.info.resolution * map_msg.info.width && v_j.y() < map_msg.info.resolution * map_msg.info.height) 
        	{
            		std::size_t x_j = v_j.x() / map_msg.info.resolution;
            		std::size_t y_j = v_j.y() / map_msg.info.resolution;
            
            		map_msg.data[ y_j* map_width + x_j] = 0;
           
        	}
        
        }
     }
}
