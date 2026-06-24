// Dummy CustomMsg: not needed for Velodyne-type lidar (lidar_type=2)
// FAST-LIO2 uses this only for lidar_type=1 (AVIA/Livox)
#ifndef LIVOX_ROS_DRIVER_CUSTOMMSG_H
#define LIVOX_ROS_DRIVER_CUSTOMMSG_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <vector>

namespace livox_ros_driver {

struct CustomPoint {
    float x, y, z;
    float reflectivity;
    uint32_t offset_time;
    uint8_t line;
};

struct CustomMsg {
    typedef boost::shared_ptr<CustomMsg> Ptr;
    typedef boost::shared_ptr<const CustomMsg> ConstPtr;
    std_msgs::Header header;
    uint64_t timebase;
    uint32_t point_num;
    uint8_t  lidar_id;
    std::vector<CustomPoint> points;
};

} // namespace livox_ros_driver

#endif
