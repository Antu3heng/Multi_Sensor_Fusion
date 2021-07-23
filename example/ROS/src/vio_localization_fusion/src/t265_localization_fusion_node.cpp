/**
 * @file vio_localization_fusion.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief fuse the pose from vio and map-based localization
 * @version 3
 * @date 2021-06-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "vio_localization_fusion/t265_localization_wrapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_localization_fusion");
    ros::NodeHandle nh("~");

    t265_localization_wrapper fusion_locator(nh);

    return 0;
}