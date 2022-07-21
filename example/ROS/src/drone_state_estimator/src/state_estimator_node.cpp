/**
 * @file state_estimator_node.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief fuse the pose from vio and map-based localization
 * @version 0.1
 * @date 2021-10-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "drone_state_estimator/state_estimator_warpper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_state_estimator");
    ros::NodeHandle nh("~");
    state_estimator_warpper state_estimator(nh);
    ros::spin();

    return 0;
}