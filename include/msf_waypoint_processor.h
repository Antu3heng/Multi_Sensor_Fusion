/**
 * @file msf_waypoint_processor.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_imu_processor.cpp
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_WAYPOINT_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_WAYPOINT_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace multiSensorFusion
{
    class msf_waypoint_processor
    {
    public:
        msf_waypoint_processor() = default;

        ~msf_waypoint_processor() = default;

        static void  updateState(baseStatePtr &currentState, const posDataPtr &data);

    };
}

#endif //MULTI_SENSOR_FUSION_MSF_WAYPOINT_PROCESSOR_H
