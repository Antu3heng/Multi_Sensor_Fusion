/**
 * @file msf_core.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_master.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MULTI_SENSOR_FUSION_MSF_CORE_H
#define MULTI_SENSOR_FUSION_MSF_CORE_H

#include <iostream>
#include <memory>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"
#include "msf_initializer.h"
#include "msf_imu_processor.h"
#include "msf_vio_processor.h"
#include "msf_mapLoc_processor.h"
#include "msf_waypoint_processor.h"

namespace multiSensorFusion
{
    class msf_core
    {
    public:
        msf_core();

        ~msf_core() = default;

        bool isInitialized() const;

        void inputIMU(const imuDataPtr &data);

        void inputVIO(const odomDataPtr &data);

        void inputMapLoc(const poseDataPtr &data);

        void inputWaypoint(const posDataPtr &data);

        void inputMoCap(const poseDataPtr &data);

        baseState outputCurrentState();

        bool useMoCap_;

    private:
        bool addMeasurement(const baseDataPtr &data);

        void applyMeasurement(const double &timestamp);

        void checkFutureMeasurement();

        void pruneBuffer();

        std::shared_ptr<msf_initializer> initializer_;
        std::shared_ptr<msf_imu_processor> imuProcessor_;
        std::shared_ptr<msf_vio_processor> vioProcessor_;
        std::shared_ptr<msf_mapLoc_processor> mapLocProcessor_;
        std::shared_ptr<msf_waypoint_processor> waypointProcessor_;
        std::shared_ptr<msf_mapLoc_processor> moCapProcessor_;

        bool initialized_;
        bool isWithMap_;
        bool isWithMoCap_;

        baseStatePtr currentState_;

        const int max_buffer_size = 1000;

        std::map<double, baseStatePtr> state_buffer_;

        std::map<double, baseDataPtr> meas_buffer_;
        std::map<double, baseDataPtr> futureMeas_buffer_;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_CORE_H
