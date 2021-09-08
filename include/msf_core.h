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

namespace multiSensorFusion
{
    class msf_core
    {
    public:
        msf_core();

        ~msf_core() = default;

        bool isInitialized() const;

        void inputIMU(const imuDataPtr &data);

        void inputVIO(const vioDataPtr &data);

        void inputMapLoc(const mapLocDataPtr &data);

        baseState outputCurrentState();

    private:
        bool addMeasurement(sensorType type, const double &timestamp);

        void applyMeasurement(const double &timestamp);

        void screenFutureMeasurement();

        std::shared_ptr<msf_initializer> initializer_;
        std::shared_ptr<msf_imu_processor> imuProcessor_;
        std::shared_ptr<msf_vio_processor> vioProcessor_;
        std::shared_ptr<msf_mapLoc_processor> mapLocProcessor_;

        bool initialized_;
        bool isWithMap_;
        bool isFutureMeasurement_;

        baseStatePtr currentState_;

        std::map<double, baseStatePtr> state_buffer_;

        std::map<double, sensorType> measurement_buffer_;
        std::map<double, sensorType> futureMeasurement_buffer_;
        std::map<double, vioDataPtr> vioData_buffer_;
        std::map<double, mapLocDataPtr> mapLocData_buffer_;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_CORE_H
