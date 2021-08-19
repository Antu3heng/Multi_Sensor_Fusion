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

        void inputIMU(const imuData &data);

        void inputVIO(const vioData &data);

        void inputMapLoc(const mapLocData &data);

        baseState outputState() const;

    private:
        void applyMeasurement();

        std::shared_ptr<msf_initializer> initializer_;
        std::shared_ptr<msf_imu_processor> imuProcessor_;
        std::shared_ptr<msf_vio_processor> vioProcessor_;
        std::shared_ptr<msf_mapLoc_processor> mapLocProcessor_;

        bool initialized_;
        bool isWithMap_;

        baseState currentState_;

        std::map<double, baseState> state_buffer_;

        std::map<double, sensorType> measurement_buffer_;
        std::map<double, vioData> vioData_buffer_;
        std::map<double, mapLocData> mapLocData_buffer_;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_CORE_H
