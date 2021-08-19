/**
 * @file msf_core.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the realization of msf_master.cpp
 * @version 0.1
 * @date 2021-08-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "msf_core.h"

namespace multiSensorFusion
{
    msf_core::msf_core()
            : initialized_(false), isWithMap_(false)
    {
        initializer_ = std::make_shared<msf_initializer>();
        imuProcessor_ = std::make_shared<msf_imu_processor>();
        vioProcessor_ = std::make_shared<msf_vio_processor>();
        mapLocProcessor_ = std::make_shared<msf_mapLoc_processor>();
    }

    bool msf_core::isInitialized() const
    {
        return initialized_;
    }

    void msf_core::inputIMU(const imuData &data)
    {
        if (!isInitialized())
        {
            initializer_->addIMU(data);
            std::cerr << "[msf_core]: Waiting for initializing the filter..." << std::endl;
        } else
        {
            currentState_.timestamp_ = data.timestamp_;
            currentState_.imuData_ = data;
            imuProcessor_->predictState(state_buffer_.end()->second, currentState_);
            imuProcessor_->propagateCov(state_buffer_.end()->second, currentState_);
            state_buffer_.insert(std::pair<double, baseState>(currentState_.timestamp_, currentState_));
        }
    }

    void msf_core::inputVIO(const vioData &data)
    {
        if (!isInitialized())
        {
            if (initializer_->initializeUsingVIO(data, currentState_))
                initialized_ = true;
        } else
        {
            measurement_buffer_.insert(std::pair<double, sensorType>(data.timestamp_, VIO));
            vioData_buffer_.insert(std::pair<double, vioData>(data.timestamp_, data));
            applyMeasurement();
        }
    }

    void msf_core::inputMapLoc(const mapLocData &data)
    {
        if (!isInitialized())
            std::cerr << "[msf_core]: Waiting for initializing the filter..." << std::endl;
        else
        {
            if (!isWithMap_)
            {
                auto it = state_buffer_.lower_bound(data.timestamp_ - 0.005);
                if (fabs(it->first - data.timestamp_) > 0.005)

                    std::cerr << "[msf_core]: Map pose and IMU's timestamps are not synchronized!" << std::endl;
                else
                    mapLocProcessor_->getInitTransformation(it->second, data);
            }
            else
            {
                measurement_buffer_.insert(std::pair<double, sensorType>(data.timestamp_, MapLoc));
                mapLocData_buffer_.insert(std::pair<double, mapLocData>(data.timestamp_, data));
                applyMeasurement();
            }
        }
    }

    baseState msf_core::outputState() const
    {
        return baseState();
    }

    void msf_core::applyMeasurement()
    {

    }
}