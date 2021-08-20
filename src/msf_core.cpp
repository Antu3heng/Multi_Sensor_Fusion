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

    void msf_core::inputIMU(const imuDataPtr &data)
    {
        if (!isInitialized())
        {
            initializer_->addIMU(data);
            std::cerr << "[msf_core]: Waiting for initializing the filter..." << std::endl;
        } else
        {
            addSensorData(IMU, data->timestamp_);

            currentState_ = std::make_shared<baseState>();
            currentState_->timestamp_ = data->timestamp_;
            currentState_->imuData_ = data;

            imuProcessor_->predictState((--(state_buffer_.end()))->second, currentState_);
            imuProcessor_->propagateCov((--(state_buffer_.end()))->second, currentState_);

            state_buffer_.insert(std::pair<double, baseStatePtr>(currentState_->timestamp_, currentState_));
        }
    }

    void msf_core::inputVIO(const vioDataPtr &data)
    {
        if (!isInitialized())
        {
            if (initializer_->initializeUsingVIO(data, currentState_))
            {
                initialized_ = true;
                addSensorData(IMU, currentState_->timestamp_);
                state_buffer_.insert(std::pair<double, baseStatePtr>(currentState_->timestamp_, currentState_));
            }
        } else
        {
            if (addSensorData(VIO, data->timestamp_))
            {
                vioData_buffer_.insert(std::pair<double, vioDataPtr>(data->timestamp_, data));
                applyMeasurement(data->timestamp_);
            }
        }
    }

    void msf_core::inputMapLoc(const mapLocDataPtr &data)
    {
        if (!isInitialized())
            std::cerr << "[msf_core]: Waiting for initializing the filter..." << std::endl;
        else
        {
            if (!isWithMap_)
            {
                auto it = state_buffer_.lower_bound(data->timestamp_ - 0.003);
                if (fabs(it->first - data->timestamp_) > 0.003)
                    std::cerr
                            << "[msf_core]: Map pose and IMU's timestamps are not synchronized, which can't be used to get the map!"
                            << std::endl;
                else
                    mapLocProcessor_->getInitTransformation(it->second, data);
            } else
            {
                if (addSensorData(MapLoc, data->timestamp_))
                {
                    mapLocData_buffer_.insert(std::pair<double, mapLocDataPtr>(data->timestamp_, data));
                    applyMeasurement(data->timestamp_);
                }
            }
        }
    }

    baseState msf_core::outputCurrentState()
    {
        mapLocProcessor_->transformStateToMap(currentState_);
        return *currentState_;
    }

    // TODO: Figure out how to use the forward sensor data
    bool msf_core::addSensorData(sensorType type, const double &timestamp)
    {
        if (type != IMU)
        {
            auto it = state_buffer_.lower_bound(timestamp - 0.003);
            if (it == state_buffer_.end())
            {
                std::cerr << "[msf_core]: The new measurement is forward the states!" << std::endl;
                // forwardSensorData_buffer_.insert(std::pair<double, sensorType>(timestamp, type));
                return false;
            } else
            {
                if (fabs(it->first - timestamp) > 0.003)
                {
                    std::cerr << "[msf_core]: The new measurement and states' timestamps are not synchronized!"
                              << std::endl;
                    return false;
                } else
                {
                    sensorData_buffer_.insert(std::pair<double, sensorType>(timestamp, type));
                    return true;
                }
            }
        } else
        {
            sensorData_buffer_.insert(std::pair<double, sensorType>(timestamp, type));
            return true;
        }
    }

    void msf_core::applyMeasurement(const double &timestamp)
    {
        for (auto itSensor = sensorData_buffer_.find(timestamp); itSensor != sensorData_buffer_.end(); ++itSensor)
        {
            auto itState = state_buffer_.lower_bound(itSensor->first - 0.003);
            switch (itSensor->second)
            {
                case IMU:
                {
                    auto itStateNow = itState;
                    imuProcessor_->predictState((--itState)->second, itStateNow->second);
                    imuProcessor_->propagateCov((--itState)->second, itStateNow->second);
                    break;
                }
                case VIO:
                {
                    vioProcessor_->updateState(itState->second, vioData_buffer_.find(itSensor->first)->second);
                    break;
                }
                case MapLoc:
                {
                    mapLocProcessor_->updateState(itState->second, mapLocData_buffer_.find(itSensor->first)->second);
                    break;
                }
                default:
                    break;
            }
        }
    }
}