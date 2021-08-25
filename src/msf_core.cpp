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
        currentState_ = std::make_shared<baseState>();
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
        } else
        {
            currentState_ = std::make_shared<baseState>();
            currentState_->timestamp_ = data->timestamp_;
            currentState_->imuData_ = data;
            imuProcessor_->predict((--(state_buffer_.end()))->second, currentState_);
            state_buffer_.insert(std::pair<double, baseStatePtr>(currentState_->timestamp_, currentState_));
        }
    }

    void msf_core::inputVIO(const vioDataPtr &data)
    {
        if (!isInitialized())
        {
            if (initializer_->initializeUsingVIO(data, currentState_))
            {
                std::cerr << "[msf_core]: The MSF has been initialized!" << std::endl;
                initialized_ = true;
                state_buffer_.insert(std::pair<double, baseStatePtr>(currentState_->timestamp_, currentState_));
            }
        } else
        {
            if (addMeasurement(VIO, data->timestamp_))
            {
                vioData_buffer_.insert(std::pair<double, vioDataPtr>(data->timestamp_, data));
                applyMeasurement(data->timestamp_);
            }
        }
    }

    void msf_core::inputMapLoc(const mapLocDataPtr &data)
    {
        if (isInitialized())
        {
            if (!isWithMap_)
            {
                auto it = state_buffer_.lower_bound(data->timestamp_ - 0.003);
                if (it != state_buffer_.end() && fabs(it->first - data->timestamp_) > 0.003)
                    std::cerr
                            << "[msf_core]: Map pose and IMU's timestamps are not synchronized, which can't be used to get the map!"
                            << std::endl;
                else
                {
                    mapLocProcessor_->getInitTransformation(it->second, data);
                    isWithMap_ = true;
                }
            } else
            {
                if (addMeasurement(MapLoc, data->timestamp_))
                {
                    mapLocData_buffer_.insert(std::pair<double, mapLocDataPtr>(data->timestamp_, data));
                    applyMeasurement(data->timestamp_);
                }
            }
        }
    }

    baseState msf_core::outputCurrentState()
    {
        currentState_ = (--state_buffer_.end())->second;
        if (isWithMap_)
            mapLocProcessor_->transformStateToMap(currentState_);
        return *currentState_;
    }

    // TODO: Figure out how to use the forward sensor data
    bool msf_core::addMeasurement(sensorType type, const double &timestamp)
    {
        if (type != IMU)
        {
            auto it = state_buffer_.lower_bound(timestamp - 0.003);
            if (it == state_buffer_.end())
            {
                // std::cerr << "[msf_core]: The new measurement is forward the states!" << std::endl;
                // futureMeasurement_buffer_.insert(std::pair<double, sensorType>(timestamp, type));
                return false;
            } else
            {
                if (fabs(it->first - timestamp) > 0.003)
                {
                    // std::cerr << "[msf_core]: The new measurement and states' timestamps are not synchronized!"
                    //           << std::endl;
                    return false;
                } else
                {
                    measurement_buffer_.insert(std::pair<double, sensorType>(timestamp, type));
                    return true;
                }
            }
        } else
        {
            measurement_buffer_.insert(std::pair<double, sensorType>(timestamp, type));
            return true;
        }
    }

    void msf_core::applyMeasurement(const double &timestamp)
    {
        auto itState = state_buffer_.lower_bound(timestamp - 0.003);
        for (auto itSensor = measurement_buffer_.find(timestamp); itSensor != measurement_buffer_.end(); ++itSensor)
        {
            while (itState != state_buffer_.lower_bound(itSensor->first - 0.003))
            {
                auto itLastState = itState;
                imuProcessor_->predict(itLastState->second, (++itState)->second);
            }
            switch (itSensor->second)
            {
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
        while (itState != (--state_buffer_.end()))
        {
            auto itLastState = itState;
            imuProcessor_->predict(itLastState->second, (++itState)->second);
        }
    }
}