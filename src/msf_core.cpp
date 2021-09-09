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
            checkFutureMeasurement();
        }
    }

    void msf_core::inputVIO(const odomDataPtr &data)
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
            if (addMeasurement(data))
                applyMeasurement(data->timestamp_);
        }
    }

    void msf_core::inputMapLoc(const poseDataPtr &data)
    {
        if (isInitialized())
        {
            if (!isWithMap_)
            {
                auto it = state_buffer_.lower_bound(data->timestamp_ - 0.003);
                if (it != state_buffer_.end())
                {
                    if (fabs(it->first - data->timestamp_) <= 0.003)
                    {
                        mapLocProcessor_->getInitTransformation(it->second, data);
                        isWithMap_ = true;
                    }
#ifdef TEST_DEBUG
                    else
                        std::cerr
                                << "[msf_core]: Map pose and IMU's timestamps are not synchronized, which can't be used to get the map!"
                                << std::endl;
#endif
                }
#ifdef TEST_DEBUG
                else
                    std::cerr << "[msf_core]: Map pose is forward the IMU, which can't be used to get the map!" << std::endl;
#endif
            } else
            {
                if (addMeasurement(data))
                    applyMeasurement(data->timestamp_);
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



    bool msf_core::addMeasurement(const baseDataPtr &data)
    {
        auto it = state_buffer_.lower_bound(data->timestamp_ - 0.003);
        if (it == state_buffer_.end())
        {
#ifdef TEST_DEBUG
            std::cerr << "[msf_core]: The new measurement is forward the states!" << std::endl;
#endif
            futureMeas_buffer_.insert(std::pair<double, baseDataPtr>(data->timestamp_, data));
            return false;
        } else
        {
            if (fabs(it->first - data->timestamp_) <= 0.003)
            {
                meas_buffer_.insert(std::pair<double, baseDataPtr>(data->timestamp_, data));
                return true;
            } else
            {
#ifdef TEST_DEBUG
                std::cerr << "[msf_core]: The new measurement and states' timestamps are not synchronized!"
                          << std::endl;
#endif
                return false;
            }
        }
    }

    void msf_core::applyMeasurement(const double &timestamp)
    {
        auto itState = state_buffer_.lower_bound(timestamp - 0.003);
        for (auto itSensor = meas_buffer_.find(timestamp); itSensor != meas_buffer_.end(); ++itSensor)
        {
            while (itState != state_buffer_.lower_bound(itSensor->first - 0.003))
            {
                auto itLastState = itState;
                imuProcessor_->predict(itLastState->second, (++itState)->second);
            }
            switch (itSensor->second->type_)
            {
                case VIO:
                {
#ifdef TEST_DEBUG
                    std::cerr << "[msf_core]: VIO update!!!"
                          << std::endl;
#endif
                    vioProcessor_->updateState(itState->second, std::dynamic_pointer_cast<odomData>(itSensor->second));
                    break;
                }
                case MapLoc:
                {
#ifdef TEST_DEBUG
                    std::cerr << "[msf_core]: MapLoc update!!!"
                          << std::endl;
#endif
                    mapLocProcessor_->updateState(itState->second,
                                                  std::dynamic_pointer_cast<poseData>(itSensor->second));
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

    void msf_core::checkFutureMeasurement()
    {
        for (auto it = futureMeas_buffer_.begin(); it != futureMeas_buffer_.end();)
        {
            auto itState = state_buffer_.lower_bound(it->first - 0.003);
            if (itState == state_buffer_.end())
                break;
            else
            {
                if (fabs(it->first - itState->first) <= 0.003)
                    meas_buffer_.insert(*it);
                it = futureMeas_buffer_.erase(it);
            }
        }
    }
}