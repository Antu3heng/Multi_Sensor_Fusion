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
            : useMoCap_(false), initialized_(false), isWithMap_(false), isWithMoCap_(false)
    {
        initializer_ = std::make_shared<msf_initializer>();
        imuProcessor_ = std::make_shared<msf_imu_processor>();
        vioProcessor_ = std::make_shared<msf_vio_processor>(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), true);
        mapLocProcessor_ = std::make_shared<msf_mapLoc_processor>(0.1, 1, true);
        waypointProcessor_ = std::make_shared<msf_waypoint_processor>();
        moCapProcessor_ = std::make_shared<msf_mapLoc_processor>(0.1, 0.1, true);
        currentState_ = std::make_shared<baseState>();
    }

    bool msf_core::isInitialized() const
    {
        return initialized_;
    }

    void msf_core::inputIMU(const imuDataPtr &data)
    {
#ifdef TEST_DEBUG
        std::cout << "======================================" << std::endl;
        std::cout << "state buffer size: " << state_buffer_.size() << std::endl;
        std::cout << "measurement buffer size: " << meas_buffer_.size() << std::endl;
        std::cout << "future measurement buffer size: " << futureMeas_buffer_.size() << std::endl;
        std::cout << std::endl;
#endif
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
            pruneBuffer();
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
                        mapLocProcessor_->setInitTransformation(it->second, data);
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

    void msf_core::inputWaypoint(const posDataPtr &data)
    {
        if (isInitialized())
        {
            if (addMeasurement(data))
                applyMeasurement(data->timestamp_);
        }
    }

    baseState msf_core::outputCurrentState()
    {
        currentState_ = (--state_buffer_.end())->second;
        if (useMoCap_ && isWithMoCap_)
        {
            moCapProcessor_->transformStateToMap(currentState_);
            currentState_->isWithMap_ = true;
        }
        else if (!useMoCap_ && isWithMap_)
        {
            mapLocProcessor_->transformStateToMap(currentState_);
            currentState_->isWithMap_ = true;
        }
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
                case Waypoint:
                {
                    waypointProcessor_->updateState(itState->second,
                                                    std::dynamic_pointer_cast<posData>(itSensor->second));
                    break;
                }
                case MoCap:
                {
                    moCapProcessor_->updateState(itState->second,
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

    void msf_core::pruneBuffer()
    {
        while (state_buffer_.size() > max_buffer_size)
            state_buffer_.erase(state_buffer_.begin());
        while (meas_buffer_.size() > max_buffer_size)
            meas_buffer_.erase((meas_buffer_.begin()));
    }

    void msf_core::inputMoCap(const poseDataPtr &data)
    {
        if (isInitialized())
        {
            if (!isWithMoCap_)
            {
                auto it = state_buffer_.lower_bound(data->timestamp_ - 0.003);
                if (it != state_buffer_.end())
                {
                    if (fabs(it->first - data->timestamp_) <= 0.003)
                    {
                        moCapProcessor_->setInitTransformation(it->second, data);
                        isWithMoCap_ = true;
                    }
                }
            } else
            {
                if (addMeasurement(data))
                    applyMeasurement(data->timestamp_);
            }
        }
    }

}
