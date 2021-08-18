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
            : initialized_(false)
    {
        initializer_ = std::make_shared<msf_initializer>();
        imuProcessor_ = std::make_shared<msf_imu_processor>();

    }

    void msf_core::inputIMU(const imuData &data)
    {

    }

    void msf_core::inputVIO(const vioData &data)
    {

    }

    void msf_core::inputMapLoc(const mapLocData &data)
    {

    }

    baseState msf_core::outputState() const
    {
        return baseState();
    }

    void msf_core::applyMeasurement()
    {

    }
}