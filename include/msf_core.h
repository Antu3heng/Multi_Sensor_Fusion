/**
 * @file msf_core.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of msf_core.cpp
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
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include "msf_utils.h"
#include "msf_type.h"
#include "msf_initializer.h"
#include "msf_imu_processor.h"
#include "msf_odom_processor.h"
#include "msf_pose_processor.h"
#include "msf_pos_processor.h"

namespace multiSensorFusion
{
    class msf_core
    {
    public:
        msf_core() = delete;

        explicit msf_core(const std::string &config_file_path);

        ~msf_core() = default;

        bool isInitialized() const;

        void inputIMU(const imuDataPtr &data);

        void inputPos(const posDataPtr &data);

        void inputPose(const poseDataPtr &data);

        void inputOdom(const odomDataPtr &data);

        baseState outputCurrentState();

    private:
        bool addMeasurement(const baseDataPtr &data);

        void applyMeasurement(const double &timestamp);

        void checkFutureMeasurement();

        void pruneBuffer();

        Eigen::Quaterniond body_q_imu_;

        std::shared_ptr<msf_initializer> initializer_;
        std::shared_ptr<msf_imu_processor> imuProcessor_;
        std::unordered_map<std::string, std::shared_ptr<msf_pos_processor>> posProcessors_;
        std::unordered_map<std::string, std::shared_ptr<msf_pose_processor>> poseProcessors_;
        std::unordered_map<std::string, std::shared_ptr<msf_odom_processor>> odomProcessors_;

        bool initialized_ = false;
        std::string initial_sensor_name_;

        bool is_with_map_ = false;
        dataType global_sensor_type_;
        std::string global_sensor_name_;

        baseStatePtr current_state_;

        const int max_buffer_size_ = 2000;

        std::map<double, baseStatePtr> state_buffer_;

        std::map<double, baseDataPtr> meas_buffer_;
        std::map<double, baseDataPtr> futureMeas_buffer_;
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_CORE_H
