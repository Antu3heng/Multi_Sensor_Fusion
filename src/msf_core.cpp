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

namespace MSF
{
    msf_core::msf_core(const std::string &config_file_path)
    {
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(config_file_path);
        } catch (YAML::BadFile &e)
        {
            std::cerr << "Wrong path to config file!" << std::endl;
            exit(-1);
        }

        if (!config["imu"])
        {
            std::cerr << "Need to provide imu parameters!" << std::endl;
            exit(1);
        }

        auto vec_body_q_imu = config["imu"]["body_q_imu"].as<std::vector<double>>();
        body_q_imu_ = Eigen::Quaterniond(vec_body_q_imu[3], vec_body_q_imu[0], vec_body_q_imu[1], vec_body_q_imu[2]);

        auto n_a = config["imu"]["acc_noise_density"].as<double>();
        auto n_ba = config["imu"]["acc_random_walk"].as<double>();
        auto n_w = config["imu"]["gyr_noise_density"].as<double>();
        auto n_bw = config["imu"]["gyr_random_walk"].as<double>();
        imuProcessor_ = std::make_shared<msf_imu_processor>(n_a, n_w, n_ba, n_bw);

        auto initializer_buffer_size = config["imu"]["initializer_buffer_size"].as<int>();
        auto initializer_max_acc_std = config["imu"]["initializer_max_acc_std"].as<double>();
        initializer_ = std::make_shared<msf_initializer>(initializer_buffer_size, initializer_max_acc_std);

        if (config["pos_sensor"])
        {
            int num_pos_sensor = config["pos_sensor"]["quantity"].as<int>();
            for (int i = 1; i <= num_pos_sensor; i++)
            {
                std::string sensor_idx = "sensor" + std::to_string(i);

                auto vec_body_T_sensor = config["pos_sensor"][sensor_idx]["body_T_sensor"].as<std::vector<double>>();
                auto body_p_sensor = Eigen::Vector3d(vec_body_T_sensor[0], vec_body_T_sensor[1], vec_body_T_sensor[2]);
                auto body_q_sensor = Eigen::Quaterniond(vec_body_T_sensor[6], vec_body_T_sensor[3],
                                                        vec_body_T_sensor[4], vec_body_T_sensor[5]);
                if (!config["pos_sensor"][sensor_idx]["local_T_global"])
                {
                    std::cerr
                            << "Need to provide the transformation from position sensor's world coordinate system to MSF's world coordinate system!"
                            << std::endl;
                    exit(1);
                }
                auto vec_local_T_global = config["pos_sensor"][sensor_idx]["local_T_global"].as<std::vector<double>>();
                auto local_p_global = Eigen::Vector3d(vec_local_T_global[0], vec_local_T_global[1],
                                                      vec_local_T_global[2]);
                auto local_q_global = Eigen::Quaterniond(vec_local_T_global[6], vec_local_T_global[3],
                                                         vec_local_T_global[4], vec_local_T_global[5]);

                auto pos_processor = std::make_shared<msf_pos_processor>(body_p_sensor, body_q_sensor, local_p_global,local_q_global);
                if (!config["pos_sensor"][sensor_idx]["input_with_cov"].as<int>())
                {
                    auto n_pos = config["pos_sensor"][sensor_idx]["n_pos"].as<double>();
                    pos_processor->fixNoise(n_pos);
                }
                std::string sensor_name = config["pos_sensor"][sensor_idx]["name"].as<std::string>();
                posProcessors_[sensor_name] = pos_processor;

                if (config["pos_sensor"][sensor_idx]["use_for_initial"].as<int>())
                {
                    initial_sensor_name_ = sensor_name;
                }

                if (config["pos_sensor"][sensor_idx]["use_for_global"].as<int>())
                {
                    if (!global_sensor_name_.empty())
                    {
                        std::cerr << "More than one sensor used for global!" << std::endl;
                        exit(-1);
                    }
                    global_sensor_type_ = Pos;
                    global_sensor_name_ = sensor_name;
                    is_with_map_ = true;
                }
            }
        }

        if (config["pose_sensor"])
        {
            int num_pose_sensor = config["pose_sensor"]["quantity"].as<int>();
            for (int i = 1; i <= num_pose_sensor; i++)
            {
                std::string sensor_idx = "sensor" + std::to_string(i);

                std::shared_ptr<msf_pose_processor> pose_processor;

                auto vec_body_T_sensor = config["pose_sensor"][sensor_idx]["body_T_sensor"].as<std::vector<double>>();
                auto body_p_sensor = Eigen::Vector3d(vec_body_T_sensor[0], vec_body_T_sensor[1], vec_body_T_sensor[2]);
                auto body_q_sensor = Eigen::Quaterniond(vec_body_T_sensor[6], vec_body_T_sensor[3],
                                                        vec_body_T_sensor[4], vec_body_T_sensor[5]);
                bool get_transformation = false;
                if (config["pose_sensor"][sensor_idx]["local_T_global"])
                {
                    get_transformation = true;
                    auto vec_local_T_global = config["pose_sensor"][sensor_idx]["local_T_global"].as<std::vector<double>>();
                    auto local_p_global = Eigen::Vector3d(vec_local_T_global[0], vec_local_T_global[1],
                                                          vec_local_T_global[2]);
                    auto local_q_global = Eigen::Quaterniond(vec_local_T_global[6], vec_local_T_global[3],
                                                             vec_local_T_global[4], vec_local_T_global[5]);
                    pose_processor = std::make_shared<msf_pose_processor>(body_p_sensor, body_q_sensor, local_p_global, local_q_global);
                } else
                    pose_processor = std::make_shared<msf_pose_processor>(body_p_sensor, body_q_sensor);
                if (!config["pose_sensor"][sensor_idx]["input_with_cov"].as<int>())
                {
                    auto n_pos = config["pose_sensor"][sensor_idx]["n_pos"].as<double>();
                    auto n_q = config["pose_sensor"][sensor_idx]["n_q"].as<double>();
                    pose_processor->fixNoise(n_pos, n_q);
                }
                std::string sensor_name = config["pose_sensor"][sensor_idx]["name"].as<std::string>();
                poseProcessors_[sensor_name] = pose_processor;

                if (config["pose_sensor"][sensor_idx]["use_for_initial"].as<int>())
                {
                    initial_sensor_name_ = sensor_name;
                }

                if (config["pose_sensor"][sensor_idx]["use_for_global"].as<int>())
                {
                    if (!global_sensor_name_.empty())
                    {
                        std::cerr << "More than one sensor used for global!" << std::endl;
                        exit(-1);
                    }
                    global_sensor_type_ = Pose;
                    global_sensor_name_ = sensor_name;
                    if (get_transformation)
                        is_with_map_ = true;
                }
            }
        }

        if (config["odom_sensor"])
        {
            int num_odom_sensor = config["odom_sensor"]["quantity"].as<int>();
            for (int i = 1; i <= num_odom_sensor; i++)
            {
                std::string sensor_idx = "sensor" + std::to_string(i);

                std::shared_ptr<msf_odom_processor> odom_processor;

                auto vec_body_T_sensor = config["odom_sensor"][sensor_idx]["body_T_sensor"].as<std::vector<double>>();
                auto body_p_sensor = Eigen::Vector3d(vec_body_T_sensor[0], vec_body_T_sensor[1], vec_body_T_sensor[2]);
                auto body_q_sensor = Eigen::Quaterniond(vec_body_T_sensor[6], vec_body_T_sensor[3],
                                                        vec_body_T_sensor[4], vec_body_T_sensor[5]);
                bool get_transformation = false;
                if (config["odom_sensor"][sensor_idx]["local_T_global"])
                {
                    get_transformation = true;
                    auto vec_local_T_global = config["odom_sensor"][sensor_idx]["local_T_global"].as<std::vector<double>>();
                    auto local_p_global = Eigen::Vector3d(vec_local_T_global[0], vec_local_T_global[1],
                                                          vec_local_T_global[2]);
                    auto local_q_global = Eigen::Quaterniond(vec_local_T_global[6], vec_local_T_global[3],
                                                             vec_local_T_global[4], vec_local_T_global[5]);
                    odom_processor = std::make_shared<msf_odom_processor>(body_p_sensor, body_q_sensor, local_p_global, local_q_global);
                } else
                    odom_processor = std::make_shared<msf_odom_processor>(body_p_sensor, body_q_sensor);
                if (!config["odom_sensor"][sensor_idx]["input_with_cov"].as<int>())
                {
                    auto n_pos = config["odom_sensor"][sensor_idx]["n_pos"].as<double>();
                    auto n_q = config["odom_sensor"][sensor_idx]["n_q"].as<double>();
                    auto  n_v = config["odom_sensor"][sensor_idx]["n_v"].as<double>();
                    odom_processor->fixNoise(n_pos, n_q, n_v);
                }
                std::string sensor_name = config["odom_sensor"][sensor_idx]["name"].as<std::string>();
                odomProcessors_[sensor_name] = odom_processor;

                if (config["odom_sensor"][sensor_idx]["use_for_initial"].as<int>())
                {
                    initial_sensor_name_ = sensor_name;
                }

                if (config["odom_sensor"][sensor_idx]["use_for_global"].as<int>())
                {
                    if (!global_sensor_name_.empty())
                    {
                        std::cerr << "More than one sensor used for global!" << std::endl;
                        exit(-1);
                    }
                    global_sensor_type_ = Odom;
                    global_sensor_name_ = sensor_name;
                    if (get_transformation)
                        is_with_map_ = true;
                }
            }
        }

        if (config["gps_sensor"])
        {
            int num_gps_sensor = config["gps_sensor"]["quantity"].as<int>();
            for (int i = 1; i <= num_gps_sensor; i++)
            {
                std::string sensor_idx = "sensor" + std::to_string(i);

                auto vec_body_T_sensor = config["gps_sensor"][sensor_idx]["body_T_sensor"].as<std::vector<double>>();
                auto body_p_sensor = Eigen::Vector3d(vec_body_T_sensor[0], vec_body_T_sensor[1], vec_body_T_sensor[2]);
                auto body_q_sensor = Eigen::Quaterniond(vec_body_T_sensor[6], vec_body_T_sensor[3],
                                                        vec_body_T_sensor[4], vec_body_T_sensor[5]);

                auto gps_processor = std::make_shared<msf_gps_processor>(body_p_sensor, body_q_sensor);
                if (!config["gps_sensor"][sensor_idx]["input_with_cov"].as<int>())
                {
                    auto n_pos = config["gps_sensor"][sensor_idx]["n_pos"].as<double>();
                    gps_processor->fixNoise(n_pos);
                }
                std::string sensor_name = config["gps_sensor"][sensor_idx]["name"].as<std::string>();
                gpsProcessors_[sensor_name] = gps_processor;

                if (config["gps_sensor"][sensor_idx]["use_for_initial"].as<int>())
                {
                    initial_sensor_name_ = sensor_name;
                }

                if (config["gps_sensor"][sensor_idx]["use_for_global"].as<int>())
                {
                    if (!global_sensor_name_.empty())
                    {
                        std::cerr << "More than one sensor used for global!" << std::endl;
                        exit(-1);
                    }
                    global_sensor_type_ = GPS;
                    global_sensor_name_ = sensor_name;
                    is_with_map_ = true;
                }
            }
        }

        current_state_ = std::make_shared<baseState>();
    }

    bool msf_core::isInitialized() const
    {
        return initialized_;
    }

    void msf_core::inputIMU(const imuDataPtr &data)
    {
        data->acc_ = body_q_imu_ * data->acc_;
        data->gyro_ = body_q_imu_ * data->gyro_;

        if (!isInitialized())
        {
            initializer_->addIMU(data);
        } else
        {
            current_state_ = std::make_shared<baseState>();
            current_state_->timestamp_ = data->timestamp_;
            current_state_->imu_data_ = data;
            imuProcessor_->predict((--(state_buffer_.end()))->second, current_state_);
            state_buffer_.insert(std::pair<double, baseStatePtr>(current_state_->timestamp_, current_state_));
            checkFutureMeasurement();
            pruneBuffer();
        }
    }

    void msf_core::inputPos(const posDataPtr &data)
    {
        if (isInitialized())
        {
            if (addMeasurement(data))
                applyMeasurement(data->timestamp_);
        }
    }

    void msf_core::inputPose(const poseDataPtr &data)
    {
        if (!isInitialized())
        {
            if (initial_sensor_name_ == data->name_ && initializer_->initialize(data, current_state_))
            {
                std::cerr << "[msf_core]: The MSF has been initialized!" << std::endl;
                initialized_ = true;
                state_buffer_.insert(std::pair<double, baseStatePtr>(current_state_->timestamp_, current_state_));
                if (!poseProcessors_[data->name_]->has_init_transformation_)
                {
                    poseProcessors_[data->name_]->setInitTransformation(current_state_, data);
                    if (global_sensor_name_ == data->name_)
                        is_with_map_ = true;
                }
            }
        } else
        {
            if (poseProcessors_[data->name_]->has_init_transformation_)
            {
                if (addMeasurement(data))
                    applyMeasurement(data->timestamp_);
            } else
            {
                auto it = state_buffer_.lower_bound(data->timestamp_ - max_time_interval_);
                if (it != state_buffer_.end())
                {
                    if (fabs(it->first - data->timestamp_) <= max_time_interval_)
                    {
                        poseProcessors_[data->name_]->setInitTransformation(it->second, data);
                        if (global_sensor_name_ == data->name_)
                            is_with_map_ = true;
                    }
#ifdef TEST_DEBUG
                    else
                        std::cerr
                                << "[msf_core]: " << data->name_ << " and IMU's timestamps are not synchronized!"
                                << std::endl;
#endif
                }
            }
        }
    }

    void msf_core::inputOdom(const odomDataPtr &data)
    {
        if (!isInitialized())
        {
            if (initial_sensor_name_ == data->name_ && initializer_->initialize(data, current_state_))
            {
                std::cerr << "[msf_core]: The MSF has been initialized!" << std::endl;
                initialized_ = true;
                state_buffer_.insert(std::pair<double, baseStatePtr>(current_state_->timestamp_, current_state_));
                if (!odomProcessors_[data->name_]->has_init_transformation_)
                {
                    odomProcessors_[data->name_]->setInitTransformation(current_state_, data);
                    if (global_sensor_name_ == data->name_)
                        is_with_map_ = true;
                }
            }
        } else
        {
            if (odomProcessors_[data->name_]->has_init_transformation_)
            {
                if (addMeasurement(data))
                    applyMeasurement(data->timestamp_);
            } else
            {
                auto it = state_buffer_.lower_bound(data->timestamp_ - max_time_interval_);
                if (it != state_buffer_.end())
                {
                    if (fabs(it->first - data->timestamp_) <= max_time_interval_)
                    {
                        odomProcessors_[data->name_]->setInitTransformation(it->second, data);
                        if (global_sensor_name_ == data->name_)
                            is_with_map_ = true;
                    }
#ifdef TEST_DEBUG
                    else
                        std::cerr
                                << "[msf_core]: " << data->name_ << " and IMU's timestamps are not synchronized!"
                                << std::endl;
#endif
                }
            }
        }
    }

    void msf_core::inputGPS(const gpsDataPtr &data)
    {
        if (!isInitialized())
        {
            if (initial_sensor_name_ == data->name_ && initializer_->initialize(data, current_state_))
            {
                std::cerr << "[msf_core]: The MSF has been initialized!" << std::endl;
                initialized_ = true;
                state_buffer_.insert(std::pair<double, baseStatePtr>(current_state_->timestamp_, current_state_));
                if (!gpsProcessors_[data->name_]->has_init_transformation_)
                {
                    gpsProcessors_[data->name_]->setInitTransformation(current_state_, data);
                    if (global_sensor_name_ == data->name_)
                        is_with_map_ = true;
                }
            }
        } else
        {
            if (gpsProcessors_[data->name_]->has_init_transformation_)
            {
                if (addMeasurement(data))
                    applyMeasurement(data->timestamp_);
            } else
            {
                auto it = state_buffer_.lower_bound(data->timestamp_ - max_time_interval_);
                if (it != state_buffer_.end())
                {
                    if (fabs(it->first - data->timestamp_) <= max_time_interval_)
                    {
                        gpsProcessors_[data->name_]->setInitTransformation(it->second, data);
                        if (global_sensor_name_ == data->name_)
                            is_with_map_ = true;
                    }
                }
            }
        }
    }

    baseState msf_core::outputCurrentState()
    {
        current_state_ = (--state_buffer_.end())->second;
        if (is_with_map_)
        {
            switch (global_sensor_type_)
            {
                case Pos:
                {
                    posProcessors_[global_sensor_name_]->transformStateToGlobal(current_state_);
                    break;
                }
                case Pose:
                {
                    poseProcessors_[global_sensor_name_]->transformStateToGlobal(current_state_);
                    break;
                }
                case Odom:
                {
                    odomProcessors_[global_sensor_name_]->transformStateToGlobal(current_state_);
                    break;
                }
                case GPS:
                {
                    gpsProcessors_[global_sensor_name_]->transformStateToGlobal(current_state_);
                    break;
                }
                default:
                    break;
            }
            current_state_->has_global_state_ = true;
        }
        return *current_state_;
    }


    bool msf_core::addMeasurement(const baseDataPtr &data)
    {
        auto it = state_buffer_.lower_bound(data->timestamp_ - max_time_interval_);
        if (it == state_buffer_.end())
        {
#ifdef TEST_DEBUG
            std::cerr << "[msf_core]: The new measurement is forward the states!" << std::endl;
#endif
            futureMeas_buffer_.insert(std::pair<double, baseDataPtr>(data->timestamp_, data));
            return false;
        } else
        {
            if (fabs(it->first - data->timestamp_) <= max_time_interval_)
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
        auto itState = state_buffer_.lower_bound(timestamp - max_time_interval_);
        for (auto itSensor = meas_buffer_.find(timestamp); itSensor != meas_buffer_.end(); ++itSensor)
        {
            while (itState != state_buffer_.lower_bound(itSensor->first - max_time_interval_))
            {
                auto itLastState = itState;
                imuProcessor_->predict(itLastState->second, (++itState)->second);
            }
            switch (itSensor->second->type_)
            {
                case Pos:
                {
                    posProcessors_[itSensor->second->name_]->update(itState->second,
                                                                    std::dynamic_pointer_cast<posData>(
                                                                            itSensor->second));
                    break;
                }
                case Pose:
                {
                    poseProcessors_[itSensor->second->name_]->update(itState->second,
                                                                          std::dynamic_pointer_cast<poseData>(
                                                                                  itSensor->second));
                    break;
                }
                case Odom:
                {
                    odomProcessors_[itSensor->second->name_]->update(itState->second,
                                                                          std::dynamic_pointer_cast<odomData>(
                                                                                  itSensor->second));
                    break;
                }
                case GPS:
                {
                    gpsProcessors_[itSensor->second->name_]->update(itState->second,
                                                                    std::dynamic_pointer_cast<gpsData>(
                                                                            itSensor->second));
                    break;
                }
                default:
                    break;
            }
#ifdef TEST_DEBUG
            std::cerr << "[msf_core]: use " << itSensor->second->name_ << " data update!!!"
                          << std::endl;
#endif
        }
        while (itState != (--state_buffer_.end()))
        {
            auto itLastState = itState;
            imuProcessor_->predict(itLastState->second, (++itState)->second);
        }
    }

    void msf_core::checkFutureMeasurement()
    {
        double timestamp;
        bool has_available_meas = false;
        for (auto it = futureMeas_buffer_.begin(); it != futureMeas_buffer_.end();)
        {
            auto itState = state_buffer_.lower_bound(it->first - max_time_interval_);
            if (itState == state_buffer_.end())
                break;
            else
            {
                if (!has_available_meas)
                {
                    has_available_meas = true;
                    timestamp = it->first;
                }
                if (fabs(it->first - itState->first) <= max_time_interval_)
                    meas_buffer_.insert(*it);
                it = futureMeas_buffer_.erase(it);
            }
        }
        if (has_available_meas)
            applyMeasurement(timestamp);
    }

    void msf_core::pruneBuffer()
    {
        while (state_buffer_.size() > max_buffer_size_)
            state_buffer_.erase(state_buffer_.begin());
        while (meas_buffer_.size() > max_buffer_size_)
            meas_buffer_.erase((meas_buffer_.begin()));
    }
}
