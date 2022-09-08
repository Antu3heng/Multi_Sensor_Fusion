//
// Created by antusheng on 9/4/22.
//

#ifndef MULTI_SENSOR_FUSION_MSF_GPS_PROCESSOR_H
#define MULTI_SENSOR_FUSION_MSF_GPS_PROCESSOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "msf_utils.h"
#include "msf_type.h"

namespace MSF
{
    class msf_gps_processor
    {
    public:
        msf_gps_processor() = delete;

        msf_gps_processor(Eigen::Vector3d body_p_sensor, const Eigen::Quaterniond &body_q_sensor);

        ~msf_gps_processor() = default;

        void fixNoise(double n_pos);

        void setInitTransformation(const baseStatePtr &state, const gpsDataPtr &data);

        void updateTransformation(baseStatePtr &currentState, const gpsDataPtr &data);

        void updateState(baseStatePtr &currentState, const gpsDataPtr &data);

        void update(baseStatePtr &currentState, const gpsDataPtr &data);

        void transformStateToGlobal(baseStatePtr &state);

        bool has_init_transformation_ = false;

    private:
        // the transformation from position sensor's body coordinate system to MSF's body coordinate system
        Eigen::Vector3d body_p_sensor_;
        Eigen::Quaterniond body_q_sensor_;
        Eigen::Matrix<double, 6, 6> cov_for_T_bs_;
        // the transformation from position sensor's world coordinate system to MSF's world coordinate system
        Eigen::Vector3d local_p_global_;
        Eigen::Quaterniond local_q_global_;
        Eigen::Matrix<double, 6, 6> cov_for_T_lg_;

        Eigen::Vector3d init_lla_;

        bool is_use_fixed_noise_ = false;

        // position noise parameter
        // unit: n_pos_ - m
        double n_pos_{};
    };
}

#endif //MULTI_SENSOR_FUSION_MSF_GPS_PROCESSOR_H
