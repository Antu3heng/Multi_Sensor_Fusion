/**
 * @file imuGPSFusion.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of imuGPSFusion.cpp
 * @version 0.1
 * @date 2021-06-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef FUSION_IMUGPS_H_
#define FUSION_IMUGPS_H_

#include <iostream>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "baseKF.h"
#include "msf_utils.h"

using namespace std;
using namespace Eigen;

namespace MSF
{
struct imuGPSFusionStates
{
    // states
    // p, v, q, ba, bw
    Vector3d pos_;
    Vector3d vel_;
    Quaterniond q_;
    Vector3d ba_;
    Vector3d bw_;
};

class imuGPSFusion
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    imuGPSFusion();
    imuGPSFusion(Vector3d gravity, double n_a, double n_w, double n_ba, double n_bw);
    ~imuGPSFusion(){};

    bool isInitialized();

    void inputIMU(double timestamp, Vector3d imuAccRaw, Vector3d imuGyroRaw);
    void inputGPS(double timestamp, Vector3d lla, Matrix<double, 3, 3> Covariance);

    double getCurrentTime();

    Vector3d getPosition();
    Vector3d getVelocity();
    Quaterniond getRotation();

private:
    void updateStates();

    bool isInitialized_;

    const int imuBuffLength_ = 100;
    const double imuAccStdLimit = 3;
    deque<double> imu_t_buff_;
    deque<Vector3d> imu_acc_buff_, imu_gyro_buff_;

    Vector3d init_lla_;

    double lasttime_;

    // states
    imuGPSFusionStates state_;
    // dimension of states
    const int nStates_ = 15;

    // kalman filter
    baseKF kf_;

    // basic parameters
    // imu noise parameters
    // unit: n_a_-m/s^2/sqrt(hz) n_w_-rad/s/sqrt(hz) n_ba_-m/s^2*sqrt(hz) n_bw_-rad/s*sqrt(hz)
    double n_a_, n_w_, n_ba_, n_bw_;
    Vector3d gravity_;
};

} // namespace multiSensorFusion

#endif //FUSION_IMUGPS_H_