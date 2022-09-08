/**
 * @file multiSensorsFusion.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of multiSensorsFusion.cpp
 * @version 0.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef FUSION_MSF_H_
#define FUSION_MSF_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>
#include <map>
#include "baseKF.h"
#include "msf_utils.h"

using namespace std;
using namespace Eigen;

namespace MSF
{

enum msgType {vioMsg, mapPoseMsg};



struct vioMapFusionStates
{
    double timestamp_;

    // states
    // p, v, q, ba, bw, g
    Vector3d pos_;
    Vector3d vel_;
    Quaterniond q_;
    Vector3d ba_;
    Vector3d bw_;
    Vector3d gravity_;

    Vector3d imuAcc_;
    Vector3d imuGyro_;
};

struct VioOdomMsg
{
    double timestamp_;
    Vector3d vioPosRaw_;
    Vector3d vioVelRaw_;
    Quaterniond vioRotRaw_;
    Matrix<double, 9, 9> vioCovariance_;
};

struct MapPoseMsg
{
    double timestamp_;
    Vector3d mapPosRaw_;
    Quaterniond mapRotRaw_;
};

class MSF
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    MSF();
    MSF(Vector3d gravity, double n_a, double n_w, double n_ba, double n_bw, double n_p_map, double n_q_map);
    ~MSF()= default;

    bool isInitialized() const;
    bool isWithMap() const;

    // TODO: 利用yaml配置文件加载参数？
    void configFromYaml();

    void inputIMU(const double &timestamp, const Vector3d &imuAccRaw, const Vector3d &imuGyroRaw);
    void inputVIO(const double &timestamp, const Vector3d &vioPosRaw, const Vector3d &vioVelRaw, const Quaterniond& vioRotRaw, const Matrix<double, 9, 9>& vioCovariance);
    void inputMapPose(const double &timestamp, const Vector3d &mapPosRaw, const Quaterniond &mapRotRaw);

    double getCurrentTime() const;

    Vector3d getPosition() const;
    Vector3d getVelocity() const;
    Quaterniond getRotation() const;

    Vector3d getPositionInMapFrame();
    Vector3d getVelocityInMapFrame();
    Quaterniond getRotationInMapFrame();

    Isometry3d getVIOtoMapT();
    Isometry3d getMaptoVIOT();

private:
    void updateStates();
    // TODO: use one kalman filter to update the states and extrinsic parameters simultaneously?
    // Current stage: use one kalman filter will reduce versatility for more sensors or the case without map
    void updateMaptoVIOT();

    bool isInitialized_;
    bool isWithMap_;

    double lasttime_;

    queue<VioOdomMsg> bufVioOdomMsg;
    queue<MapPoseMsg> bufMapPoseMsg;

    // states
    vioMapFusionStates state_;
    // transformation from map to VIO's coordinate system
    Vector3d vio_p_map_;
    Quaterniond vio_q_map_;
    // num of states
    const int nStates_ = 18;

    // kalman filter
    baseKF pose_kf_;
    baseKF VioTMap_kf_;

    // basic parameters
    // imu noise parameters
    // unit: n_a_-m/s^2/sqrt(hz) n_w_-rad/s/sqrt(hz) n_ba_-m/s^2*sqrt(hz) n_bw_-rad/s*sqrt(hz)
    double n_a_, n_w_, n_ba_, n_bw_;
    // map-based localization error 
    double n_p_map_, n_q_map_;

    // buffer and its parameters
    map<double, vioMapFusionStates> stateBuffer_;
    map<double, msgType> msgIndexBuffer_;
    map<double, VioOdomMsg> vioBuffer_;
    map<double, MapPoseMsg> mapPoseBuffer_;
    const int stateBufferSize_ = 1000;
    const int msgBufferSize_ = 100;
};

} // namespace multiSensorFusion

#endif //FUSION_MSF_H_