/**
 * @file multiSensorsFusion.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief realize multi-sensors fusion based on error state extended kalman filter
 * use the imu data to predict the states and other sensors or modules are used to update
 * @version 0.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "multiSensorsFusion.h"

#include <utility>

namespace multiSensorFusion
{

/**
 * @brief Construct a new MSF::MSF object(Default constructor) which must be configd later.
 */
MSF::MSF()
{
    state_.pos_.setZero();
    state_.vel_.setZero();
    state_.q_.setIdentity();
    state_.ba_.setZero();
    // state_.ba_ << 0.2, -0.1, -0.1;
    state_.bw_.setZero();
    state_.imuAcc.setZero();
    state_.imuGyro.setZero();
    vio_p_map_.setZero();
    vio_q_map_.setIdentity();
    lasttime_ = 0.0;
    isInitialized_ = isWithMap_ = false;
    n_a_ = 1.8032560209070164e-02;
    n_w_ = 2.1483047214801627e-03;
    n_ba_ = 4.9098209065407837e-04;
    n_bw_ = 1.5036495212638137e-05;
    state_.gravity_ << 0.0, 0.0, -9.8;
}

/**
 * @brief Construct a new MSF::MSF object with basic parameters.
 * Only be used for imu-VIO-map_based localization fusion.
 * @param gravity 
 * @param n_a 
 * @param n_w 
 * @param n_ba 
 * @param n_bw 
 * @param n_p_map 
 * @param n_q_map 
 */
MSF::MSF(Vector3d gravity, double n_a, double n_w, double n_ba, 
            double n_bw, double n_p_map, double n_q_map) : 
            isInitialized_(false), isWithMap_(false), n_a_(n_a), n_w_(n_w), 
            n_ba_(n_ba), n_bw_(n_bw), n_p_map_(n_p_map), n_q_map_(n_q_map)
{
    state_.pos_.setZero();
    state_.vel_.setZero();
    state_.q_.setIdentity();
    state_.ba_.setZero();
    state_.bw_.setZero();
    state_.imuAcc.setZero();
    state_.imuGyro.setZero();
    state_.gravity_ = std::move(gravity);
    vio_p_map_.setZero();
    vio_q_map_.setIdentity();
    lasttime_ = 0.0;
}

// TODO: use the config to get basic parameters
void MSF::configFromYaml()
{

}

/**
 * @brief If the MSF finished the initialization, return true; Otherwise, return false.  
 * @return true or false 
 */
bool MSF::isInitialized() const
{
    return isInitialized_;
}

/**
 * @brief If the MSF has get the map pose, return true; Otherwise, return false.
 * @return true or false  
 */
bool MSF::isWithMap() const
{
    return isWithMap_;
}

/**
 * @brief Input imu data to MSF.
 * Note: the imu data must the data in the VIO's baslink coordinate system.
 * @param timestamp 
 * @param imuAccRaw 
 * @param imuGyroRaw 
 */
void MSF::inputIMU(const double &timestamp, const Vector3d &imuAccRaw, const Vector3d &imuGyroRaw)
{
    if(!isInitialized())
    {
        state_.imuAcc = imuAccRaw;
        state_.imuGyro = imuGyroRaw;

        lasttime_ = timestamp;

        cout << "waiting for VIO pose message to initialize the filter..." << endl;
    }
    else
    {
        // cout << state_.ba_.transpose() << " " << state_.gravity_.transpose() << endl;

        // double dt = timestamp - lasttime_;
        // lasttime_ = timestamp;

        // Vector3d acc = imuAccRaw - state_.ba_;
        // Vector3d gyro = imuGyroRaw - state_.bw_;
        // Vector3d theta = gyro * dt;

        // // predict the states
        // // attitude
        // Quaterniond dq = getQuaternionFromAngle(theta);
        // state_.q_ *= dq;
        // // position and velocity
        // Matrix3d R = state_.q_.toRotationMatrix();
        // Vector3d linearAcc = R * acc + state_.gravity_;
        // state_.pos_ += (state_.vel_ * dt + 0.5 * linearAcc * dt * dt);
        // state_.vel_ += linearAcc * dt;

        double dt = timestamp - lasttime_;
        lasttime_ = timestamp;

        vioMapFusionStates last_state_ = state_;

        state_.imuAcc = imuAccRaw;
        state_.imuGyro = imuGyroRaw;

        Vector3d acc = 0.5 * (last_state_.imuAcc + imuAccRaw) - last_state_.ba_;
        Vector3d gyro = 0.5 * (last_state_.imuGyro + imuGyroRaw)  - last_state_.bw_;
        Vector3d theta = gyro * dt;

        // predict the states
        // attitude
        Quaterniond dq = getQuaternionFromAngle(theta);
        state_.q_ = last_state_.q_ * dq;
        Matrix3d R = state_.q_.toRotationMatrix();
        // velocity
        Vector3d deltaVel = (state_.q_ * acc + state_.gravity_) * dt;
        state_.vel_ = last_state_.vel_ + deltaVel;
        // position
        state_.pos_ = last_state_.pos_ + 0.5 * (last_state_.vel_ + state_.vel_) * dt;

        // TODO: consider use the diff to calculate the jacobi 
        // <<Quaternion kinematics for the error-state Kalman filter>>
        // propagate the covariance
        // Matrix F
        MatrixXd F = MatrixXd::Identity(nStates_,nStates_);
        F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;
        F.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;
        F.block<3, 3>(3, 9) = -R * dt;
        F.block<3, 3>(3, 15) = Matrix3d::Identity() * dt;
        F.block<3, 3>(6, 6) = Matrix3d::Identity() - skew_symmetric(gyro) * dt;
        // F.block<3, 3>(6, 6) = R.transpose() * skew_symmetric(gyro) * dt;
        F.block<3, 3>(6, 12) = -Matrix3d::Identity() * dt;
        // Matrix Q
        MatrixXd Fi = MatrixXd::Zero(nStates_, 12);
        Fi.block<12, 12>(3, 0) = MatrixXd::Identity(12, 12);
        MatrixXd Qi = MatrixXd::Zero(12, 12);
        // Qi.block<3, 3>(0, 0) = Matrix3d::Identity() * n_a_ * n_a_ * dt * dt;
        // Qi.block<3, 3>(3, 3) = Matrix3d::Identity() * n_w_ * n_w_ * dt * dt;
        Qi.block<3, 3>(0, 0) = Matrix3d::Identity() * n_a_ * n_a_ * dt;
        Qi.block<3, 3>(3, 3) = Matrix3d::Identity() * n_w_ * n_w_ * dt;
        // TODO: still have some confusion on random walk parameters
        // Qi.block<3, 3>(6, 6) = Matrix3d::Identity() * n_ba_ * n_ba_ * dt;
        // Qi.block<3, 3>(9, 9) = Matrix3d::Identity() * n_bw_ * n_bw_ * dt;
        Qi.block<3, 3>(6, 6) = Matrix3d::Identity() * n_ba_ * n_ba_;
        Qi.block<3, 3>(9, 9) = Matrix3d::Identity() * n_bw_ * n_bw_;
        MatrixXd Q = Fi * Qi * Fi.transpose();
        pose_kf_.predict(F, Q);
    }
}

/**
 * @brief Input VIO pose data to MSF.
 * @param timestamp 
 * @param vioPosRaw 
 * @param vioRotRaw 
 * @param vioCovariance 
 */
void MSF::inputVIO(const double &timestamp, const Vector3d &vioPosRaw, const Vector3d &vioVelRaw, const Quaterniond &vioRotRaw, const Matrix<double, 9, 9> &vioCovariance)
{
    if(!isInitialized())
    {
        state_.pos_ = vioPosRaw;
        state_.vel_ = vioVelRaw;
        state_.q_ = vioRotRaw;

        MatrixXd P = MatrixXd::Zero(nStates_, nStates_);
        // P.block<9, 9>(0, 0) = vioCovariance;
        P.block<6, 6>(0, 0) = Matrix<double, 6, 6>::Identity();
        P.block<3, 3>(6, 6) = Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
        P.block<3, 3>(9, 9) = Matrix3d::Identity() * 0.1 * 0.1;
        P.block<3, 3>(12, 12) = Matrix3d::Identity() * 0.01 * 0.01;
        P.block<3, 3>(15, 15) = Matrix3d::Identity() * 0.01 * 0.01;
        pose_kf_ = baseKF(nStates_);
        pose_kf_.init(P);

//        lasttime_ = timestamp;
        isInitialized_ = true;
    }
    else
    {
        // residuals
        VectorXd dz = VectorXd::Zero(9);
        dz.segment(0, 3) = vioPosRaw - state_.pos_;
        dz.segment(3, 3) = vioVelRaw - state_.vel_;
        Quaterniond dq = state_.q_.conjugate() * vioRotRaw;
        // dz.segment(6, 3) = dq.vec() / dq.w() * 2.0;
        AngleAxisd dtheta(dq);
        dz.segment(6, 3) = dtheta.axis() * dtheta.angle();

        // update the covariance and the error state
        // Matrix H
        MatrixXd H = MatrixXd::Zero(9, nStates_);
        H.block<9, 9>(0, 0) = Matrix<double, 9, 9>::Identity();
        // Matrix R
        MatrixXd R = vioCovariance;
        // MatrixXd R = MatrixXd::Zero(9, 9);
        // R.block<3, 3>(0, 0) = Matrix3d::Identity() * 0.1 * 0.1;
        // R.block<3, 3>(3, 3) = vioCovariance.block<3, 3>(3, 3);
        // R.block<3, 3>(6, 6) = Matrix3d::Identity() * 2.0 * degreeToRadian * 2.0 * degreeToRadian;
        pose_kf_.update(dz, H, R);

        // update the states
        updateStates();

        // ESKF reset
        MatrixXd G = MatrixXd::Identity(nStates_, nStates_);
        VectorXd dx = pose_kf_.getDeltaStates();
        G.block<3, 3>(6, 6) -= skew_symmetric(0.5 * dx.segment(6, 3));
        pose_kf_.resetESKF(G);
    }
}

/**
 * @brief Input map-based localization pose data to MSF.
 * Note: the pose data must the pose of the VIO's baselink in map's coordinate system.
 * @param timestamp 
 * @param mapPosRaw 
 * @param mapRotRaw 
 */
void MSF::inputMapPose(const double &timestamp, const Vector3d &mapPosRaw, const Quaterniond &mapRotRaw)
{
    if(!isInitialized())
        cout << "waiting for VIO pose message to initialize the filter..." << endl;
    else
    {
        if(!isWithMap())
        {
            if(abs(timestamp - lasttime_) > 0.1)
            {
                cout << "Map pose and IMU's timestamps are not synchronized!" << endl;
                return;
            }

            Isometry3d map_T_i = Isometry3d::Identity(), vio_T_i = Isometry3d::Identity();
            map_T_i.rotate(mapRotRaw);
            map_T_i.pretranslate(mapPosRaw);
            vio_T_i.rotate(state_.q_);
            vio_T_i.pretranslate(state_.pos_);
            Isometry3d vio_T_map = vio_T_i * map_T_i.inverse();
            vio_p_map_ = vio_T_map.translation();
            vio_q_map_ = vio_T_map.linear();

            MatrixXd P_VioTMap = MatrixXd::Zero(6, 6);
            P_VioTMap.block<3, 3>(0, 0) = Matrix3d::Identity();
            P_VioTMap.block<3, 3>(3, 3) = Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
            VioTMap_kf_ = baseKF(6);
            VioTMap_kf_.init(P_VioTMap);

            isWithMap_ = true;

            cout << "Map pose got!" << endl;
        }
        else
        {
            // ********************update the transformation from VIO's coordinate system to map's****
            //  residuals
            VectorXd dz_VioTMap = VectorXd::Zero(6);
            dz_VioTMap.segment(0, 3) = state_.pos_ - getMaptoVIOT() * mapPosRaw;
            Quaterniond dq_VioTMap = (vio_q_map_ * mapRotRaw).conjugate() * state_.q_;
            AngleAxisd dtheta_VioTMap(dq_VioTMap);
            dz_VioTMap.segment(3, 3) = dtheta_VioTMap.axis() * dtheta_VioTMap.angle();

            // update the covariance and the error state
            // Matrix H
            MatrixXd H_VioTMap = MatrixXd::Zero(6, 6);
            H_VioTMap.block<3, 3>(0, 0) = Matrix3d::Identity();
            H_VioTMap.block<3, 3>(3, 3) = -mapRotRaw.toRotationMatrix().transpose();
            // Matrix R
            MatrixXd R_VioTMap = MatrixXd::Zero(6, 6);
            R_VioTMap.block<3, 3>(0, 0) = Matrix3d::Identity() * 0.2 * 0.2;
            // R_VioTMap.block<3, 3>(0, 0) = pose_kf_.getUncertainty().block<3, 3>(0, 0);
            R_VioTMap.block<3, 3>(3, 3) = Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
            // R_VioTMap.block<3, 3>(3, 3) = pose_kf_.getUncertainty().block<3, 3>(6, 6);
            VioTMap_kf_.update(dz_VioTMap, H_VioTMap, R_VioTMap);

            // update the states
            updateMaptoVIOT();

            // ESKF reset
            MatrixXd G_VioTMap = MatrixXd::Identity(6, 6);
            VectorXd dx_VioTMap = VioTMap_kf_.getDeltaStates();
            G_VioTMap.block<3, 3>(3, 3) -= skew_symmetric(0.5 * dx_VioTMap.segment(3, 3));
            VioTMap_kf_.resetESKF(G_VioTMap);

            // ********************update the VIO and Map fusion state********************************

            // residuals
            VectorXd dz = VectorXd::Zero(6);
            dz.segment(0, 3) = mapPosRaw - getVIOtoMapT() * state_.pos_;
            Quaterniond dq = (vio_q_map_.conjugate() * state_.q_).conjugate() * mapRotRaw;
            AngleAxisd dtheta(dq);
            dz.segment(3, 3) = dtheta.axis() * dtheta.angle();

            // update the covariance and the error state
            // Matrix H
            MatrixXd H = MatrixXd::Zero(6, nStates_);
            H.block<3, 3>(0, 0) = getVIOtoMapT().linear();
            H.block<3, 3>(3, 6) = Matrix3d::Identity();
            // Matrix R
            MatrixXd R = MatrixXd::Zero(6, 6);
            R.block<3, 3>(0, 0) = Matrix3d::Identity() * 0.2 * 0.2;
            R.block<3, 3>(3, 3) = Matrix3d::Identity() * 5.0 * degreeToRadian * 5.0 * degreeToRadian;
            pose_kf_.update(dz, H, R);

            // update the states
            updateStates();

            // ESKF reset
            MatrixXd G = MatrixXd::Identity(nStates_, nStates_);
            VectorXd dx = pose_kf_.getDeltaStates();
            G.block<3, 3>(6, 6) -= skew_symmetric(0.5 * dx.segment(6, 3));
            pose_kf_.resetESKF(G);          
        }
    }
}

/**
 * @brief Update the states of MSF.
 */
void MSF::updateStates()
{
    VectorXd dx = pose_kf_.getDeltaStates();
    state_.pos_ += dx.segment(0, 3);
    state_.vel_ += dx.segment(3, 3);
    Quaterniond dq = getQuaternionFromAngle(dx.segment(6, 3));
    state_.q_ *= dq;
    state_.ba_ += dx.segment(9, 3);
    state_.bw_ += dx.segment(12, 3);
    state_.gravity_ += dx.segment(15, 3);
}

/**
 * @brief Update the the transformation from VIO's coordinate system to map's coordinate system.
 */
void MSF::updateMaptoVIOT()
{
    VectorXd dx = VioTMap_kf_.getDeltaStates();
    vio_p_map_ += dx.segment(0, 3);
    Quaterniond dq = getQuaternionFromAngle(dx.segment(3, 3));
    vio_q_map_ *= dq;
}

/**
 * @brief Return the transformation from VIO's coordinate system to map's coordinate system.
 * @return Eigen::Isometry3d map_T_vio
 */
Isometry3d MSF::getVIOtoMapT()
{
    return getMaptoVIOT().inverse();
}

/**
 * @brief Return the transformation from map's coordinate system to VIO's coordinate system.
 * @return Eigen::Isometry3d vio_T_map
 */
Isometry3d MSF::getMaptoVIOT()
{
    Isometry3d T;
    T.linear() = vio_q_map_.toRotationMatrix();
    T.pretranslate(vio_p_map_);
    return T;
}

/**
 * @brief Return the position in VIO's coordinate system.
 * @return Eigen::Vector3d position
 */
Vector3d MSF::getPosition() const
{
    return state_.pos_;
}

/**
 * @brief Return the velocity in VIO's coordinate system.
 * @return Eigen::Vector3d velocity
 */
Vector3d MSF::getVelocity() const
{
    return state_.vel_;
}

/**
 * @brief Return the orientation in VIO's coordinate system.
 * @return Eigen::Quaterniond orientation
 */
Quaterniond MSF::getRotation() const
{
    return state_.q_;
}

/**
 * @brief Return the position in map's coordinate system.
 * @return Eigen::Vector3d position
 */
Vector3d MSF::getPositionInMapFrame()
{
    return getVIOtoMapT() * state_.pos_;
}

/**
 * @brief Return the velocity in map's coordinate system.
 * @return Eigen::Vector3d velocity
 */
Vector3d MSF::getVelocityInMapFrame()
{
    return getVIOtoMapT().linear() * state_.vel_;
}

/**
 * @brief Return the orientation in map's coordinate system.
 * @return Eigen::Quaterniond orientation
 */
Quaterniond MSF::getRotationInMapFrame()
{
    return vio_q_map_.conjugate() * state_.q_;
}

/**
 * @brief Return the time of filer
 * @return double time
 */
double MSF::getCurrentTime() const
{
    return lasttime_;
}

} // namespace multiSensorFusion