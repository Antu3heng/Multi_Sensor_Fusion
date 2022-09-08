/**
 * @file imuGPSFusion.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief realize imu and gps fusion based on extended kalman filter
 * use the imu data to predict the states and gps are used to update
 * @version 0.1
 * @date 2021-06-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "imuGPSFusion.h"

namespace MSF
{

/**
 * @brief Construct a new imuGPSFusion::imuGPSFusion object(Default constructor) which must be configd later.
 */
    imuGPSFusion::imuGPSFusion()
    {
        state_.pos_.setZero();
        state_.vel_.setZero();
        state_.q_.setIdentity();
        state_.ba_.setZero();
        state_.bw_.setZero();
        init_lla_.setZero();
        lasttime_ = 0.0;
        isInitialized_ = false;
        n_a_ = 1e-2;
        n_w_ = 1e-4;
        n_ba_ = 1e-6;
        n_bw_ = 1e-8;
        gravity_ << 0.0, 0.0, -9.81007;
    }

/**
 * @brief Construct a new imuGPSFusion::imuGPSFusion object with basic parameters.
 * Only be used for imu and gps localization fusion.
 * @param gravity 
 * @param n_a 
 * @param n_w 
 * @param n_ba 
 * @param n_bw 
 */
    imuGPSFusion::imuGPSFusion(Vector3d gravity, double n_a, double n_w, double n_ba, double n_bw) : isInitialized_(
            false),
                                                                                                     gravity_(gravity),
                                                                                                     n_a_(n_a),
                                                                                                     n_w_(n_w),
                                                                                                     n_ba_(n_ba),
                                                                                                     n_bw_(n_bw)
    {
        state_.pos_.setZero();
        state_.vel_.setZero();
        state_.q_.setIdentity();
        state_.ba_.setZero();
        state_.bw_.setZero();
        init_lla_.setZero();
        lasttime_ = 0.0;
    }

/**
 * @brief If the imuGPSFusion finished the initialization, return true; Otherwise, return false.  
 * @return true or false 
 */
    bool imuGPSFusion::isInitialized()
    {
        return isInitialized_;
    }

/**
 * @brief Input imu data to imuGPSFusion.
 * Note: the imu data must the data in the VIO's baslink coordinate system.
 * @param timestamp 
 * @param imuAccRaw 
 * @param imuGyroRaw 
 */
    void imuGPSFusion::inputIMU(double timestamp, Vector3d imuAccRaw, Vector3d imuGyroRaw)
    {
        if (!isInitialized())
        {
            cout << "waiting to initialize the filter..." << endl;

            imu_t_buff_.push_back(timestamp);
            imu_acc_buff_.push_back(imuAccRaw);
            imu_gyro_buff_.push_back(imuGyroRaw);

            if (imu_t_buff_.size() > imuBuffLength_)
            {
                imu_t_buff_.pop_front();
                imu_acc_buff_.pop_front();
                imu_gyro_buff_.pop_front();
            }
        } else
        {
            // normal state

            // double dt = timestamp - lasttime_;
            // lasttime_ = timestamp;

            // imuGPSFusionStates last_state_ = state_;

            // Vector3d acc = imuAccRaw - last_state_.ba_;
            // Vector3d gyro = imuGyroRaw - last_state_.bw_;
            // Vector3d theta = gyro * dt;

            // // predict the states
            // // attitude
            // Quaterniond dq = getQuaternionFromAngle(theta);
            // state_.q_ = last_state_.q_ * dq;
            // // velocity
            // Vector3d deltaVel = (state_.q_ * acc + gravity_) * dt;
            // state_.vel_ = last_state_.vel_ + deltaVel;
            // // position
            // state_.pos_ = last_state_.pos_ + 0.5 * (last_state_.vel_ + state_.vel_) * dt;

            // // propagate the covariance
            // // Matrix F
            // MatrixXd F = MatrixXd::Identity(nStates_,nStates_);
            // F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;
            // Matrix<double, 4, 3> temp1, temp2, temp3, F_temp;
            // temp1 << state_.q_.w(), -state_.q_.z(), state_.q_.y(),
            //             state_.q_.x(), state_.q_.y(), state_.q_.z(),
            //             -state_.q_.y(), state_.q_.x(), state_.q_.w(),
            //             -state_.q_.z(), -state_.q_.w(), state_.q_.x();
            // temp2 << state_.q_.z(), state_.q_.w(), -state_.q_.x(),
            //             state_.q_.y(), -state_.q_.x(), -state_.q_.w(),
            //             state_.q_.x(), state_.q_.y(), state_.q_.z(),
            //             state_.q_.w(), -state_.q_.z(), state_.q_.y();
            // temp3 << -state_.q_.y(), state_.q_.x(), state_.q_.w(),
            //             state_.q_.z(), state_.q_.w(), -state_.q_.x(),
            //             -state_.q_.w(), state_.q_.z(), -state_.q_.y(),
            //             state_.q_.x(), state_.q_.y(), state_.q_.z();
            // F_temp.block<4, 1>(0, 0) = temp1 * deltaVel;
            // F_temp.block<4, 1>(0, 1) = temp2 * deltaVel;
            // F_temp.block<4, 1>(0, 2) = temp3 * deltaVel;
            // F.block<3, 4>(3, 6) = F_temp.transpose() * 2;
            // F.block<3, 3>(3, 10) = -state_.q_.toRotationMatrix() * dt;
            // F.block<4, 4>(6, 6) << 1, -0.5 * theta[0], -0.5 * theta[1], -0.5 * theta[2],
            //                         0.5 * theta[0], 1, 0.5 * theta[2], -0.5 * theta[1],
            //                         0.5 * theta[1], -0.5 * theta[2], 1, 0.5 * theta[0],
            //                         0.5 * theta[2], 0.5 * theta[1], -0.5 * theta[0], 1;
            // F.block<4, 3>(6, 13) << 0.5 * state_.q_.x(), 0.5 * state_.q_.y(), 0.5 * state_.q_.z(),
            //                         -0.5 * state_.q_.w(), 0.5 * state_.q_.z(), -0.5 * state_.q_.y(),
            //                         -0.5 * state_.q_.z(), -0.5 * state_.q_.w(), 0.5 * state_.q_.x(),
            //                         0.5 * state_.q_.y(), -0.5 * state_.q_.x(), -0.5 * state_.q_.w();
            // F.block<4, 3>(6, 13) *= dt;
            // // Matrix Q
            // MatrixXd Fi = MatrixXd::Zero(nStates_, 12);
            // Fi.block<3, 3>(3, 0) << -state_.q_.toRotationMatrix();
            // Fi.block<4, 3>(6, 3) << 0.5 * state_.q_.x(), 0.5 * state_.q_.y(), 0.5 * state_.q_.z(),
            //                         -0.5 * state_.q_.w(), 0.5 * state_.q_.z(), -0.5 * state_.q_.y(),
            //                         -0.5 * state_.q_.z(), -0.5 * state_.q_.w(), 0.5 * state_.q_.x(),
            //                         0.5 * state_.q_.y(), -0.5 * state_.q_.x(), -0.5 * state_.q_.w();
            // Fi.block<6, 6>(10, 6) = MatrixXd::Identity(12, 12);
            // MatrixXd Qi = MatrixXd::Zero(12, 12);
            // Qi.block<3, 3>(0, 0) = Matrix3d::Identity() * n_a_ * n_a_ * dt * dt;
            // Qi.block<3, 3>(3, 3) = Matrix3d::Identity() * n_w_ * n_w_ * dt * dt;
            // Qi.block<3, 3>(6, 6) = Matrix3d::Identity() * n_ba_ * n_ba_ * dt;
            // Qi.block<3, 3>(9, 9) = Matrix3d::Identity() * n_bw_ * n_bw_ * dt;
            // MatrixXd Q = Fi * Qi * Fi.transpose();
            // kf_.predict(F, Q);

            double dt = timestamp - lasttime_;
            lasttime_ = timestamp;

            imuGPSFusionStates last_state_ = state_;

            Vector3d acc = imuAccRaw - last_state_.ba_;
            Vector3d gyro = imuGyroRaw - last_state_.bw_;
            Vector3d theta = gyro * dt;

            // predict the states
            // attitude
            Quaterniond dq = getQuaternionFromAngle(theta);
            state_.q_ = last_state_.q_ * dq;
            Matrix3d R = state_.q_.toRotationMatrix();
            // velocity
            Vector3d deltaVel = (state_.q_ * acc + gravity_) * dt;
            state_.vel_ = last_state_.vel_ + deltaVel;
            // position
            state_.pos_ = last_state_.pos_ + 0.5 * (last_state_.vel_ + state_.vel_) * dt;

            // propagate the covariance
            // Matrix F
            MatrixXd F = MatrixXd::Identity(nStates_, nStates_);
            F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;
            F.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;
            F.block<3, 3>(3, 9) = -R * dt;
            // F.block<3, 3>(6, 6) = R.transpose() * skew_symmetric(gyro) * dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity() - skew_symmetric(gyro) * dt;
            F.block<3, 3>(6, 12) = -Matrix3d::Identity() * dt;
            // Matrix Q
            MatrixXd Fi = MatrixXd::Zero(nStates_, 12);
            Fi.block<12, 12>(3, 0) = MatrixXd::Identity(12, 12);
            MatrixXd Qi = MatrixXd::Zero(12, 12);
            Qi.block<3, 3>(0, 0) = Matrix3d::Identity() * n_a_ * n_a_ * dt * dt;
            Qi.block<3, 3>(3, 3) = Matrix3d::Identity() * n_w_ * n_w_ * dt * dt;
            Qi.block<3, 3>(6, 6) = Matrix3d::Identity() * n_ba_ * n_ba_ * dt;
            Qi.block<3, 3>(9, 9) = Matrix3d::Identity() * n_bw_ * n_bw_ * dt;
            MatrixXd Q = Fi * Qi * Fi.transpose();
            kf_.predict(F, Q);
        }
    }

/**
 * @brief Input GPS data to imuGPSFusion.
 * @param timestamp 
 * @param lla 
 * @param Covariance 
 */
    void imuGPSFusion::inputGPS(double timestamp, Vector3d lla, Matrix<double, 3, 3> Covariance)
    {
        if (!isInitialized())
        {
            if (imu_t_buff_.size() < imuBuffLength_)
            {
                cout << "No enough imu data to initialize!" << endl;
                return;
            }

            if (abs(timestamp - imu_t_buff_.back()) > 0.1)
            {
                cout << "GPS and IMU's timestamps are not synchronized!" << endl;
                return;
            }

            // init the state with imu data
            Vector3d sum_acc = Vector3d::Zero();
            for (auto &acc: imu_acc_buff_)
                sum_acc += acc;
            Vector3d mean_acc = sum_acc / (double) imu_acc_buff_.size();
            Vector3d sum_err2 = Vector3d::Zero();
            for (auto &acc: imu_acc_buff_)
                sum_err2 += (acc - mean_acc).cwiseAbs2();
            Vector3d std_acc = (sum_err2 / (double) imu_acc_buff_.size()).cwiseSqrt();
            if (std_acc.maxCoeff() > imuAccStdLimit)
            {
                cout << "IMU acc std is too big: " << std_acc.transpose() << endl;
                return;
            }

            Vector3d z_axis = mean_acc.normalized();
            Vector3d x_axis = (Vector3d::UnitX() - z_axis * z_axis.transpose() * Vector3d::UnitX()).normalized();
            Vector3d y_axis = (z_axis.cross(x_axis)).normalized();
            Matrix3d baselink_R_ENU;
            baselink_R_ENU.block<3, 1>(0, 0) = x_axis;
            baselink_R_ENU.block<3, 1>(0, 1) = y_axis;
            baselink_R_ENU.block<3, 1>(0, 2) = z_axis;
            state_.q_ = baselink_R_ENU.transpose();

            MatrixXd P = MatrixXd::Zero(nStates_, nStates_);
            P.block<6, 6>(0, 0) = Matrix<double, 6, 6>::Identity() * 100.0;
            P.block<2, 2>(6, 6) = Matrix2d::Identity() * 10.0 * degreeToRadian * 10.0 * degreeToRadian;
            P(8, 8) = 100.0 * degreeToRadian * 100.0 * degreeToRadian;
            P.block<6, 6>(9, 9) = Matrix<double, 6, 6>::Identity() * 0.0004;
            kf_ = baseKF(nStates_);
            kf_.init(P);

            lasttime_ = imu_t_buff_.back();
            isInitialized_ = true;
            init_lla_ = lla;
        } else
        {
            // residuals
            Vector3d dz = convertLlaToENU(init_lla_, lla) - state_.pos_;

            // update the covariance
            // Matrix H
            MatrixXd H = MatrixXd::Zero(3, nStates_);
            H.block<3, 3>(0, 0) = Matrix3d::Identity();
            // Matrix R
            MatrixXd R = Covariance;
            kf_.update(dz, H, R);

            // update the states
            updateStates();
        }
    }

/**
 * @brief Update the states of imuGPSFusion.
 */
    void imuGPSFusion::updateStates()
    {
        VectorXd dx = kf_.getDeltaStates();
        state_.pos_ += dx.segment(0, 3);
        state_.vel_ += dx.segment(3, 3);
        Quaterniond dq = getQuaternionFromAngle(dx.segment(6, 3));
        state_.q_ *= dq;
        state_.ba_ += dx.segment(9, 3);
        state_.bw_ += dx.segment(12, 3);
    }

/**
 * @brief Return the position
 * @return Eigen::Vector3d position
 */
    Vector3d imuGPSFusion::getPosition()
    {
        return state_.pos_;
    }

/**
 * @brief Return the velocity
 * @return Eigen::Vector3d velocity
 */
    Vector3d imuGPSFusion::getVelocity()
    {
        return state_.vel_;
    }

/**
 * @brief Return the orientation
 * @return Eigen::Quaterniond orientation
 */
    Quaterniond imuGPSFusion::getRotation()
    {
        return state_.q_;
    }

/**
 * @brief Return the time of filer
 * @return double time
 */
    double imuGPSFusion::getCurrentTime()
    {
        return lasttime_;
    }

} // namespace multiSensorFusion