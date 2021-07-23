/**
 * @file baseKF.cpp
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief realize the base kalman filter, predict and update the covariance of states
 * Note: the states are predicted or updated outside this filter
 * @version 0.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "baseKF.h"

namespace multiSensorFusion
{

baseKF::baseKF(int nStates)
{
    nStates_ = nStates;
    P_ = MatrixXd::Identity(nStates_, nStates_);
    delta_states_ = VectorXd::Zero(nStates_);
}

void baseKF::init(const MatrixXd &P)
{
    nStates_ = int(P.rows());
    P_ = P;
    delta_states_ = VectorXd::Zero(nStates_);
}

void baseKF::predict(const MatrixXd &F, const MatrixXd &Q)
{
    P_ = F * P_ * F.transpose() + Q;
}

void baseKF::update(const VectorXd &dz, const MatrixXd &H, const MatrixXd &R)
{
    MatrixXd S = H * P_ * H.transpose() + R;
    MatrixXd K = P_ * H.transpose() * S.inverse();
    delta_states_ = K * dz;
    MatrixXd I = MatrixXd::Identity(nStates_, nStates_);
    // P_ = (I - K * H) * P_;
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
}

void baseKF::resetESKF(const MatrixXd &G)
{
    delta_states_.setZero();
    P_ = G * P_ * G.transpose();
}

VectorXd baseKF::getDeltaStates() const
{
    return delta_states_;
}

MatrixXd baseKF::getUncertainty() const
{
    return P_;
}

} // namespace multiSensorFusion