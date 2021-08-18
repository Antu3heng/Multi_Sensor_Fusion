/**
 * @file baseKF.h
 * @author Xinjiang Wang (wangxj83@sjtu.edu.cn)
 * @brief the header of baseKF.cpp
 * @version 0.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef FUSION_BASE_KF_H_
#define FUSION_BASE_KF_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace multiSensorFusion
{
    class baseKF
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    public:
        baseKF() = default;;

        explicit baseKF(int nStates);

        ~baseKF() = default;;

        void init(const MatrixXd &P);

        void predict(const MatrixXd &F, const MatrixXd &Q);

        void update(const VectorXd &dz, const MatrixXd &H, const MatrixXd &R);

        void resetESKF(const MatrixXd &G);

        VectorXd getDeltaStates() const;

        MatrixXd getUncertainty() const;

    private:
        int nStates_;
        MatrixXd P_;
        VectorXd delta_states_;
    };
} // namespace multiSensorFusion

#endif //FUSION_BASE_KF_H_