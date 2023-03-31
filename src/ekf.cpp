#include "ekf.h"
#include <iostream>

EKF::EKF(
    ProcNoiseMat proc_noise, 
    ObvNoiseMatAccel accel_noise, 
    ObvNoiseMatGPS gps_noise, 
    double dt
    ) : 
Q(proc_noise), R_Accel(accel_noise), R_GPS(gps_noise), dt(dt)
{
    x.dX = Vector6d::Zero();
    x.X.setIdentity();
}


EKF::State EKF::get_state() {
    return x;
}

void EKF::predict(Eigen::Vector3d gyro, Eigen::Vector3d accel) {
    std::cout << "accel " << accel.x() << ", " << accel.y() << ", " << accel.z() << std::endl;
    std::cout << "vel " << x.dX.x() << ", " << x.dX.y() << ", " << x.dX.z() << std::endl;
    // Compute gyro in global frame
    //auto omega = x.X.asSO3().adj() * gyro;

    //auto adjusted_accel = accel

    // Update dX
    x.dX.head<3>() += accel*dt;
    //x.dX.tail<3>() = gyro;//omega;

    std::cout << manif::SE3Tangentd(x.dX) << std::endl;

    // Update X
    manif::SE3d::Jacobian J_o_x, J_o_dx;
    x.X = x.X.rplus(manif::SE3Tangentd(x.dX), J_o_x, J_o_dx);

    // Construct dynamics Jacobian
    /*
    auto F = Eigen::Matrix<double, 12, 12>::Zero();
    F.block(0, 0, 6, 6) = J_o_x;
    F.block(0, 6, 6, 6) = J_o_dx;
    F.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity();
    F.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity();

    // Prediction state covariance
    P = F * P * F.transpose() + Q; 

    */
}