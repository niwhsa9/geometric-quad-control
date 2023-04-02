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
    P.setZero();
}


EKF::State EKF::get_state() {
    return x;
}

// Discrete time state propogation
void EKF::predict(Eigen::Vector3d gyro, Eigen::Vector3d accel) {
    Eigen::Vector3d gravity(0.0, 0.0, 9.81);
    auto R_body_to_world = x.X.asSO3().rotation();
    auto adjusted_accel = accel- R_body_to_world.transpose()* gravity; 

    // Compute state updates in local frame
    Vector6d imu_local;
    imu_local.head<3>() = accel*dt;
    imu_local.tail<3>() = gyro;//omega;

    // Use adjoint to get imu in global frame
    Vector6d imu_global = x.X.adj() * imu_local; 

    // Compute update

    // Update X
    manif::SE3d::Jacobian J_o_x, J_o_dx;
    x.X = x.X.rplus(manif::SE3Tangentd(x.dX*dt), J_o_x, J_o_dx);

    // Construct dynamics Jacobian
    Eigen::Matrix<double, 12, 12> F = Eigen::Matrix<double, 12, 12>::Zero();
    F.block<6,6>(0, 0) = J_o_x;
    F.block<6,6>(0, 6) = J_o_dx;
    F.block<3,3>(6, 6) = Eigen::Matrix3d::Identity();
    F.block<3,3>(9, 9) = Eigen::Matrix3d::Identity();

    // Prediction state covariance
    P = F * P * F.transpose() + Q; 
    std::cout << P.coeff(0, 0) << std::endl;
}

// EKF update step
void EKF::update_gps(Eigen::Vector3d pos, Eigen::Vector3d vel) {
        //auto omega = x.X.asSO3().adj() * gyro;

    // Error measured minus expected
    auto y_pos = pos - x.X.translation();
    auto y_vel = vel - x.dX.head<3>();
}