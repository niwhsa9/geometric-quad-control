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

    // Update dX
    x.dX.head<3>() += adjusted_accel*dt;
    x.dX.tail<3>() = gyro;//omega;

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
}

// EKF update step
void EKF::update_gps(Eigen::Vector3d pos, Eigen::Vector3d vel) {
    // Innovation 
    auto y_pos = pos - x.X.translation();
    auto y_vel = vel - x.X.asSO3().adj() * x.dX.head<3>();
    Vector6d y;
    y << y_pos, y_vel;

    // Sensor model Jacobian
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    H.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(3, 3) = x.X.asSO3().adj();

    // Innovation covariance
    auto S = H * P * H.transpose() + R_GPS;

    // Kalman gain
    auto K = P * H.transpose() * S.inverse();

    // State update
    auto dx = K * y;
    //x.X.translation() += dx.head<3>();
    //x.dX.head<3>() += dx.tail<3>();

    // State Covariance update
    //P -= K * H * P;
}