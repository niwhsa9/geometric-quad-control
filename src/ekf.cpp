#include "ekf.h"
#include <iostream>
#include <manif/impl/se_2_3/SE_2_3Tangent.h>

EKF::EKF(
    ProcNoiseMat proc_noise, 
    ObvNoiseMatAccel accel_noise, 
    ObvNoiseMatGPS gps_noise, 
    double dt
    ): 
Q(proc_noise), R_Accel(accel_noise), R_GPS(gps_noise), dt(dt)
{
    X.setIdentity();
    P.setZero();
}


EKF::State EKF::get_state() {
    return X;
}

// Discrete time state propogation
// For SE_2 (3) velocity component is world frame velocity
void EKF::predict(Eigen::Vector3d gyro, Eigen::Vector3d accel) {
    Eigen::Vector3d gravity(0.0, 0.0, 9.81);
    auto R_body_in_world = X.rotation();
    auto accel_in_body = accel - R_body_in_world.transpose()*gravity;

    // Update X
    Vector9d u;
    u << R_body_in_world.transpose() * X.linearVelocity() * dt + 0.5 * dt * dt * accel_in_body,
        dt * gyro, dt * accel_in_body;

    // Construct dynamics Jacobian
    manif::SE_2_3d::Jacobian J_o_x, J_o_dx;
    X = X.rplus(manif::SE_2_3Tangentd(u), J_o_x, J_o_dx);
    auto F = J_o_x;

    // Prediction state covariance
    P = F * P * F.transpose() + Q; 
}

// EKF update step
/*
void EKF::update_gps(Eigen::Vector3d pos, Eigen::Vector3d vel) {
    // Innovation 
    auto y_pos = pos - x.X.translation();
    //auto y_vel = vel - x.X.asSO3().adj() * x.dX.head<3>();
    //Vector6d y;
    //y << y_pos, y_vel;
    auto y = y_pos;

    // Sensor model Jacobian
    //Eigen::Matrix<double, 6, 12> H = Eigen::Matrix<double, 6, 12>::Zero();
    Eigen::Matrix<double, 3, 12> H = Eigen::Matrix<double, 3, 12>::Zero();
    H.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
    //H.block<3,3>(3, 6) = x.X.asSO3().adj();

    // Innovation covariance
    auto S = H * P * H.transpose() + Eigen::Matrix3d::Identity();//R_GPS;

    // Kalman gain
    auto K = P * H.transpose() * S.inverse();

    // State update
    Eigen::Matrix<double, 12, 1> dx = K * y;
    x.X = x.X.rplus(manif::SE3Tangentd(dx.head<6>()));
    x.dX += dx.tail<6>();
    //x.X.translation(dx.head<3>() + x.X.translation());
    //x.dX.head<3>() += dx.block<3, 1>(6, 0);//dx.tail<3>();
    //std::cout << dx << std::endl;

    // State Covariance update
    P -= K * H * P;
}
*/