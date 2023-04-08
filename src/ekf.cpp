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
    Eigen::Matrix3d R_body_in_world = X.rotation();
    Eigen::Vector3d accel_in_body = accel - R_body_in_world.transpose()*gravity;

    // Update X
    Vector9d u;
    u << R_body_in_world.transpose() * X.linearVelocity() * dt + 0.5 * dt * dt * accel_in_body,
        dt * gyro, dt * accel_in_body;

    // Construct dynamics Jacobian
    manif::SE_2_3d::Jacobian F, J_o_dx;
    X = X.rplus(manif::SE_2_3Tangentd(u), F, J_o_dx);

    // Prediction state covariance
    P = F * P * F.transpose() + Q; 
}

// EKF update step

void EKF::update_gps(Eigen::Vector3d pos, Eigen::Vector3d vel) {
    // Innovation 
    Eigen::Vector3d y_pos = pos - X.translation();
    auto y_vel = vel - X.linearVelocity();
    Vector6d y;
    y << y_pos, y_vel;

    // Sensor model Jacobian
    Eigen::Matrix<double, 6, 9> H = Eigen::Matrix<double, 6, 9>::Zero();
    H.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(3, 6) = Eigen::Matrix3d::Identity();

    // Innovation covariance
    Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + R_GPS;

    // Kalman gain
    Eigen::Matrix<double, 9, 6> K = P * H.transpose() * S.inverse();

    // State update
    Eigen::Matrix<double, 9, 1> dx = K * y;
    X = X.rplus(manif::SE_2_3Tangentd(dx));
    //std::cout << dx << std::endl;

    // State Covariance update
    //P -= K * H * P;
    P -= K * S * K.transpose();
}
