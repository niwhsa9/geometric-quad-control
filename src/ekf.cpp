#include "ekf.h"
#include <iostream>

EKF::EKF(
    ProcNoiseMat proc_noise, 
    ObvNoiseMatAccel accel_noise, 
    ObvNoiseMatGPS gps_noise, 
    double dt
    ): 
Q(proc_noise), R_Accel(accel_noise), R_GPS(gps_noise), dt(dt),
X(Eigen::Vector3d::Zero(), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Eigen::Vector3d::Zero())
{
    //X.setIdentity();
    //X.quat();
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
    X.normalize();

    // Prediction state covariance
    P = F * P * F.transpose() + Q; 
}

// EKF update step
void EKF::left_invariant_update(Eigen::Vector3d z, Eigen::Vector3d b, Eigen::Matrix3d R) {
    // Innovation and sensor Jacobian
    manif::SE_2_3d::Jacobian J1; 
    Eigen::Matrix<double, 3, 9> J2;
    //J1.setIdentity();
    //J2.setIdentity();

    Eigen::Vector3d y = -X.inverse(J1).act(z, J2) + b;

    Eigen::Matrix<double, 3, 9> H = J2 * J1;
    
    // Innovation covariance
    Eigen::Matrix<double, 3, 3> S = H * P * H.transpose() + R;//X.asSO3().adj() * R * X.asSO3().adj().transpose();

    // Kalman gain
    Eigen::Matrix<double, 9, 3> K = P * H.transpose() * S.inverse();

    // State update
    Eigen::Matrix<double, 9, 1> dx = K * y;
    X = X.rplus(manif::SE_2_3Tangentd(dx));
    X.normalize();

    // State Covariance update
    P -= K * S * K.transpose();
    /*
    // Innovation 
    Eigen::Vector3d y_pos = pos - X.translation();
    Eigen::Vector3d y_vel = vel - X.linearVelocity();
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

    //dx.block<3, 1>(3, 0) = Eigen::Vector3d::Zero();

    //std::cout << dx << std::endl;
    X = X.rplus(manif::SE_2_3Tangentd(dx));
    X.normalize();

    // State Covariance update
    P -= K * S * K.transpose();
    */
}

// Right Invariant EKF update
void EKF::right_invariant_update(Eigen::Vector3d z, Eigen::Vector3d b, Eigen::Matrix3d R) {
    // Innovation and sensor Jacobian
    manif::SO3d::Jacobian J1;
    J1.setIdentity();

    Eigen::Vector3d y = manif::SO3d(X.quat()).act(z, J1) - b;

    Eigen::Matrix3d H_rot = J1;//Eigen::Matrix3d::Identity();//J1 * X.asSO3().adj();
    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
    H.block<3, 3>(0, 3) = H_rot;
    //std::cout << H_rot << std::endl;

    ProcNoiseMat P_left = X.adj() * P * X.adj().transpose();
    
    // Innovation covariance
    Eigen::Matrix<double, 3, 3> S = H * P_left * H.transpose() + R;

    // Kalman gain
    Eigen::Matrix<double, 9, 3> K = P_left * H.transpose() * S.inverse();

    // State update
    Eigen::Matrix<double, 9, 1> dx = K * y;
    X = manif::SE_2_3Tangentd(dx) + X;
    X.normalize();

    // State Covariance update
    P = X.adj().inverse() * (P_left - K * S * K.transpose()) * X.adj().inverse().transpose();
    //P = X.inverse().adj() * (P_left - K * S * K.transpose()) * X.inverse().adj().transpose();
}

void EKF::update_imu(Eigen::Vector3d mag, Eigen::Vector3d acc) {
    right_invariant_update(acc, Eigen::Vector3d(0.0, 0.0, 9.81), R_Accel);
    right_invariant_update(mag, Eigen::Vector3d(0.0, 1.0, 0.0), R_Accel);
}

void EKF::update_gps(Eigen::Vector3d pos, Eigen::Vector3d vel) {
    //left_invariant_update(pos, Eigen::Vector3d(0.0, 0.0, 0.0), R_GPS.block<3,3>(0, 0)); 
}


/*
TODO design:
1. run ekf in its own thread, data updates wake up cv and run update steps
2. right_invraint_update(std::function<tuple<y,h>(z, b)>) then deduce matrix sizes
3. change to correct forms

*/