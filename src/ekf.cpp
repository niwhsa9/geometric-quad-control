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
    P.setIdentity();
}


EKF::State EKF::get_state() {
    return x;
}

void EKF::predict(Eigen::Vector3d gyro, Eigen::Vector3d accel) {
    //std::cout << "accel " << accel.x() << ", " << accel.y() << ", " << accel.z() << std::endl;
    //std::cout << "vel " << x.dX.x() << ", " << x.dX.y() << ", " << x.dX.z() << std::endl;
    // Compute gyro in global frame
    //auto omega = x.X.asSO3().adj() * gyro;
    Eigen::Vector3d gravity(0.0, 0.0, 9.81);
    auto R_body_to_world = x.X.asSO3().rotation();
    auto adjusted_accel = accel- R_body_to_world.transpose()* gravity; 

    //std::cout << "adj accel " << adjusted_accel.x() << ", " << adjusted_accel.y() << ", " << adjusted_accel.z() << std::endl;
    //auto ea = x.X.rotation().eulerAngles(0, 1, 2);
    //std::cout << "angle estimate" <<  ea.x() << ", " << ea.y() << ", " << ea.z() << std::endl;

    // Update dX
    x.dX.head<3>() += adjusted_accel*dt;
    x.dX.tail<3>() = gyro;//omega;

    //std::cout << manif::SE3Tangentd(x.dX) << std::endl;

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