#include "controller.h"
#include <iostream>
#include <manif/impl/so3/SO3.h>


Eigen::Vector4d Controller::iterate_ctrl(const State &X, const State &X_d) {
    //I = Eigen::Matrix3cd::Ze
    I << 0.000913, 0, -0.000357, 0, 0.00236, 0, -0.000357, 0, 0.00279;

    // Tracking errors
    Eigen::Vector3d e_p = X.X.translation() - X_d.X.translation();
    Eigen::Vector3d e_v = X.X.linearVelocity() - X_d.X.linearVelocity();
    Eigen::Matrix3d e_r_hat = 0.5 * (X_d.X.rotation().transpose() * X.X.rotation() - 
        X.X.rotation().transpose() * X_d.X.rotation());
    Eigen::Vector3d e_r(e_r_hat(2, 1), e_r_hat(0, 2), e_r_hat(1, 0));

    //Eigen::Matrix3d t = X_d.X.rotation().transpose() * X.X.rotation();
    //Eigen::Matrix3d t = X_d.X.rotation().transpose() * X.X.rotation();
    //Eigen::Vector3d e_r(manif::SO3d(Eigen::Quaterniond(t)).log().coeffs());
    Eigen::Vector3d e_omega = X.omega - X.X.rotation().transpose() * X_d.X.rotation() * X_d.omega;

    Eigen::Vector3d g(0.0, 0.0, 9.81);
    Eigen::Vector3d e3(0, 0, 1);

    // Force control law projects ideal correction force onto vehicle z
    double f_z = 9.81*0.7;//(-kp * e_p - kv * e_v + mass * g + mass * X_d.acc).dot(X.X.rotation()*e3);

    // Torque control law stabilizes attitude 
    // TODO higher order terms neglected
    Eigen::Vector3d tau = -kr * e_r;//- komega * e_omega + X.omega.cross(I * X.omega);

    Eigen::Vector3d fl(0.110, 0.1375, 0);
    Eigen::Vector3d bl(-0.110, 0.1375, 0);
    Eigen::Vector3d br(-0.110, -0.1375, 0);
    Eigen::Vector3d fr(0.110, -0.1375, 0);

    // Map from angular velocity to torque
    Eigen::Matrix<double, 4, 4> F;
    F << cf, cf, cf, cf,
    cd*e3 + cf*fl.cross(e3),
    -cd*e3 + cf*bl.cross(e3), //-1
    cd*e3 + cf*br.cross(e3),
    -cd*e3 + cf*fr.cross(e3); //-
    Eigen::Vector4d q;
    q << f_z, tau;

    //std::cout << "Error P" << std::endl;
    //std::cout << e_p << std::endl;
    //std::cout << "Error R" << std::endl;
    //std::cout << e_r/e_r.norm()<< std::endl;
    //Eigen::Vector3d n = X_d.X.asSO3().between(X.X.asSO3()).log().coeffs();
    //std::cout <<  n/ n.norm()<< std::endl;
    //std::cout << "CMD" << std::endl;
    //std::cout << q << std::endl;

    Eigen::Vector4d vel_square = (F.inverse() *q);
    return vel_square.cwiseAbs().cwiseSqrt().array() * vel_square.cwiseSign().array();


}
/*
Controller::State track_target(const Eigen::Vector4d &y, const std::optional<manif::SO3d> &prev_attitude) {
    // Determine desired attitude

    // Determine desired angular rate by numerical differentiation
}
*/