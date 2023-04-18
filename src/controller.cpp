#include "controller.h"


Eigen::Vector4d Controller::iterate_ctrl(const State &X, const State &X_d) {
    Eigen::Vector3d e_p = X.X.translation() - X_d.X.translation();
    Eigen::Vector3d e_v = X.X.linearVelocity() - X_d.X.linearVelocity();

    Eigen::Matrix3d R_w_d; // determine

    // determine desired angular rate

    //Eigen::Vector3d e_r = 0.5 * ()
}