#include "ekf.h"
#include <manif/impl/se3/SE3Tangent.h>



EKF::EKF(ProcNoiseMat proc_noise, ObvNoiseMatAccel accel_noise, ObvNoiseMatGPS gps_noise, double dt) : 
Q(proc_noise), R_Accel(accel_noise), R_GPS(gps_noise), dt(dt)
{

}

void EKF::predict(Eigen::Vector3d gyro, Eigen::Vector3d accel) {
    // Compute gyro in global frame
    auto omega = x.X.asSO3().adj() * gyro;

    // Update dX
    x.dX.head<3>() += accel*dt;
    x.dX.tail<3>() = omega;

    // Update X
    manif::SE3d::Jacobian J_o_x;
    x.X.lplus(manif::SE3Tangentd(x.dX), J_o_x);
}