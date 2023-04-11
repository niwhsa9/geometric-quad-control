#include "manif/SE_2_3.h"
#include <Eigen/Dense>

class EKF {
    public:
        using Vector6d = Eigen::Matrix<double, 6, 1>;
        using Vector9d = Eigen::Matrix<double, 9, 1>;
        using ProcNoiseMat = Eigen::Matrix<double, 9, 9>;
        using ObvNoiseMatAccel = Eigen::Matrix3d;
        using ObvNoiseMatGPS = Eigen::Matrix<double, 6, 6>;
        using State = manif::SE_2_3d;

        EKF(ProcNoiseMat, ObvNoiseMatAccel, ObvNoiseMatGPS, double);
        EKF(const EKF&) = delete;

        State get_state();
        void predict(Eigen::Vector3d gyro, Eigen::Vector3d accel);
        void update_gps(Eigen::Vector3d pos, Eigen::Vector3d vel);
        void update_imu(Eigen::Vector3d mag, Eigen::Vector3d acc);

    private:
        void invariant_update(Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d);

        State X;
        ProcNoiseMat P, Q;
        ObvNoiseMatGPS R_GPS;
        ObvNoiseMatAccel R_Accel;
        double dt;
        
};