#include "manif/SE3.h"
#include <Eigen/Dense>

class EKF {
    public:
        using Vector6d = Eigen::Matrix<double, 6, 1>;
        using ProcNoiseMat = Eigen::Matrix<double, 12, 12>;
        using ObvNoiseMatAccel = Eigen::Matrix3d;
        using ObvNoiseMatGPS = Eigen::Matrix3d;

        EKF(ProcNoiseMat, ObvNoiseMatAccel, ObvNoiseMatGPS, double);
        EKF(const EKF&) = delete;

        struct State {
            manif::SE3d X;
            Vector6d dX;
        };
        
        State get_state();
        void predict(Eigen::Vector3d gyro, Eigen::Vector3d accel);
        void update_gps(Eigen::Vector3d pos, Eigen::Vector3d vel);

    private:
        State x;
        ProcNoiseMat P, Q;
        ObvNoiseMatGPS R_GPS;
        ObvNoiseMatAccel R_Accel;
        double dt;
        
};