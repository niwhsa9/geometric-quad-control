#include <concepts>
#include <Eigen/Dense>
#include "manif/SE_2_3.h"
#include <tuple>

namespace amg {
    // Returns concept checks lambda return type
    template <typename T, typename U, typename... Args> 
    concept returns = requires(T a, Args&&... b) 
    {
    {a( std::forward<Args>(b)...)} -> std::same_as<U>;
    };
}

class EKF {
    public:
        using Vector6d = Eigen::Matrix<double, 6, 1>;
        using Vector9d = Eigen::Matrix<double, 9, 1>;
        using ProcNoiseMat = Eigen::Matrix<double, 9, 9>;
        using ObvNoiseMatAccel = Eigen::Matrix3d;
        using ObvNoiseMatGPS = Eigen::Matrix<double, 6, 6>;
        using ObvJacobian = Eigen::Matrix<double, 3, 9>;
        using State = manif::SE_2_3d;

        EKF(const ProcNoiseMat&, const ObvNoiseMatAccel&, const ObvNoiseMatGPS&, double);
        EKF(const EKF&) = delete;

        State get_state();
        void predict(const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel);
        void update_gps(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel);
        void update_imu(const Eigen::Vector3d &mag, const Eigen::Vector3d &acc);

    private:
        void right_invariant_update(Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d);
        void left_invariant_update(Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d);

        template <typename Callable>
        void obv_update(const Eigen::Vector3d &z, Callable obv_model, const Eigen::Matrix3d &R) requires 
            std::invocable<Callable, manif::SE_2_3d&> && 
            amg::returns<Callable, std::tuple<Eigen::Vector3d, ObvJacobian>, manif::SE_2_3d&> 
        {
            auto [z_hat, H] = std::move(obv_model(X));

            // Innovation
            Eigen::Vector3d y = z - z_hat;

            Eigen::Matrix<double, 3, 3> S = H * P * H.transpose() + R;

            // Kalman gain
            Eigen::Matrix<double, 9, 3> K = P * H.transpose() * S.inverse();

            // State update
            Eigen::Matrix<double, 9, 1> dx = K * y;
            X = X.rplus(manif::SE_2_3Tangentd(dx));
            X.normalize();

            // State Covariance update
            P -= K * S * K.transpose();
        }

        State X;
        ProcNoiseMat P, Q;
        ObvNoiseMatGPS R_GPS;
        ObvNoiseMatAccel R_Accel;
        double dt;
        
};