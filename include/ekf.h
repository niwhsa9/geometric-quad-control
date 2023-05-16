#pragma once

#include <concepts>
#include <Eigen/Dense>
#include "manif/SE_2_3.h"
#include <tuple>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <variant>
#include <thread>

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
        using ObvNoiseMatMag = Eigen::Matrix3d;
        using ObvNoiseMatGPS = Eigen::Matrix<double, 6, 6>;
        using ObvJacobian = Eigen::Matrix<double, 3, 9>;
        using State = manif::SE_2_3d;

        EKF(const ProcNoiseMat&, const ObvNoiseMatAccel&, const ObvNoiseMatGPS&, const ObvNoiseMatMag&, double);
        EKF(const EKF&) = delete;
        EKF& operator=(const EKF&) = delete;

        virtual State get_state();
        virtual void predict(const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc);
        virtual void update_gps(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel);
        virtual void update_imu(const Eigen::Vector3d &mag, const Eigen::Vector3d &acc);
        virtual void update_acc(const Eigen::Vector3d &acc);
        virtual void update_mag(const Eigen::Vector3d &mag);

    protected:
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
        ObvNoiseMatMag R_Mag;
        double dt;
};

class EKFWorker : public EKF {

    using EKF::EKF;

    public:
        void loop_ekf();
        void predict(const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc) override;
        void update_gps(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) override;
        void update_imu(const Eigen::Vector3d &mag, const Eigen::Vector3d &acc) override;
        void update_acc(const Eigen::Vector3d &acc) override;
        void update_mag(const Eigen::Vector3d &mag) override;
        State get_state() override;

    private: 
        std::mutex mtx;
        std::condition_variable cv;
        std::unique_ptr<std::thread> thread;

        void dispatch();

        struct Predict {
            Eigen::Vector3d gyro, acc;
        };
        struct GpsUpdate {
            Eigen::Vector3d pos, vel;
        };
        struct ImuUpdate {
            Eigen::Vector3d mag, acc;
        };
        struct MagUpdate {
            Eigen::Vector3d mag;
        };
        struct AccUpdate {
            Eigen::Vector3d acc;
        };

        using WorkType = std::variant<Predict, GpsUpdate, ImuUpdate, MagUpdate, AccUpdate>;
        std::queue<WorkType> work_queue;
};