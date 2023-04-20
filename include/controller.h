#include <Eigen/Dense>
#include <manif/impl/se_2_3/SE_2_3.h>
#include <optional>
#include "manif/SE_2_3.h"


class Controller {
    public: 
        //Controller(double, double, double, Eigen::Matrix3d);
        Controller()=default;
        Controller(const Controller&) = delete;
        Controller& operator=(const Controller&) = delete;

        struct State {
            manif::SE_2_3d X;
            Eigen::Vector3d omega;
            Eigen::Vector3d acc;
        };
        // Computes rotor velocities based on tracking errors
        Eigen::Vector4d iterate_ctrl(const State &X, const State &X_d);

        // Uses differential flatness in [x, y, z, yaw] to compute the desired
        // attitude and angular velocity to track
        State track_target(const Eigen::Vector4d &y, const std::optional<manif::SO3d> &prev_attitude);

    private:
        double kp = 2.0, kv = 0.0, kr = 0.04, komega = 0.0;

        // Thrust coefficient newton/(rad/s)^2
        double cf = 0.00026 ;//* 1.0/10.0;
        // Drag coefficient
        double cd = 0.0000052;
        // Inertia tensor
        Eigen::Matrix3d I; 
        // Mass
        double mass = 0.4;
};