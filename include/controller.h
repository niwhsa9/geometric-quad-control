#include <Eigen/Dense>
#include <manif/impl/se_2_3/SE_2_3.h>
#include "manif/SE_2_3.h"


class Controller {
    public: 
        Controller(double, double, double, Eigen::Matrix3d);
        Controller(const Controller&) = delete;
        Controller& operator=(const Controller&) = delete;

        struct State {
            manif::SE_2_3d X;
            Eigen::Vector3d omega;
        };

        Eigen::Vector4d iterate_ctrl(const State &X, const State &X_d);
    private:
        // Thrust coefficient newton/(rad/s)^2
        double cf = 1.0;
        // Drag coefficient
        double cd = 1.0;
        // Inertia tensor
        Eigen::Matrix3d I; 
        // Mass
        double mass = 1.0;
};