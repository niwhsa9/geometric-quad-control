#include "manif/SE3.h"

class EKF {
    using ProcNoiseMat = Eigen::Matrix<double, 6, 6>;
    using ObvNoiseMat = Eigen::Matrix<double, 6, 6>;


    public:
        EKF(ProcNoiseMat process_noise_cov, Eigen::Matrix3d, );
        EKF(const EKF&) = delete;

        struct State {
            manif::SE3d x;
            manif::SE3Tangentd dx;
        };
        
        State get_state();

    private:
        State X;

        ProcNoiseMat Q;
        
};