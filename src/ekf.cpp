#include "ekf.h"
#include <iostream>
#include <manif/impl/se3/SE3.h>
#include <memory>
#include <mutex>
#include <variant>

EKF::EKF(
    const ProcNoiseMat &proc_noise, 
    const ObvNoiseMatAccel &accel_noise, 
    const ObvNoiseMatGPS &gps_noise, 
    const ObvNoiseMatMag &mag_noise, 
    double dt
    ): 
Q(proc_noise), R_Accel(accel_noise), R_GPS(gps_noise), R_Mag(mag_noise), dt(dt),
X(Eigen::Vector3d::Zero(), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Eigen::Vector3d::Zero())
{
    X.setIdentity();
    P.setZero();
}

EKF::State EKF::get_state() {
    return X;
}

// Discrete time state propogation
void EKF::predict(const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel) {
    Eigen::Vector3d gravity(0.0, 0.0, 9.81);
    Eigen::Matrix3d R_body_in_world = X.rotation();
    Eigen::Vector3d accel_in_body = accel - R_body_in_world.transpose()*gravity;

    // Update X
    Vector9d u;
    u << R_body_in_world.transpose() * X.linearVelocity() * dt + 0.5 * dt * dt * accel_in_body,
        dt * gyro, dt * accel_in_body;

    // Construct dynamics Jacobian
    manif::SE_2_3d::Jacobian F, J_o_dx;
    X = X.rplus(manif::SE_2_3Tangentd(u), F, J_o_dx);
    X.normalize();

    // Prediction state covariance
    P = F * P * F.transpose() + Q; 
}

void EKF::update_imu(const Eigen::Vector3d &mag, const Eigen::Vector3d &acc) {
    obv_update(acc, 
        [] (manif::SE_2_3d &X){
            manif::SO3d::Jacobian J1, J2;
            Eigen::Vector3d b(0.0, 0.0, 9.81);
            Eigen::Vector3d m = X.asSO3().inverse(J1).act(b, J2); 
            Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
            H.block<3, 3>(0, 3) = J2 * J1;
            return std::make_tuple<Eigen::Vector3d, ObvJacobian>(std::move(m), std::move(H));
        }, 
        R_Accel);

    obv_update(mag, 
        [] (manif::SE_2_3d &X){
            manif::SO3d::Jacobian J1, J2;
            Eigen::Vector3d b(0.0, 1.0, 0.0);
            Eigen::Vector3d m = X.asSO3().inverse(J1).act(b, J2); 
            Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
            H.block<3, 3>(0, 3) = J2 * J1;
            return std::make_tuple<Eigen::Vector3d, ObvJacobian>(std::move(m), std::move(H));
        }, 
        R_Mag);
}

void EKF::update_gps(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) {
    obv_update(pos, 
        [] (manif::SE_2_3d &X){
            Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
            H.block<3, 3>(0, 0) = X.rotation();
            return std::make_tuple<Eigen::Vector3d, ObvJacobian>(X.translation(), std::move(H));
        }, 
        R_GPS.block<3, 3>(0, 0));

    obv_update(vel, 
        [] (manif::SE_2_3d &X){
            Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
            H.block<3, 3>(0, 6) = X.rotation();
            return std::make_tuple<Eigen::Vector3d, ObvJacobian>(X.linearVelocity(), std::move(H));
        }, 
        R_GPS.block<3, 3>(3, 3));
}


void EKFWorker::predict(const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc) {
    std::unique_lock<std::mutex> lock(mtx);
    work_queue.push(Predict { gyro, acc } );
    cv.notify_one();    
}
void EKFWorker::update_gps(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) {
    std::unique_lock<std::mutex> lock(mtx);
    work_queue.push(GpsUpdate { pos, vel } );
    cv.notify_one();    
}
void EKFWorker::update_imu(const Eigen::Vector3d &mag, const Eigen::Vector3d &acc) {
    std::unique_lock<std::mutex> lock(mtx);
    work_queue.push(ImuUpdate { mag, acc } );
    cv.notify_one();    
}

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

void EKFWorker::dispatch() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]{ return !work_queue.empty();});    

    while(!work_queue.empty()) {
        auto job = work_queue.front();
        work_queue.pop();

        auto visitor = overloaded {
            [this](const Predict& p) { EKF::predict(p.gyro, p.acc); },
            [this](const GpsUpdate& u) {  EKF::update_gps(u.pos, u.vel);  },
            [this](const ImuUpdate& u) { EKF::update_imu(u.mag, u.acc);  },
        };
        std::visit(visitor, job);

    }
}

EKFWorker::State EKFWorker::get_state() {
    std::unique_lock<std::mutex> lock(mtx);
    return X;
}

void EKFWorker::loop_ekf() {
    thread = std::make_unique<std::thread>([this] {
        while(true) {
            dispatch();

        }
    });
}
