#include <memory>
#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <math.h>

using namespace webots;

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// Basic teleop control
// Refactored from: https://github.com/cyberbotics/webots/blob/master/projects/robots/dji/mavic/controllers/mavic2pro/mavic2pro.c
class Teleop {
    public:
        Teleop(Robot* robot) : 
            robot(robot), 
            imu(robot->getInertialUnit("inertial unit")),
            gyro(robot->getGyro("gyro")),
            gps(robot->getGPS("gps")),
            kb(std::make_unique<Keyboard>()),
            front_left_motor(robot->getMotor("front left propeller")),
            front_right_motor(robot->getMotor("front right propeller")),
            rear_left_motor(robot->getMotor("rear left propeller")),
            rear_right_motor(robot->getMotor("rear right propeller"))
        { 
            
            auto dt = robot->getBasicTimeStep();
            imu->enable(dt);
            gyro->enable(dt);
            gps->enable(dt);
            kb->enable(dt);

            front_left_motor->setPosition(INFINITY);
            front_right_motor->setPosition(INFINITY);
            rear_left_motor->setPosition(INFINITY);
            rear_right_motor->setPosition(INFINITY);
        };

        void keyboard_ctrl() {
            // Retrieve robot position using the sensors.
            auto imu_data = imu->getRollPitchYaw();
            const double roll = imu_data[0];
            const double pitch = imu_data[1];
            const double altitude = gps->getValues()[2];
            auto gyro_data = gyro->getValues();
            const double roll_velocity = gyro_data[0];
            const double pitch_velocity = gyro_data[1];

            // Transform the keyboard input to disturbances on the stabilization algorithm.
            double roll_disturbance = 0.0;
            double pitch_disturbance = 0.0;
            double yaw_disturbance = 0.0;
            int key = kb->getKey();
            while (key > 0) {
                switch (key) {
                    case Keyboard::UP:
                    pitch_disturbance = -2.0;
                    break;
                    case Keyboard::DOWN:
                    pitch_disturbance = 2.0;
                    break;
                    case Keyboard::RIGHT:
                    yaw_disturbance = -1.3;
                    break;
                    case Keyboard::LEFT:
                    yaw_disturbance = 1.3;
                    break;
                    case (Keyboard::SHIFT + Keyboard::RIGHT):
                    roll_disturbance = -1.0;
                    break;
                    case (Keyboard::SHIFT + Keyboard::LEFT):
                    roll_disturbance = 1.0;
                    break;
                    case (Keyboard::SHIFT + Keyboard::UP):
                    target_altitude += 0.05;
                    printf("target altitude: %f [m]\n", target_altitude);
                    break;
                    case (Keyboard::SHIFT + Keyboard::DOWN):
                    target_altitude -= 0.05;
                    printf("target altitude: %f [m]\n", target_altitude);
                    break;
                }
                key = kb->getKey();
            }
            // Compute the roll, pitch, yaw and vertical inputs.
            const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
            const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
            const double yaw_input = yaw_disturbance;
            const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
            const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0); 

            // Actuate the motors taking into consideration all the computed inputs.
            const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
            const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
            const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
            const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
            front_left_motor->setVelocity(front_left_motor_input);
            front_right_motor->setVelocity(-front_right_motor_input);
            rear_left_motor->setVelocity( -rear_left_motor_input);
            rear_right_motor->setVelocity( rear_right_motor_input); 
        };

    private:
        Robot* robot;
        InertialUnit* imu;
        Gyro* gyro;
        GPS* gps;
        std::unique_ptr<Keyboard> kb;

        Motor* front_left_motor;
        Motor* front_right_motor;
        Motor* rear_left_motor;
        Motor* rear_right_motor;

        // Constants, empirically found.
        const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
        const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
        const double k_vertical_p = 3.0;        // P constant of the vertical PID.
        const double k_roll_p = 50.0;           // P constant of the roll PID.
        const double k_pitch_p = 30.0;          // P constant of the pitch PID.
        double target_altitude = 1.0;  // The target altitude. Can be changed by the user.
};
