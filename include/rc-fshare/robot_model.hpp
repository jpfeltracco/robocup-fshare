#pragma once

#ifndef EIGEN_HAS_CXX11_MATH
#define EIGEN_HAS_CXX11_MATH 0
#endif
#include <Eigen/Dense>
#include <array>
#include <cmath>
//#include "const-math.hpp"


/// Model parameters for a robot.  Used by the controls system.
class RobotModel {

static constexpr double DegToRad(double val) { return val * M_PI / 180.0; }

public:
    using EncReading = Eigen::Matrix<int32_t, 4, 1>;

    // singleton pattern
    static RobotModel& get() {
        static RobotModel instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }
    RobotModel(RobotModel const&) = delete;
    void operator=(RobotModel const&) = delete;

public:
    // Wheel angles (in radians) measured between +x axis and wheel axle
    std::array<double, 4> WheelAngles;
    // Radius of omni-wheel (in meters)
    double WheelRadius;
    // Distance from center of robot to center of wheel
    double WheelDist;

    // Made up lower level motor constant
    // duty_cycle * multiplier == motor vel
    double DutyCycleMultiplier;

    // Number of encoder increments for every full revolution
    int32_t EncTicksPerRev;

    // wheelSpeeds = BotToWheel * V_bot
    Eigen::Matrix<double, 4, 3> BotToWheel;
    // botSpeed = WheelToBot * wheelSpeeds
    Eigen::Matrix<double, 3, 4> WheelToBot;
    // botSpeed = EncToBot * EncoderReading
    Eigen::Matrix<double, 3, 4> EncToBot;

private:
    RobotModel()
    {
        WheelAngles = {
            DegToRad(180 - 30),  // M1
            DegToRad(180 + 39),  // M2
            DegToRad(360 - 39),  // M3
            DegToRad(0 + 30),    // M4
        };

        WheelRadius = 0.02786;
        WheelDist = 0.0798576;
        EncTicksPerRev = 2048 * 3;

        DutyCycleMultiplier = 2.0;

        // http://people.idsia.ch/~foerster/2006/1/omnidrive_kiart_preprint.pdf
        BotToWheel <<
            -sinf(WheelAngles[0]), cosf(WheelAngles[0]), WheelDist,
            -sinf(WheelAngles[1]), cosf(WheelAngles[1]), WheelDist,
            -sinf(WheelAngles[2]), cosf(WheelAngles[2]), WheelDist,
            -sinf(WheelAngles[3]), cosf(WheelAngles[3]), WheelDist;
        // Invert because our wheels spin opposite to paper
        BotToWheel *= -1;
        BotToWheel /= WheelRadius;

        WheelToBot = (BotToWheel.transpose() * BotToWheel).inverse() * BotToWheel.transpose();

        EncToBot = WheelToBot * 2.0*M_PI / EncTicksPerRev;
    }

};
