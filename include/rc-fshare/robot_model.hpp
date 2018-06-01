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

    double V_Max;
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 4> B;
    Eigen::Matrix<double, 4, 4> PinvB;
    Eigen::Matrix<double, 4, 8> K;

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

        V_Max = 18.5;

        A <<
            0.8122,   0.0660,  -0.1223,   0.0452,
            0.0616,   0.8107,   0.0493,  -0.1207,
           -0.1220,   0.0452,   0.7903,   0.0807,
            0.0494,  -0.1210,   0.0767,   0.7892;

        B <<
            3.9296,  -1.3279,   2.5316,  -0.9867,
           -1.3311,   3.9296,  -0.9834,   2.5319,
            2.5319,  -0.9867,   4.3815,  -1.6391,
           -0.9834,   2.5316,  -1.6424,   4.3815;

        PinvB <<
            0.4454,   0.1327,  -0.2544,  -0.0716,
            0.1342,   0.4454,  -0.0729,  -0.2545,
           -0.2545,  -0.0716,   0.4108,   0.1377,
           -0.0729,  -0.2544,   0.1390,   0.4108;

        K <<
            0.662839, 0.000665, -0.001269, -0.000357, -0.707104, 0.001403, -0.000075, -0.001331, 
            0.000665, 0.662839, -0.000357, -0.001269, -0.001403, -0.707104, 0.001331, 0.000070, 
            -0.001269, -0.000357, 0.662666, 0.000690, 0.000070, -0.001331, -0.707104, 0.001259, 
            -0.000357, -0.001269, 0.000690, 0.662666, 0.001331, -0.000075, -0.001259, -0.707104;


    }
};
