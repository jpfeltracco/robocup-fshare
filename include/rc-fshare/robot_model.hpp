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

        // K <<
            // 0.662839, 0.000665, -0.001269, -0.000357, -0.707104, 0.001403, -0.000075, -0.001331, 
            // 0.000665, 0.662839, -0.000357, -0.001269, -0.001403, -0.707104, 0.001331, 0.000070, 
            // -0.001269, -0.000357, 0.662666, 0.000690, 0.000070, -0.001331, -0.707104, 0.001259, 
            // -0.000357, -0.001269, 0.000690, 0.662666, 0.001331, -0.000075, -0.001259, -0.707104;


        // R = 5
        // Q = eye / 10^2, 0
        //     0, eye * 10^2
        K <<
            0.0996,     0.0195,   -0.0400,   -0.0051,   -3.8479,   -0.0645,    0.1876,   -0.0774,
            0.0195,     0.0996,   -0.0052,   -0.0400,   -0.1268,   -3.8469,   -0.0179,    0.1904,
            -0.0400,   -0.0051,    0.0938,    0.0212,    0.1912,   -0.0774,   -3.8157,   -0.0866,
            -0.0052,   -0.0400,    0.0213,    0.0938,   -0.0179,    0.1868,   -0.1432,   -3.8149;


        // R=10
            // 0.1551,    0.0254,   -0.0576,    0.0096,
            // 0.0249,    0.1549,    0.0101,   -0.0574,
           // -0.0576,    0.0096,    0.1456,    0.0306,
            // 0.0101,   -0.0575,    0.0301,    0.1454,

        // R=1
        // model.K <<
            // 0.2981,    0.0736,   -0.1656,   -0.0183,
            // 0.0726,    0.2978,   -0.0173,   -0.1653,
           // -0.1655,   -0.0183,    0.2737,    0.0813,
           // -0.0173,   -0.1654,    0.0803,    0.2735;

        // R = 3
        // model.K <<
            // 0.2310,    0.0468,   -0.1107,    0.0018,
            // 0.0460,    0.2308,    0.0025,   -0.1105,
           // -0.1107,    0.0018,    0.2138,    0.0540,
            // 0.0025,   -0.1105,    0.0533,    0.2137;

        // R = 5
        //model.K <<
        //    0.662839, 0.000665, -0.001269, -0.000357, -0.707104, 0.001403, -0.000075, -0.001331, 
        //0.000665, 0.662839, -0.000357, -0.001269, -0.001403, -0.707104, 0.001331, 0.000070, 
        //-0.001269, -0.000357, 0.662666, 0.000690, 0.000070, -0.001331, -0.707104, 0.001259, 
        //-0.000357, -0.001269, 0.000690, 0.662666, 0.001331, -0.000075, -0.001259, -0.707104;
            // 0.681282, 0.006431, -0.012350, -0.003381, -7.071043, 0.013475, -0.000721, -0.012791,
            // 0.006431, 0.681282, -0.003381, -0.012350, -0.013477, -7.071043, 0.012791, 0.000675,
            // -0.012350, -0.003381, 0.679594, 0.006689, 0.000675, -0.012791, -7.071046, 0.012106,
            // -0.003381, -0.012350, 0.006689, 0.679594, 0.012791, -0.000721, -0.012104, -7.071046;
            // 0.1981,    0.0367,   -0.0863,    0.0069,
            // 0.0360,    0.1979,    0.0076,   -0.0860,
           // -0.0862,    0.0069,    0.1844,    0.0432,
            // 0.0076,   -0.0861,    0.0426,    0.1842;

        // R = 20
        // model.K <<
            // 0.1162,    0.0167,   -0.0357,    0.0088,
            // 0.0164,    0.1161,    0.0092,   -0.0356,
           // -0.0357,    0.0088,    0.1101,    0.0203,
            // 0.0092,   -0.0356,    0.0200,    0.1100;

        // R = 1000
            // 0.0082,    0.0005,   -0.0009,    0.0004,
            // 0.0005,    0.0082,    0.0004,   -0.0009,
           // -0.0009,    0.0004,    0.0081,    0.0006,
            // 0.0004,   -0.0009,    0.0006,    0.0081;

        // R = 5
        // Q = eye / 10^2, 0
        //     0, eye * 10^2
        // model.K <<
            // 0.0996,     0.0195,   -0.0400,   -0.0051,   -3.8479,   -0.0645,    0.1876,   -0.0774,
            // 0.0195,     0.0996,   -0.0052,   -0.0400,   -0.1268,   -3.8469,   -0.0179,    0.1904,
            // -0.0400,   -0.0051,    0.0938,    0.0212,    0.1912,   -0.0774,   -3.8157,   -0.0866,
            // -0.0052,   -0.0400,    0.0213,    0.0938,   -0.0179,    0.1868,   -0.1432,   -3.8149;


    }
};
