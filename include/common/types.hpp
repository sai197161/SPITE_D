#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace spite_d {

using Mat3 = std::array<std::array<double, 3>, 3>;
using Vec3 = std::array<double, 3>;

//rotation + translation of obstacles
struct Pose3 { 
    Mat3 rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    Vec3 translation{0, 0, 0};
};

// single tracked obstacle, produce by perception module (detection and trajectory)
struct TrackedObstacle {
    int32_t id{-1};
    double stamp{0.0}; // seconds, source clock = sensor
    Pose3 pose;
    Vec3 halfExtents{0, 0, 0}; 
    Vec3 velocity{0, 0, 0};
    Vec3 positionStd{0, 0, 0};
};

// future trajectory for single tracked obstacle, from trajectory module
struct PredictedTrajectory {
    int32_t id{-1};
    Vec3 halfExtents{0, 0, 0};
    std::vector<double> stamps; //absolute time
    std::vector<Pose3> poses;
    std::vector<Vec3> positionStd;
};


}