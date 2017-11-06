#ifndef ROBOT_DEFINITION
#define ROBOT_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>

#define MEASURE_TIME 0

#define _DEF_SENSOR_DATA_ double time, const std::vector<double> & jpos, const std::vector<double> & jvel, const std::vector<double> & torque, const std::vector<double> & imu_acc, const std::vector<double> & imu_ang_vel

#define _VAR_SENSOR_DATA_ time, jpos, jvel, torque, imu_acc, imu_ang_vel


enum SJLinkID{
    LK_base = 0,
    NUM_LINK,
};

enum SJJointID{
    leftHipYaw = 0,
};

#define NUM_QDOT 1
#define NUM_VIRTUAL 6
#define NUM_Q (NUM_QDOT + 1)
#define NUM_ACT_JOINT (NUM_QDOT - NUM_VIRTUAL)

#endif
