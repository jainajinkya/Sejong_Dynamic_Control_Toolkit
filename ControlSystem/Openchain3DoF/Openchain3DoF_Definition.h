#ifndef OPENCHAIN_3DOF_DEFINITION
#define OPENCHAIN_3DOF_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>

#define MEASURE_TIME 0

#define _DEF_SENSOR_DATA_ double time, const std::vector<double> & jpos, const std::vector<double> & jvel, const std::vector<double> & torque

#define _VAR_SENSOR_DATA_ time, jpos, jvel, torque

enum SJLinkID{
  LK_L1 = 0,
  LK_L2,
  LK_L3,
  NUM_LINK,
  LK_EE
};

enum SJJointID{
  J1 = 0,
  J2,
  J3,
  NUM_JOINT
};

#define NUM_QDOT 3
#define NUM_VIRTUAL 0
#define NUM_Q NUM_QDOT
#define NUM_ACT_JOINT (NUM_QDOT - NUM_VIRTUAL)

#endif
