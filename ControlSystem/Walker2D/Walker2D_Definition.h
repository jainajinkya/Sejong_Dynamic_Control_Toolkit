#ifndef WALKER_2D_DEFINITION
#define WALKER_2D_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>

#define MEASURE_TIME 0
#define CONFIG_PATH THIS_COM""

#define _DEF_SENSOR_DATA_ double time, const std::vector<double> & jpos, const std::vector<double> & jvel, const std::vector<double> & torque, const std::vector<double> & body_pos, const std::vector<double> & body_vel, const double & body_ori, const double & body_ang_vel

#define _VAR_SENSOR_DATA_ time, jpos, jvel, torque, body_pos, body_vel, body_ori, body_ang_vel

enum SJLinkID{
  LK_BODY = 0,
  LK_LEFT_THIGH,
  LK_LEFT_SHANK,
  LK_RIGHT_THIGH,
  LK_RIGHT_SHANK,
  NUM_LINK,
  LK_LEFT_FOOT,
  LK_RIGHT_FOOT
};

enum SJJointID{
  VIRTUAL_X = 0,
  VIRTUAL_Z,
  VIRTUAL_Ry,
  LEFT_HIP,
  LEFT_KNEE,
  RIGHT_HIP,
  RIGHT_KNEE,
  NUM_JOINT
};

#define NUM_QDOT 7
#define NUM_VIRTUAL 3
#define NUM_Q NUM_QDOT
#define NUM_ACT_JOINT (NUM_QDOT - NUM_VIRTUAL)

#endif
