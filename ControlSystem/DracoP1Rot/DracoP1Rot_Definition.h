#ifndef DRACO_DEFINITION
#define DRACO_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>

#define MEASURE_TIME 0

#define _DEF_SENSOR_DATA_ double time, const std::vector<double> & jpos, const std::vector<double> & jvel, const std::vector<double> & torque, const std::vector<double> & body_pos, const std::vector<double> & body_vel, const double & body_ori, const double & body_ang_vel

#define _VAR_SENSOR_DATA_ time, jpos, jvel, torque, body_pos, body_vel, body_ori, body_ang_vel


enum SJLinkID{
  LK_body = 0,
  LK_upperLeg,
  LK_lowerLeg,
  LK_foot,
  LK_FootToe,
  LK_FootHeel,
  NUM_LINK
};

enum SJJointID{
  VIRTUAL_X = 0,
  VIRTUAL_Z,
  VIRTUAL_Ry,
  bodyPitch,
  kneePitch,
  anklePitch,
  NUM_JOINT
};

#define NUM_QDOT 6
#define NUM_VIRTUAL 3
#define NUM_Q NUM_QDOT
#define NUM_ACT_JOINT (NUM_QDOT - NUM_VIRTUAL)

#endif
