#ifndef MERCURY_DEFINITION
#define MERCURY_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>

#define MEASURE_TIME 0

#define _DEF_SENSOR_DATA_ double time, const std::vector<double> & jpos, const std::vector<double> & jvel, const std::vector<double> & torque, const std::vector<double> & imu_acc, const std::vector<double> & imu_ang_vel, bool rfoot_contact, bool lfoot_contact

#define _VAR_SENSOR_DATA_ time, jpos, jvel, torque, imu_acc, imu_ang_vel, rfoot_contact, lfoot_contact

#define CONFIG_PATH THIS_COM"ControlSystem/Mercury/MercuryTestConfig/"

enum SJLinkID{
  LK_Body,
  LK_RAbduct,
  LK_RThigh,
  LK_RCalf,
  LK_LAbduct,
  LK_LThigh,
  LK_LCalf,
  LK_LFOOT,
  LK_RFOOT,
  LK_IMU,
  LED_BODY_0, 
  LED_BODY_1, 
  LED_BODY_2, 
  LED_BODY_3, 
    
  LED_LEG1_0,
  LED_LEG1_1,
  LED_LEG1_2,
    
  LED_LEG2_0,
  LED_LEG2_1
};

enum SJJointID{
  // Right
  RAbduction = 0,
  RHip = 1,
  RKnee = 2,
  // Left
  LAbduction = 3,
  LHip = 4,
  LKnee = 5,
};

#define NUM_ACT_JOINT 6
#define NUM_VIRTUAL 6
#define NUM_QDOT (NUM_VIRTUAL + NUM_ACT_JOINT)
#define NUM_Q (NUM_QDOT + 1)

#endif
