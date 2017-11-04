#ifndef DEFINITION_HUME
#define DEFINITION_HUME

#include <string>
// Simulation 3D drawing
#define CAD_FILE_DIMENSION_RATIO 0.001

// Setting
#define TORQUE_LIMIT 500.0
#define TORQUE_LIMIT_AIR 150.0
#define TORQUE_LIMIT_KNEE 500.0
#define TORQUE_LIMIT_ABDUCTION 50.0

#define M_ABDUCTION 2.1656 
#define M_THIGH     2.93 //2.4194
#define M_CALF      0.33
#define M_BODY      10.61

#define ROTOR_INERTIA_Abduction 0.13572
#define ROTOR_INERTIA_Hip  0.0872756
#define ROTOR_INERTIA_Knee 0.0872756

#define PREPARING_COUNT 5 //Simulation
// #define PREPARING_COUNT 6000

#define FLOATING_PT 10000.0

#define NUM_Q 13
#define NUM_QDOT 12
#define NUM_VIRTUAL 6

#define _DEF_SENSOR_DATA_ const std::vector<double> & jpos, const std::vector<double> & torque, const std::vector<double> & motor_theta, const std::vector<double> & ang_vel, const std::vector<double> & accelerometer, const std::vector<double> & magnetometer, bool _left_foot_contact, bool _right_foot_contact

#define _VAR_SENSOR_DATA_ jpos, torque, motor_theta, ang_vel, accelerometer, magnetometer, _left_foot_contact, _right_foot_contact

// Quaternion x, y, z, w

enum SJLinkID{ 
    HIP,
    RAbduct,
    RThigh,
    RCalf,
    LAbduct,
    LThigh,
    LCalf,
    LFOOT,
    RFOOT,
    IMU,
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

enum Active{
    // Right
    RAbduction = 0,
    RHip = 1,
    RKnee = 2,
    // Left
    LAbduction = 3,
    LHip = 4,
    LKnee = 5,
    NUM_ACT_JOINT = 6
};

inline SJLinkID FindLinkID(std::string const & str_id){
    if(str_id == "LFOOT")  return LFOOT;
    if(str_id == "RFOOT")  return RFOOT;
}

#endif
