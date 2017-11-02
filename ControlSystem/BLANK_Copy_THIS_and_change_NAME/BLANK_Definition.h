#ifndef VALKYRIE_DEFINITION
#define VALKYRIE_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>

#define MEASURE_TIME 0

#define _DEF_SENSOR_DATA_ double time, const std::vector<double> & jpos, const std::vector<double> & jvel, const std::vector<double> & torque, const sejong::Vect3 & body_pos, const sejong::Quaternion & body_ori, const sejong::Vect3 & body_vel, const sejong::Vect3 & ang_vel

#define _VAR_SENSOR_DATA_ time, jpos, jvel, torque, body_pos, body_ori, body_vel, ang_vel


enum SJLinkID{
    LK_pelvis = 0,
    LK_leftHipYawLink,
    LK_leftHipRollLink,
    LK_leftHipPitchLink,
    LK_leftKneePitchLink,
    LK_leftAnklePitchLink,
    LK_leftFoot,
    LK_rightHipYawLink,
    LK_rightHipRollLink,
    LK_rightHipPitchLink,
    LK_rightKneePitchLink,
    LK_rightAnklePitchLink,
    LK_rightFoot,
    LK_torsoYawLink,
    LK_torsoPitchLink,
    LK_torso = 15,
    LK_leftShoulderPitchLink,
    LK_leftShoulderRollLink,
    LK_leftShoulderYawLink,
    LK_leftElbowPitchLink,
    LK_leftForearmLink,
    /* LK_leftWristRollLink, */
    /* LK_leftPalm, */
    LK_lowerNeckPitchLink,
    LK_neckYawLink,
    LK_upperNeckPitchLink,
    LK_rightShoulderPitchLink,
    LK_rightShoulderRollLink,
    LK_rightShoulderYawLink,
    LK_rightElbowPitchLink,
    LK_rightForearmLink,
    /* LK_rightWristRollLink, */
    /* LK_rightPalm, */
    NUM_LINK,
    LK_leftCOP_Frame,
    LK_rightCOP_Frame,
    LK_leftFootOutFront,
    LK_leftFootOutBack,
    LK_leftFootInBack,
    LK_leftFootInFront,
    LK_rightFootOutFront,
    LK_rightFootOutBack,
    LK_rightFootInBack,
    LK_rightFootInFront
};

enum SJJointID{
    VIRTUAL_X = 0,
    VIRTUAL_Y = 1,
    VIRTUAL_Z = 2,
    VIRTUAL_Rx = 3,
    VIRTUAL_Ry = 4,
    VIRTUAL_Rz = 5,
    leftHipYaw = 6,
    leftHipRoll = 7  ,
    leftHipPitch = 8 ,
    leftKneePitch = 9 ,
    leftAnklePitch = 10,
    leftAnkleRoll = 11,
    rightHipYaw   ,
    rightHipRoll  ,
    rightHipPitch ,
    rightKneePitch,
    rightAnklePitch,
    rightAnkleRoll       ,
    torsoYaw             ,
    torsoPitch           ,
    torsoRoll            ,
    leftShoulderPitch = 21,
    leftShoulderRoll     ,
    leftShoulderYaw      ,
    leftElbowPitch       ,
    leftForearmYaw = 25  ,
    /* leftWristRoll        , */
    /* leftWristPitch       , */
    lowerNeckPitch  , 
    neckYaw         , 
    upperNeckPitch  , 
    rightShoulderPitch = 29, 
    rightShoulderRoll , 
    rightShoulderYaw  , 
    rightElbowPitch   , 
    rightForearmYaw = 33 , 
    /* rightWristRoll    ,  */
    /* rightWristPitch    */
};

#define NUM_QDOT 34
#define NUM_VIRTUAL 6
#define NUM_Q (NUM_QDOT + 1)
#define NUM_ACT_JOINT (NUM_QDOT - NUM_VIRTUAL)

#endif
