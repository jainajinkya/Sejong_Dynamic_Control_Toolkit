#include "ReactionForceCalculator.h"
#include <stdio.h>
#include <iostream>

ReactionForceCalculator::ReactionForceCalculator(WBLC_Model* robot_model):
  cm_acc_(6),
  CoM_acc_(3),
  CAM_acc_(3),
  cm_pos_(6),
  cm_vel_(6),
  Kp_(3),
  Kv_(3),
  b_setCoM_(false), b_prepared_(false)
{
  robot_model_ = robot_model;
  // CoM x, y, z
  Kp_[0] = 50.0;
  Kp_[1] = 50.0;
  Kp_[2] = 220.0;

  Kv_[0] = 15.0;
  Kv_[1] = 15.0;
  Kv_[2] = 10.0;
}

ReactionForceCalculator::~ReactionForceCalculator(){
}
void ReactionForceCalculator::PrintName(){
  printf("[Reactio Force Calculator] Parent \n");
}

bool ReactionForceCalculator::GetReactionForce(sejong::Vector & Fr, bool update){
  if (update){
    _PreProcessing();

    bool result_;
    if(b_setCoM_ && b_prepared_){
      result_ = _CalculateReactionForce(Fr);
      Fr_ = Fr;
    }
    if(!b_setCoM_){
      result_ = false;
      printf("[Reaction Force Calculator] CoM setting is not done \n");
    }
    if(!b_prepared_){
      result_ = false;
      printf("[Reaction Force Calculator] Optimization Parameters are not prepared \n");
    }
    _PostProcessing();
    return result_;
  } else {
    Fr = Fr_;
    return true;
  }
}
void ReactionForceCalculator::SetCentroidConfiguration(const sejong::Vector & cent_conf,
                                                       const sejong::Vector & cent_vel){
  des_ff_ = sejong::Vector::Zero(3);
  SetCentroidConfiguration(des_ff_, cent_conf, cent_vel);
}

void ReactionForceCalculator::SetCentroidConfiguration(const sejong::Vector & des_ff,
                                                       const sejong::Vector & cent_conf,
                                                       const sejong::Vector & cent_vel){
  des_ff_ = des_ff;
  des_conf_ = cent_conf;
  des_vel_ = cent_vel;
  b_setCoM_ = true;
}

void ReactionForceCalculator::_PreProcessing(){
  _SetContactJacobian();
}

void ReactionForceCalculator::_PostProcessing(){
  b_setCoM_ = false;
  b_prepared_ = false;
}
void ReactionForceCalculator::getCentroidData(sejong::Vector & cm_pos,
                                              sejong::Vector & cm_vel,
                                              sejong::Vector & cm_pos_des,
                                              sejong::Vector & cm_vel_des,
                                              sejong::Vector & cm_acc_des){
  cm_pos = cm_pos_;
  cm_vel = cm_vel_;

  cm_pos_des = des_conf_;
  cm_vel_des = des_vel_;
  cm_acc_des = des_ff_;
}
