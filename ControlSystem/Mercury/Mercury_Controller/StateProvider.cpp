#include "StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include <Robot_Model/RobotModel.hpp>

StateProvider* StateProvider::GetStateProvider(){
    static StateProvider state_provider_;
    return &state_provider_;
}

StateProvider::StateProvider(): initialized_(false),
                                system_count_(0),
                                stance_foot_(SJLinkID::LK_LFOOT),
                                Q_(NUM_Q),
                                Qdot_(NUM_QDOT),
                                reaction_forces_(6)
{
  body_ang_vel_.setZero();
  Q_.setZero();
  Qdot_.setZero();
  reaction_forces_.setZero();
  global_pos_local_.setZero();
  des_location_.setZero();

  Rfoot_pos_.setZero();
  Lfoot_pos_.setZero();
  Rfoot_vel_.setZero();
  Lfoot_vel_.setZero();

  CoM_pos_.setZero();
  CoM_vel_.setZero();


  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, SJ_VEC, "config", NUM_Q);
  data_manager->RegisterData(&Qdot_, SJ_VEC, "qdot", NUM_QDOT);
  data_manager->RegisterData(&reaction_forces_, SJ_VEC, "reaction_force", 6);

  data_manager->RegisterData(&Rfoot_pos_, VECT3, "rfoot_pos", 3);
  data_manager->RegisterData(&Rfoot_vel_, VECT3, "rfoot_vel", 3);
  data_manager->RegisterData(&Lfoot_pos_, VECT3, "lfoot_pos", 3);
  data_manager->RegisterData(&Lfoot_vel_, VECT3, "lfoot_vel", 3);

  data_manager->RegisterData(&CoM_pos_, VECT3, "com_pos", 3);
  data_manager->RegisterData(&CoM_vel_, VECT3, "com_vel", 3);
}


void StateProvider::SaveCurrentData(){
  RobotModel* model = RobotModel::GetRobotModel();

  model->getPosition(Q_, LK_RFOOT, Rfoot_pos_);
  model->getPosition(Q_, LK_LFOOT, Lfoot_pos_);

  model->getVelocity(Q_, Qdot_, LK_RFOOT, Rfoot_vel_);
  model->getVelocity(Q_, Qdot_, LK_LFOOT, Lfoot_vel_);

  model->getCoMPosition(Q_, CoM_pos_);
  model->getCoMVelocity(Q_, Qdot_, CoM_vel_);
}
