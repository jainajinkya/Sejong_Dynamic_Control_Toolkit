#include "StateProvider.hpp"
#include <Utils/DataManager.hpp>

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
  Q_.setZero();
  Qdot_.setZero();
  reaction_forces_.setZero();

  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, SJ_VEC, "config", NUM_Q);
  data_manager->RegisterData(&Qdot_, SJ_VEC, "qdot", NUM_QDOT);
  data_manager->RegisterData(&reaction_forces_, SJ_VEC, "reaction_force", 6);

  // Body
  // data_manager->RegisterData(&Body_pos_, VECT3, "body_pos", 3);
  // data_manager->RegisterData(&Body_vel_, VECT3, "body_vel", 3);
  // data_manager->RegisterData(&Body_pos_des_, VECT3, "body_pos_des", 3);
  // data_manager->RegisterData(&Body_vel_des_, VECT3, "body_vel_des", 3);
  // data_manager->RegisterData(&Body_acc_des_, VECT3, "body_acc_des", 3);
}
