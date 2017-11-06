#include "StateProvider.hpp"
#include <Utils/DataManager.hpp>

StateProvider* StateProvider::GetStateProvider(){
    static StateProvider state_provider_;
    return &state_provider_;
}

StateProvider::StateProvider(): initialized_(false),
                                system_count_(0),
                                stance_foot_(SJLinkID::LK_RFOOT),
                                Q_(NUM_Q),
                                Qdot_(NUM_QDOT)
{
  Q_.setZero();
  Qdot_.setZero();

  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, SJ_VEC, "config", NUM_Q);
  data_manager->RegisterData(&Qdot_, SJ_VEC, "qdot", NUM_QDOT);
  // Body
  data_manager->RegisterData(&Body_pos_, VECT3, "body_pos", 3);
  data_manager->RegisterData(&Body_vel_, VECT3, "body_vel", 3);
  data_manager->RegisterData(&Body_pos_des_, VECT3, "body_pos_des", 3);
  data_manager->RegisterData(&Body_vel_des_, VECT3, "body_vel_des", 3);
  data_manager->RegisterData(&Body_acc_des_, VECT3, "body_acc_des", 3);
}
