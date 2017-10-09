#include "StateProvider.hpp"
#include <Utils/DataManager.hpp>

StateProvider* StateProvider::GetStateProvider(){
    static StateProvider state_provider_;
    return &state_provider_;
}

StateProvider::StateProvider(): initialized_(false),
                                system_count_(0),
                                // Arbitrary Large number
                                Q_(NUM_Q),
                                Qdot_(NUM_QDOT)
{
  Q_.setZero();
  Qdot_.setZero();

  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, SJ_VEC, "config", NUM_Q);
  data_manager->RegisterData(&Qdot_, SJ_VEC, "qdot", NUM_QDOT);
  // CoM
  data_manager->RegisterData(&CoM_pos_, VECT3, "com_pos", 3);
  data_manager->RegisterData(&CoM_vel_, VECT3, "com_vel", 3);
  data_manager->RegisterData(&CoM_pos_des_, VECT3, "com_pos_des", 3);
  data_manager->RegisterData(&CoM_vel_des_, VECT3, "com_vel_des", 3);
  data_manager->RegisterData(&CoM_acc_des_, VECT3, "com_acc_des", 3);
}
