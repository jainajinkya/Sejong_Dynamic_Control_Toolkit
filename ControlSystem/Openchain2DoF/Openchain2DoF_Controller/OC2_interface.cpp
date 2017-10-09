#include "OC2_interface.hpp"
#include <stdio.h>

#include <math.h>

#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>

#include <Openchain2DoF_Model/OC2_Model.hpp>
#include "OC2System.hpp"
#include "StateProvider.hpp"
#include <stdio.h>
#if MEASURE_TIME
#include <chrono>
#endif

OC2_interface::OC2_interface():
  count_(0), running_time_(0.0),
  initial_jpos_(NUM_ACT_JOINT),
  torque_command_(NUM_ACT_JOINT),
  sensed_torque_(NUM_ACT_JOINT) {

  oc2_sys_ = new OC2System();
  printf("[OC2_interface] Contruct\n");
}

OC2_interface::~OC2_interface(){
}

void OC2_interface::GetCommand(_DEF_SENSOR_DATA_,
                               std::vector<double> & command){
  if(!_Initialization(_VAR_SENSOR_DATA_)){
    state_estimator_.Update(_VAR_SENSOR_DATA_);
#if MEASURE_TIME
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
    // Calcualate Torque
    oc2_sys_->getTorqueInput(torque_command_);

#if MEASURE_TIME
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
    std::cout << "All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;;
#endif

    for (int i(0); i<NUM_ACT_JOINT; ++i){
      command[i] = torque_command_[i];
      sensed_torque_[i] = torque[i];
    }
  }
  running_time_ = (double)(count_) * SERVO_RATE;
  ++count_;
  StateProvider::GetStateProvider()->curr_time_ = time;
}

bool OC2_interface::_Initialization(_DEF_SENSOR_DATA_){
  if(count_ < 1){
    torque_command_.setZero();
    state_estimator_.Initialization(_VAR_SENSOR_DATA_);
    oc2_sys_->Initialization();
    for (int i(0); i<NUM_ACT_JOINT; ++i){
      initial_jpos_[i] = jpos[i];
    }
    return true;
  }
  DataManager::GetDataManager()->start();
  return false;
}
