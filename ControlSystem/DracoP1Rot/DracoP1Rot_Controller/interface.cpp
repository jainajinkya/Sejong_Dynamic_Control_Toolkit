#include "interface.hpp"
#include <stdio.h>
#include <math.h>

#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>

#include <TestSet/BodyCtrlTest.hpp>
#include "StateProvider.hpp"
#include <stdio.h>

#if MEASURE_TIME
#include <chrono>
#endif

Interface::Interface():
  count_(0), running_time_(0.0),
  torque_command_(NUM_ACT_JOINT),
  sensed_torque_(NUM_ACT_JOINT) {

  draco_test_ = new BodyCtrlTest();
  printf("[Interface] Contruct\n");
}

Interface::~Interface(){
}

void Interface::GetCommand(_DEF_SENSOR_DATA_,
                           std::vector<double> & command){
  if(!_Initialization(_VAR_SENSOR_DATA_, command)){
    state_estimator_.Update(_VAR_SENSOR_DATA_);

#if MEASURE_TIME
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
    // Calcualate Torque
    draco_test_->getTorqueInput(torque_command_);

#if MEASURE_TIME
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
    std::cout << "All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;;
#endif

  }
  for (int i(0); i<NUM_ACT_JOINT; ++i){
    command[i] = torque_command_[i];
    sensed_torque_[i] = torque[i];
  }

  running_time_ = (double)(count_) * SERVO_RATE;
  ++count_;
  StateProvider::GetStateProvider()->curr_time_ = time;
}
void Interface::GetReactionForce(std::vector<sejong::Vect3> & reaction_force ){

}

bool Interface::_Initialization(_DEF_SENSOR_DATA_, std::vector<double> & command){
  if(count_ < 1){
    torque_command_.setZero();
    state_estimator_.Initialization(_VAR_SENSOR_DATA_);
    draco_test_->TestInitialization();
    return true;
  }
  DataManager::GetDataManager()->start();
  return false;
}
