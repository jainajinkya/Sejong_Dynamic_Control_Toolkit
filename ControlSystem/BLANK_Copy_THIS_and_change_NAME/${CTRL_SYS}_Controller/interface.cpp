#include "interface.hpp"
#include <stdio.h>
#include <math.h>

#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "StateProvider.hpp"

// TEST SET LIST
#include <TestSet/BodyCtrlTest.hpp>
#include <TestSet/JointCtrlTest.hpp>

#if MEASURE_TIME
#include <chrono>
#endif

interface::interface():
  count_(0), running_time_(0.0),
  torque_command_(NUM_ACT_JOINT),
  sensed_torque_(NUM_ACT_JOINT) {

  sensed_torque_.setZero();
  torque_command_.setZero();
  DataManager::GetDataManager()->RegisterData(&running_time_, DOUBLE, "running_time");
  DataManager::GetDataManager()->RegisterData(&sensed_torque_, SJ_VEC, "torque", NUM_ACT_JOINT);
  DataManager::GetDataManager()->RegisterData(&torque_command_, SJ_VEC, "command", NUM_ACT_JOINT);

  // TEST SETUP
  // test_ = new BodyCtrlTest();
  test_ = new JointCtrlTest();

  printf("[interface] Contruct\n");
  }

interface::~interface(){
  delete test_;
}

void interface::GetCommand(_DEF_SENSOR_DATA_,
                           std::vector<double> & command){
  if(!_Initialization(_VAR_SENSOR_DATA_)){
#if MEASURE_TIME
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
    state_estimator_.Update(_VAR_SENSOR_DATA_);
    // Calcualate Torque
    test_->getTorqueInput(torque_command_);

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
  // When there is sensed time
  StateProvider::GetStateProvider()->curr_time_ = time;
}
void interface::GetReactionForce(std::vector<sejong::Vect3> & reaction_force ){

}

bool interface::_Initialization(_DEF_SENSOR_DATA_){
  if(count_ < 2){
    torque_command_.setZero();
    state_estimator_.Initialization(_VAR_SENSOR_DATA_);
    test_->TestInitialization();

    return true;
  }
  DataManager::GetDataManager()->start();
  return false;
}
