#include "interface.h"
#include <stdio.h>

#include <math.h>

#include <Utils/utilities.h>
#include <Utils/DataManager.h>
#include <Utils/wrap_eigen.hpp>

#include <Valkyrie_Model/Valkyrie_Model.h>
#include "ValkyrieSystem.h"
#include "ValkyrieController.h"
#include "StateProvider.h"
#include <stdio.h>
#if MEASURE_TIME
#include <chrono>
#endif
Interface::Interface():
    count_(0), running_time_(0.0),
    initial_jpos_(NUM_ACT_JOINT),
    torque_command_(NUM_ACT_JOINT),
    sensed_torque_(NUM_ACT_JOINT) {
  sensed_torque_.setZero();
  torque_command_.setZero();
    DataManager::GetDataManager()->RegisterData(&sensed_torque_, SJ_VEC, "torque", NUM_ACT_JOINT);
    DataManager::GetDataManager()->RegisterData(&torque_command_, SJ_VEC, "command", NUM_ACT_JOINT);

    // joint_idx_map["leftHipYaw"]           = leftHipYaw        - NUM_VIRTUAL; 
    // joint_idx_map["leftHipRoll"]          = leftHipRoll       - NUM_VIRTUAL; 
    // joint_idx_map["leftHipPitch"]         = leftHipPitch      - NUM_VIRTUAL; 
    // joint_idx_map["leftKneePitch"]        = leftKneePitch     - NUM_VIRTUAL; 
    // joint_idx_map["leftAnklePitch"]       = leftAnklePitch    - NUM_VIRTUAL; 
    // joint_idx_map["leftAnkleRoll"]        = leftAnkleRoll     - NUM_VIRTUAL; 
    // joint_idx_map["rightHipYaw"]          = rightHipYaw       - NUM_VIRTUAL; 
    // joint_idx_map["rightHipRoll"]         = rightHipRoll      - NUM_VIRTUAL; 
    // joint_idx_map["rightHipPitch"]        = rightHipPitch     - NUM_VIRTUAL; 
    // joint_idx_map["rightKneePitch"]       = rightKneePitch    - NUM_VIRTUAL; 
    // joint_idx_map["rightAnklePitch"]      = rightAnklePitch   - NUM_VIRTUAL; 
    // joint_idx_map["rightAnkleRoll"]       = rightAnkleRoll    - NUM_VIRTUAL; 
    // joint_idx_map["torsoYaw"]             = torsoYaw          - NUM_VIRTUAL; 
    // joint_idx_map["torsoPitch"]           = torsoPitch        - NUM_VIRTUAL; 
    // joint_idx_map["torsoRoll"]            = torsoRoll         - NUM_VIRTUAL; 
    // joint_idx_map["leftShoulderPitch"]    = leftShoulderPitch - NUM_VIRTUAL; 
    // joint_idx_map["leftShoulderRoll"]     = leftShoulderRoll  - NUM_VIRTUAL; 
    // joint_idx_map["leftShoulderYaw"]      = leftShoulderYaw   - NUM_VIRTUAL; 
    // joint_idx_map["leftElbowPitch"]       = leftElbowPitch    - NUM_VIRTUAL; 
    // joint_idx_map["leftForearmYaw"]       = leftForearmYaw    - NUM_VIRTUAL; 
    // joint_idx_map["leftWristRoll"]        = leftWristRoll     - NUM_VIRTUAL; 
    // joint_idx_map["leftWristPitch"]       = leftWristPitch    - NUM_VIRTUAL; 
    // joint_idx_map["lowerNeckPitch"]       = lowerNeckPitch    - NUM_VIRTUAL; 
    // joint_idx_map["neckYaw"]              = neckYaw           - NUM_VIRTUAL; 
    // joint_idx_map["upperNeckPitch"]       = upperNeckPitch    - NUM_VIRTUAL; 
    // joint_idx_map["rightShoulderPitch"]   = rightShoulderPitch- NUM_VIRTUAL; 
    // joint_idx_map["rightShoulderRoll"]    = rightShoulderRoll - NUM_VIRTUAL; 
    // joint_idx_map["rightShoulderYaw"]     = rightShoulderYaw  - NUM_VIRTUAL; 
    // joint_idx_map["rightElbowPitch"]      = rightElbowPitch   - NUM_VIRTUAL; 
    // joint_idx_map["rightForearmYaw"]      = rightForearmYaw   - NUM_VIRTUAL; 
    // joint_idx_map["rightWristRoll"]       = rightWristRoll    - NUM_VIRTUAL; 
    // joint_idx_map["rightWristPitch"]      = rightWristPitch   - NUM_VIRTUAL; 

    valkyrie_sys_ = new ValkyrieSystem();

    printf("[Interface] Contruct\n");
}

Interface::~Interface(){
}

void Interface::GetCommand(_DEF_SENSOR_DATA_,
                           std::vector<double> & command){
    if(!_Initialization(_VAR_SENSOR_DATA_)){
#if MEASURE_TIME
      std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
        state_estimator_.Update(_VAR_SENSOR_DATA_);
        // Calcualate Torque
        valkyrie_sys_->getTorqueInput(torque_command_);

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
void Interface::GetReactionForce(std::vector<sejong::Vect3> & reaction_force ){

}

bool Interface::_Initialization(_DEF_SENSOR_DATA_){
    if(count_ < 2){
        torque_command_.setZero();
        state_estimator_.Initialization(_VAR_SENSOR_DATA_);
        valkyrie_sys_->controller_->Initialization();
        for (int i(0); i<NUM_ACT_JOINT; ++i){
            initial_jpos_[i] = jpos[i];
        }
        return true;
    }
    DataManager::GetDataManager()->start();
    return false;
}
