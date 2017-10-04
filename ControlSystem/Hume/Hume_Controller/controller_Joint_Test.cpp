#include "controller_Joint_Test.h"

#include <math.h>
#include <stdio.h>
#include "utils/pseudo_inverse.hpp"
#include <utils/utilities.h>


#include "StateProvider.h"
#include "constraint_library.h"
#include "task_library.h"
#include <utils/DataManager.h>
// This test is designed for the case when the right thigh is fixed somewhere

Ctrl_Joint_Test::Ctrl_Joint_Test():
    Controller_Hume(),
    count_command_(0),
    initialization_count_(0),
    phase_shift_(NUM_ACT_JOINT),
    omega_(NUM_ACT_JOINT),
    jpos_ini_(NUM_ACT_JOINT),
    offset_(NUM_ACT_JOINT),
    amp_(NUM_ACT_JOINT),
    des_acc_(NUM_ACT_JOINT),
    des_(NUM_ACT_JOINT),
    vel_des_(NUM_ACT_JOINT) {
    
    declareParameter("joint_task", & jpos_task_);
    declareParameter("fixed_constraint", & fixed_constraint_);
    declareParameter("omega", & omega_);
    declareParameter("amp", & amp_);
    declareParameter("offset", & offset_);
    declareParameter("observing_foot", & observing_foot_);
    declareParameter("phase_shift", &phase_shift_);
    printf("[Joint Control Test] Start\n");

    DataManager::GetDataManager()->RegisterData(&des_, SJ_VEC, "des_jpos", NUM_ACT_JOINT);
    DataManager::GetDataManager()->RegisterData(&vel_des_, SJ_VEC, "des_jvel", NUM_ACT_JOINT);

    // wbc_->b_ignore_gravity_ = true;
}

Ctrl_Joint_Test::~Ctrl_Joint_Test(){
}

void Ctrl_Joint_Test::getCurrentCommand(std::vector<double> & command){
    _PreProcessing_Command();
    ++count_command_;

    if(count_command_ < 3){
        for (int i(0); i < model_->NAJ() ; ++i){
            jpos_ini_[i] = state_provider_->Q_[i + model_->NPQ()];
        }
        start_time_ = state_provider_->curr_time_;
        constraint_ = fixed_constraint_;
    }

    state_machine_time_ = state_provider_->curr_time_ - start_time_;
    phase_ = 10;

    _Test_Joint();
    _PostProcessing_Command(command);
}

void Ctrl_Joint_Test::_Test_Joint(){
    _SetDesJPos();
    task_array_.push_back(jpos_task_);
    
    wbc_->MakeTorque(state_provider_->Q_,
                     state_provider_->Qdot_,
                     task_array_, fixed_constraint_, model_, gamma_);
}

void Ctrl_Joint_Test::_SetDesJPos(){
    Vector act;
    Vector vel_act;

    act = (state_provider_->Q_).segment(NUM_VIRTUAL, NUM_ACT_JOINT);
    vel_act = (state_provider_->Qdot_).segment(NUM_VIRTUAL, NUM_ACT_JOINT);


    for (int i(0); i< model_->NAJ() ; ++i){
        des_[i] = offset_[i] + amp_[i]*sin(omega_[i]*state_machine_time_ + phase_shift_[i]);
        vel_des_[i] = amp_[i]*omega_[i]*cos(omega_[i]*state_machine_time_ + phase_shift_[i]);
        des_acc_[i] = -amp_[i]*omega_[i]*omega_[i] * sin(omega_[i]*state_machine_time_ + phase_shift_[i]);
    }
    
    jpos_task_->SetTask(des_, vel_des_, act, vel_act, des_acc_);
    state_provider_->task_des_.head(6) = des_;
    state_provider_->task_curr_.head(6) = act;
    state_provider_->task_vel_des_.head(6) = vel_des_;
    state_provider_->task_vel_curr_.head(6) = vel_act;
}

