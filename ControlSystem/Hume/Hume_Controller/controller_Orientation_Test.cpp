#include "controller_Orientation_Test.h"

#include <math.h>
#include <stdio.h>
#include "utils/pseudo_inverse.hpp"

#include "StateProvider.h"
#include "constraint_library.h"
#include "task_library.h"

// This test is designed for the case when the right thigh is fixed somewhere

Ctrl_Orientation_Test::Ctrl_Orientation_Test():
    Controller_Hume(),
    foot_des_(3),
    foot_vel_des_(3),
    foot_ini_(3),
    count_command_(0),
    omega_(5),
    amp_(5),
    offset_(5){
    
    declareParameter("stance_foot", & base_pt_);
    declareParameter("swing_foot", & swing_foot_);

    declareParameter("constraint_right", & constraint_single_);
    declareParameter("omega", & omega_);
    declareParameter("amp", & amp_);
    // COM
    declareParameter("single_contact_lfoot", & single_contact_task_);
    declareParameter("offset", & offset_);

    printf("[Orientation Test Control] Start\n");
}

Ctrl_Orientation_Test::~Ctrl_Orientation_Test(){
}

void Ctrl_Orientation_Test::getCurrentCommand(std::vector<double> & command){
    _PreProcessing_Command();
    ++count_command_;
 
    if(count_command_ <2000){
        pos_ini_ = state_provider_->CoM_pos_;
        ori_ini_ = state_provider_->getBodyOriQuat();
        foot_ini_ = state_provider_->getFootPos(swing_foot_);
        start_time_ = state_provider_->curr_time_;
    }
    state_machine_time_ = state_provider_->curr_time_ - start_time_;
    phase_ = 11;
    
    _Test_Orientation();
    _Disable_Fixed_Leg(gamma_);
    _PostProcessing_Command(command);
}

void Ctrl_Orientation_Test::_Test_Orientation(){
    b_int_gain_right_ = false;
    b_force_int_gain_left_knee_ = true; 

    _Set_Desired_Foot();
    _Set_Test_Task();
    task_array_.push_back(single_contact_task_);
    wbc_->MakeTorque(state_provider_->Q_,
                     state_provider_->Qdot_,
                     task_array_, constraint_single_, model_, gamma_);
}
void Ctrl_Orientation_Test::_Set_Desired_Foot(){
    for (int i(0); i<3; ++i){
        foot_des_[i] = foot_ini_[i];
        foot_vel_des_[i] = 0.0;
    }
}
void Ctrl_Orientation_Test::_Set_Test_Task(){
    Vector des(7);
    Vector vel_des(5);
    Vector act(7);
    Vector vel_act(5);
    // Orientation (Pitch, Roll)
    double pitch_des = EulerZYX_ini_[1] + offset_[0] + amp_[0]*sin(omega_[0]*state_machine_time_);
    // Roll
    double roll_des = EulerZYX_ini_[2] + offset_[1] + amp_[1]*sin(omega_[1]*state_machine_time_);
    vel_des[1] = amp_[0]*omega_[0]*cos(omega_[0]*state_machine_time_);
    vel_des[2] = amp_[1]*omega_[1]*cos(omega_[1]*state_machine_time_);

    sejong::Quaternion des_q;
    sejong::convert(0.0, pitch_des, roll_des, des_q);

    des[0] = des_q.w();
    des[1] = des_q.x();
    des[2] = des_q.y();
    des[3] = des_q.z();

    act.head(4) = state_provider_->getBodyOri();
    act.tail(3) = state_provider_->getFootPos(swing_foot_);

    vel_act.head(3) = state_provider_->Body_ang_vel_;
    vel_act.tail(3) = state_provider_->getFootVel(swing_foot_);
    
    // Foot
    for(int i(0); i<3; ++i){
        des[i+4] = foot_des_[i]
            + offset_[2+i] + amp_[2 + i] * sin(omega_[2 + i] * state_machine_time_);
        vel_des[i+3] = foot_vel_des_[i]
            + amp_[2+i] * omega_[2+i] * cos(omega_[2+i] * state_machine_time_);
    }
    single_contact_task_->SetTask(des, vel_des,
                                  act, vel_act);
}

void Ctrl_Orientation_Test::_Disable_Fixed_Leg(sejong::Vector & command){
    if(base_pt_ == RFOOT){
        command[2] = 0.0;
    }
    else if(base_pt_ == LFOOT){
        command[5] = 0.0;
    }
}
