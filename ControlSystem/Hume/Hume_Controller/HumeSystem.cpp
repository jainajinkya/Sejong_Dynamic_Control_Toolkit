#include "HumeSystem.h" 
#include <iostream> 
#include <stdio.h> 

#include "Controller_Hume.h" 
#include "Scenario/Factory.h"

#include "Hume_Model/Hume_Model.h"

#include "StateProvider.h"
#include "Configuration.h"
#include "utils/utilities.h"
#include "FootForce_Calculator.h"

#ifdef __RTAI__
 #include <rtai_sem.h>
 #include <rtai_sched.h>
#endif

#include <sys/syscall.h>

using namespace sejong; 

HUME_System::HUME_System():object(),
                           torque_limit_(NUM_ACT_JOINT),
                           pos_gain_kp_(NUM_ACT_JOINT),
                           pos_gain_ki_(NUM_ACT_JOINT),
                           motor_gain_kv_(NUM_ACT_JOINT) {
    
    declareParameter("Controller", &controller_);
    declareParameter("motor_pos_gain_kp",  &pos_gain_kp_);
    declareParameter("motor_pos_gain_ki",  &pos_gain_ki_);
    declareParameter("motor_gain_kv", &motor_gain_kv_);
    declareParameter("torque_limit", &torque_limit_);
    // Setting Controller
    hume_factory_ = new Factory(this);
    delete hume_factory_;

    foot_force_calculator_ = new Foot_Force_Calculator();
    foot_force_calculator_->start();
 
    printf("[Hume System] Constructed, TID: %d \n", (int)syscall(SYS_gettid));
} 

HUME_System::~HUME_System(){ 
    delete controller_;
    delete foot_force_calculator_;
    printf("[HUME_System] Destruction \n"); 
} 

void HUME_System::getTorqueInput( std::vector<signed int> & pos_gain_kp, 
                                  std::vector<signed int> & pos_gain_ki,
                                  std::vector<signed int> & motor_gain_kv,
                                  sejong::Vector & torque_command){
    controller_->getCurrentCommand(command_);
    setCommand(command_, torque_command);
    setGain(pos_gain_kp, pos_gain_ki, motor_gain_kv);
}

void HUME_System::Safty_Stop(const std::vector<double> & jvel, std::vector<double> & command){ 
    for (int i(0); i < NUM_ACT_JOINT; ++i){ 
        if(jvel[i]> SPEED_LIMIT || jvel[i]< -SPEED_LIMIT){      
            printf("%ith joint hit the velocity limit: %f \n", i, jvel[i]); 
            command[i] = 0.0; 
        } 
    } 
} 
 
void HUME_System::setGain(std::vector<signed int> & kp,
                          std::vector<signed int> & ki,
                          std::vector<signed int> & kv){
    for (int i(0); i<NUM_ACT_JOINT; ++i){
        kp[i] = pos_gain_kp_[i]; 
        ki[i] = pos_gain_ki_[i]; 
        kv[i] = motor_gain_kv_[i];
    }
} 
 
void HUME_System::setCommand(std::vector<double> & command, sejong::Vector & torque_command){ 
    for (int i(0); i < NUM_ACT_JOINT; ++i){
        if(command[i] > torque_limit_[i]){ 
            command[i] = torque_limit_[i];
        }
        else if(command[i] < -torque_limit_[i]){ 
            command[i] = -torque_limit_[i];
        }
        StateProvider::GetStateProvider()->torque_input_[i] = command[i];
        torque_command[i] = command[i];
    }

    // for(int i(0); i< NUM_ACT_JOINT; ++i){ 
    //     torque_gain_kp_[i] = torque_gain_kp_stance_[i];
    //     torque_gain_ki_[i] = torque_gain_ki_stance_[i];
    // } 

    // // Abduction Right
    // if(fabs(command[0]) < 10.5 || !StateProvider::GetStateProvider()->right_foot_contact_){
    //     torque_gain_kp_[0] = torque_gain_kp_swing_[0];
    //     torque_gain_ki_[0] = torque_gain_ki_swing_[0];
    // }
    // // Abduction Left
    // if(fabs(command[3]) < 10.5 || !StateProvider::GetStateProvider()->left_foot_contact_){
    //     torque_gain_kp_[3] = torque_gain_kp_swing_[3];
    //     torque_gain_ki_[3] = torque_gain_ki_swing_[3];
    // }    
    // // Hip Right 
    // if(controller_->Force_Integral_Gain_Right_Knee() || 
    //    (controller_->Is_Integral_Gain_Right() 
    //     &&(fabs(command[1]) < 10.5 || !StateProvider::GetStateProvider()->right_foot_contact_))){ 
    //     torque_gain_kp_[1] = torque_gain_kp_swing_[1];
    //     torque_gain_ki_[1] = torque_gain_ki_swing_[1]; 
    // } 
    // // Hip Left 
    // if(controller_->Force_Integral_Gain_Left_Knee() || 
    //    (controller_->Is_Integral_Gain_Left() 
    //     && (fabs(command[4]) < 10.5 || !StateProvider::GetStateProvider()->left_foot_contact_))){ 
    //     torque_gain_kp_[4] = torque_gain_kp_swing_[4]; 
    //     torque_gain_ki_[4] = torque_gain_ki_swing_[4]; 
    // } 
 
    // // Knee Right 
    // if(controller_->Force_Integral_Gain_Right_Knee() || 
    //    (controller_->Is_Integral_Gain_Right() && 
    //     (fabs(command[2]) < 15.5 || !StateProvider::GetStateProvider()->right_foot_contact_))){ 
    //     torque_gain_kp_[2] = torque_gain_kp_swing_[2]; 
    //     torque_gain_ki_[2] = torque_gain_ki_swing_[2]; 
    // } 
    // // Knee Left 
    // if(controller_->Force_Integral_Gain_Left_Knee() || 
    //    (controller_->Is_Integral_Gain_Left() &&  
    //     (fabs(command[5]) < 15.5 || !StateProvider::GetStateProvider()->left_foot_contact_))){ 
    //     torque_gain_kp_[5] = torque_gain_kp_swing_[5]; 
    //     torque_gain_ki_[5] = torque_gain_ki_swing_[5]; 
    // } 
     
    // if(!StateProvider::GetStateProvider()->right_foot_contact_){ 
    //     for (int i(0); i< 3; ++i){ 
    //         if(command[i] > TORQUE_LIMIT_AIR){ 
    //             command[i] = TORQUE_LIMIT_AIR;  
    //         } 
    //         if(command[i] < -TORQUE_LIMIT_AIR){  
    //             command[i] = -TORQUE_LIMIT_AIR; 
    //         } 
    //     } 
    // } 
    // if(!StateProvider::GetStateProvider()->left_foot_contact_){ 
    //     for (int i(3); i< 6; ++i){ 
    //         if(command[i] > TORQUE_LIMIT_AIR){ 
    //             command[i] = TORQUE_LIMIT_AIR; 
    //         } 
    //         if(command[i] < -TORQUE_LIMIT_AIR){ 
    //             command[i] = -TORQUE_LIMIT_AIR; 
    //         } 
    //     } 
    // }
}
