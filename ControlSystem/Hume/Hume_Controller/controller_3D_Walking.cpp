#include "controller_3D_Walking.h"
#include <math.h>
#include <stdio.h>
#include "utils/pseudo_inverse.hpp"
#include "Process.h"

#include "Planner/Planner.h"
#include "task_library.h"

#include "utils/utilities.h"
#include "StateProvider.h"

// #define STEP_TEST

Ctrl_3D_Walking::Ctrl_3D_Walking():
    Virtual_Ctrl_Step(),
    com_vel_ave_(sejong::Vector::Zero(3)),
    previous_time_(0.0),
    body_vel_pre_(sejong::Vector::Zero(3)),
    body_vel_pre_2_(sejong::Vector::Zero(3)),
    body_vel_pre_3_(sejong::Vector::Zero(3)),
    vel_est_(0.0),
    a(sejong::Vector::Zero(2)), b(sejong::Vector::Zero(2)), c(sejong::Vector::Zero(2)), d(sejong::Vector::Zero(2)),
    initialization_count_(0){
    declareParameter("com_x_offset", &com_x_offset_);
    declareParameter("com_y_offset", &com_y_offset_);
    declareParameter("com_z_offset", &com_z_offset_);
    declareParameter("stance_foot", & base_pt_);
    declareParameter("swing_foot", & swing_foot_);

    declareParameter("lifting_time", & lifting_time_);
    declareParameter("landing_time", & const_landing_time_);
    declareParameter("supporting_time", & supp_time_);
    declareParameter("landing_supp_transition_time", &land_supp_transition_time_);
    declareParameter("supp_lift_transition_time", & supp_lift_transition_time_);
    declareParameter("lifting_height", & lifting_height_);
    // for planner
    declareParameter("achieving_ratio", & achieving_ratio_);
    declareParameter("middle_time_x", & middle_time_x_);
    declareParameter("middle_time_y", & middle_time_y_);
    
    // Constraint
    declareParameter("constraint_dual", & constraint_dual_);
    declareParameter("constraint_left", & constraint_left_);
    declareParameter("constraint_right", & constraint_right_);

    // CoM
    declareParameter("dual_contact_task", &dual_contact_task_);
    declareParameter("single_contact_lfoot", & single_contact_lfoot_);
    declareParameter("single_contact_rfoot", & single_contact_rfoot_);

    declareParameter("planner", & planner_);
    
    printf("[3D Walking Control] Start\n");
    
    planner_info_ = new Planner_Info();

    constraint_ = NULL;

}
Ctrl_3D_Walking::~Ctrl_3D_Walking(){
}

void Ctrl_3D_Walking::getCurrentCommand(std::vector<double> & command){
    _PreProcessing_Command();
    ++count_command_;

    if(pro_stable_->do_process(this)){
        _PostProcessing_Command(command);

        count_command_ = 0;
        planner_->start();

        return;
    }
    else if(count_command_ == 1){
        pro_stable_->clear_process();
        _end_action();
        _printf_phase(RsDu);
    }
    
    static int curr_phase = RsDu;
    phase_ = curr_phase;
    state_provider_->hold_xy_ = false;
    if(_IsEndCurrState(curr_phase)){

        ++curr_phase;
        
        if(curr_phase == LsRs || curr_phase == RsLs){
            ++num_step_;
            printf("Number of Step: %i \n", num_step_);
        }
        if(curr_phase > 9) { curr_phase = 0; }
        // Landing Location Save
        if((curr_phase == LsTran || curr_phase == RsTran) && num_step_ > 0){
            sejong::saveVector(-state_provider_->getFootPos(swing_foot_),"landing_loc");
            walking_length_x_ -= (state_provider_->getFootPos(swing_foot_)[0]);
            walking_length_y_ -= (state_provider_->getFootPos(swing_foot_)[1]);
            
            printf("*** LANDING LOCATION (x, y): %f, %f \n",
                   -state_provider_->getFootPos(swing_foot_)[0],
                   -state_provider_->getFootPos(swing_foot_)[1]);
            
            printf("*** WALKING LENGTH (x, y): %f, %f \n", walking_length_x_, walking_length_y_);
        }
        _printf_phase(curr_phase);

    }
    _PostProcessing_Command(command);
}

bool Ctrl_3D_Walking::_Stop(){
    return true;
}

bool Ctrl_3D_Walking::CalculateLandingLocation(const sejong::Vector & base_pt_loc, double current_time, double local_switch_time){

    // CoM
    sejong::Vect3 CoM_Position;
    CoM_Position = state_provider_->CoM_pos_;
    if(num_step_ ==0){
        planner_info_->global_x = 0.0;
        planner_info_->global_y = 0.0;
        walking_length_x_ = -CoM_Position[0];
        walking_length_y_ = -CoM_Position[1];
    }
    else{
        planner_info_->global_x = CoM_Position[0] + walking_length_x_  - state_provider_->attraction_loc_[0];
        planner_info_->global_y = CoM_Position[1] + walking_length_y_  - state_provider_->attraction_loc_[1];
    }
    if(planner_->Calculate_SwitchTime(CoM_Position,
                                      state_provider_->getBodyEulerZYX(),
                                      // CoM
                                      state_provider_->CoM_vel_,
                                      base_pt_loc,
                                      base_pt_,
                                      const_landing_time_,
                                      // landing_time_,
                                      lifting_time_,
                                      achieving_ratio_,
                                      land_supp_transition_time_,
                                      supp_lift_transition_time_,
                                      supp_time_,
                                      middle_time_x_,
                                      middle_time_y_,
           planner_info_)){
        //
        landing_time_ = planner_->GetSwitchingTime()
            - (1.0 - achieving_ratio_) * lifting_time_
            - planner_->transition_time_mix_ratio_*land_supp_transition_time_;

        if (landing_time_ < 0.0){
            landing_time_ = 0.7*const_landing_time_;
        }
        
        planner_->CopyNextFootLoc(landing_loc_);
        if (base_pt_ == RFOOT && landing_loc_[1] <0 ){
            printf("[R stance] landing location is smaller than 0 (%f)", landing_loc_[1]);
            landing_loc_[1] = 0.08;
        }
        else if( base_pt_ == LFOOT && landing_loc_[1] > 0 ){
            printf("[L stance] landing location is larger than 0 (%f)", landing_loc_[1]);
            landing_loc_[1] = -0.08;
        }

        sejong::pretty_print(landing_loc_, std::cout, "landing loc", "");
#ifdef STEP_TEST
        landing_time_ = const_landing_time_
            - 0.5*land_supp_transition_time_;

        landing_loc_[0] = 0.0; //state_provider_->CoM_pos_[0];
        if(base_pt_ == LFOOT){
            landing_loc_[1] = -0.14; //state_provider_->CoM_pos_[1] - 0.142;
        }
        else if (base_pt_ == RFOOT){
            landing_loc_[1] = 0.14; //state_provider_->CoM_pos_[1] + 0.142;
        }
        landing_loc_[2] = 0.0;
        printf("[stepping test] landing location (%f, %f, %f) \n", landing_loc_[0],
               landing_loc_[1],
               landing_loc_[2]);
#endif
        // printf("const landing time: %f \n", const_landing_time_);
        // printf("switching time: %f \n", planner_->GetSwitchingTime());
        // printf("remaining_lifting_time: %f \n", (1.0-achieving_ratio_) * lifting_time_);
        // printf("included transition time: %f \n", planner_->transition_time_mix_ratio_ * land_supp_transition_time_);
        printf("landing time: %f \n", landing_time_);
        return true;
    }
    return false;
}

void Ctrl_3D_Walking::_set_land_location(){
    // Defined in CalculateLandingLocation
}
void Ctrl_3D_Walking::_set_apex_foot_location(){
    apex_foot_loc_ = foot_pre_;
    apex_foot_loc_[2] += lifting_height_;
}
bool Ctrl_3D_Walking::_Do_Supp_Lift(){
    double moving_ratio = 0.0;
    double z_re_force = 0.0;

    static bool Iscalculated(false);
    //COMMAND
    foot_des_[0] = foot_pre_[0];
    foot_vel_des_[0] = 0.0;
    foot_des_[1] = foot_pre_[1];
    foot_vel_des_[1] = 0.0;
    
    //Foot Height
    foot_des_[2] = smooth_changing(foot_pre_[2], apex_foot_loc_[2], lifting_time_, state_machine_time_);
    foot_vel_des_[2] = smooth_changing_vel(foot_pre_[2], apex_foot_loc_[2], lifting_time_, state_machine_time_);
    _set_single_contact_task();

    if( state_machine_time_ > achieving_ratio_ *lifting_time_ && !Iscalculated){
        Iscalculated = CalculateLandingLocation(state_provider_->getFootPos(base_pt_), state_machine_time_, switching_time_  + tot_transition_time_);
    }

    switch(swing_foot_){
    case RFOOT:
        task_array_.push_back(single_contact_rfoot_);
        single_contact_rfoot_->force_[0] = z_re_force;
        constraint_ = constraint_left_;
        break;
    case LFOOT:
        task_array_.push_back(single_contact_lfoot_);
        single_contact_lfoot_->force_[0] = z_re_force;
        constraint_ = constraint_right_;
        break;
    default:
        printf("[Ctrl 3D Walking] Wrong link ID\n");
        break;
    }
    wbc_->MakeTorque(state_provider_->Q_,
                     state_provider_->Qdot_,
                     task_array_, constraint_, model_, gamma_);
    /////////////////////////////////////////////////////
    if(state_machine_time_ > lifting_time_){
        if(Iscalculated){
            _end_action();
            Iscalculated = false;
            _set_foot_x_traj_coeff(foot_vel_des_);
            return false;
        }
        else{
            printf("Cannot go to the next phase on time\n");
            _end_action();
            Iscalculated = false;
            exit(0);
            return false;
        }
    }
    state_machine_time_ = state_provider_->curr_time_ - start_time_;
    return true;
}


void Ctrl_3D_Walking::_set_foot_x_traj_coeff(const sejong::Vector& curr_vel){
    double tf(landing_time_);
    double xf_dot(0.0);
    double xf, x0, x0_dot;

    for (int i(0);i <2; ++i){
        xf = landing_loc_[i];
        // if( (num_step_ > 4 && num_step_ < 7) && i ==0){
        //     xf -= 0.04;
        // }

        x0 = foot_pre_[i];
        x0_dot = curr_vel[i];
        
        d[i] = x0;
        c[i] = x0_dot;

        sejong::Matrix tmp_inv(2,2);
        sejong::Matrix tmp_coeff(2,2);
        tmp_coeff(0,0) = pow(tf, 3.0);
        tmp_coeff(0,1) = pow(tf,2.0);
        //
        tmp_coeff(1,0) = 3*pow(tf, 2.0);
        tmp_coeff(1,1) = 2*tf;
        
        // pseudoInverse(tmp_coeff, 0.001, tmp_inv, 0);
        tmp_inv = tmp_coeff.inverse();
        
        sejong::Vector tmp_vec(2);
        sejong::Vector coeff(2);
        tmp_vec[0] = xf - (c[i]*tf + d[i]);
        tmp_vec[1] = xf_dot - c[i];
        
        coeff = tmp_inv * tmp_vec;
        
        a[i] = coeff[0];
        b[i] = coeff[1];
    }
}

bool Ctrl_3D_Walking::_Do_Supp_Land(){
    // Foot z
    // foot_des_[2] = smooth_changing(foot_pre_[2], landing_loc_[2],
    //                                 landing_time_,
    //                                 state_machine_time_);
    // foot_vel_des_[2] = smooth_changing_vel(foot_pre_[2], landing_loc_[2],
    //                                         landing_time_,
    //                                         state_machine_time_);
    double z_re_force = 0.0;
    
// Foot x, y
    for (int i(0); i<2; ++i){
        foot_des_[i] = a[i] * pow(state_machine_time_, 3.0) +
            b[i] * pow(state_machine_time_, 2.0) +
            c[i] * state_machine_time_ +
            d[i] ;
        foot_vel_des_[i] = 3* a[i] * pow(state_machine_time_, 2.0) +
            2* b[i] * state_machine_time_ +
            c[i];
    }

    int indicator(0);
    foot_des_[2] = (foot_pre_[2]-landing_loc_[2])  * cos( 0.5 * M_PI *
                                        (foot_des_[indicator] - foot_pre_[indicator])/
                                        (landing_loc_[indicator] - foot_pre_[indicator]) ) + landing_loc_[2];
    foot_vel_des_[2] = - (foot_pre_[2] - landing_loc_[2]) * (0.5 * M_PI /
                                        (landing_loc_[indicator] - foot_pre_[indicator])) *
        foot_vel_des_[indicator] * sin(0.5 * M_PI *
                                       (foot_des_[indicator] - foot_pre_[indicator])/
                                       (landing_loc_[indicator] - foot_pre_[indicator]) );
    
    if(state_machine_time_>landing_time_){
        printf("Contact is not occured Yet, But Move on...\n");
        _end_action();
        return false;
    }
    _set_single_contact_task();
    
    switch(swing_foot_){
    case RFOOT:
        task_array_.push_back(single_contact_rfoot_);
        single_contact_rfoot_->force_[0] = z_re_force;
#ifndef STEP_TEST
        if(state_provider_->RFoot_Contact()){
            printf("Right foot contact occurs\n");
            _end_action();
            return false;
        }
#endif
        constraint_ = constraint_left_;
        break;

    case LFOOT:
        task_array_.push_back(single_contact_lfoot_);
        single_contact_lfoot_->force_[0] = z_re_force;
#ifndef STEP_TEST
        if(state_provider_->LFoot_Contact()){
            printf("Left foot contact occurs\n");
            _end_action();
            return false;
        }
#endif
        constraint_ = constraint_right_;
        break;
    default:
        printf("[Ctrl 3D Walking, Supp & Land ] Wrong link ID\n");
        break;

    }
    wbc_->MakeTorque(state_provider_->Q_,
                     state_provider_->Qdot_,
                     task_array_, constraint_, model_, gamma_);
    
    state_machine_time_ = state_provider_->curr_time_ - start_time_;
    return true;
}
