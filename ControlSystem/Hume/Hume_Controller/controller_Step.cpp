#include "controller_Step.h"

#include <math.h>
#include <stdio.h>
#include "constraint_library.h"
#include "StateProvider.h"

#include "FootForce_Calculator.h"

#include "task_library.h"
#include "Process.h"
#include "Planner/terrain_interpreter.h"
#include "utils/utilities.h"


Ctrl_Step::Ctrl_Step():
    Controller_Hume(),
    foot_des_(3), foot_vel_des_(Vector::Zero(3)),
    num_step_(0), state_machine_time_(0.0),
    swing_foot_(LFOOT),
    landing_loc_(Vector::Zero(3)),
    start_time_(0.0),
    walking_length_x_(0.0),
    walking_length_y_(0.0)
{
    base_pt_ = HIP;

    dual_contact_task_    = NULL; 
    single_contact_lfoot_ = NULL;
    single_contact_rfoot_ = NULL;
    constraint_dual_ = NULL;
    constraint_left_  = NULL;
    constraint_right_ = NULL;

    //Process
    pro_stable_ = new stabilizing(state_provider_, model_);
    printf("[Ctrl_Step] Contructed\n");
}

Ctrl_Step::~Ctrl_Step(){
    SAFE_DELETE(dual_contact_task_   ); 
    SAFE_DELETE(single_contact_lfoot_);
    SAFE_DELETE(single_contact_rfoot_);
    
    SAFE_DELETE(constraint_left_ );
    SAFE_DELETE(constraint_right_);

}

void Ctrl_Step::getCurrentCommand(std::vector<double> & command){
    _PreProcessing_Command();
    ++count_command_;
    
    if(pro_stable_->do_process(this)){
        _PostProcessing_Command(command);
        count_command_ = 0;
        return;
    }
    
    static int curr_phase = LsDu;
    phase_ = curr_phase;

    if(_IsEndCurrState(curr_phase)){
        phase_ = curr_phase;
        ++curr_phase;
        if(curr_phase == LsRm || curr_phase == RsLm){
            ++num_step_;
            printf("Number of Step: %i \n", num_step_);
        }
        if(curr_phase > 9) { curr_phase = 0; }
        _printf_phase(curr_phase);
    }

    _PostProcessing_Command(command);
}

void Ctrl_Step::_end_action(){
    state_machine_time_ = 0.0;
    start_time_ = state_provider_->curr_time_;
    foot_pre_ = foot_des_;
}

bool Ctrl_Step::_IsEndCurrState(const int curr_phase){
    switch(curr_phase){
    case LsDu:
        base_pt_ = LFOOT;
        swing_foot_ = RFOOT;
        return !_Supp();
    case LsTran:
        base_pt_ = LFOOT;
        swing_foot_ = RFOOT;
        return !_Supp_Lift_transition();
    case LsRl:
        base_pt_ = LFOOT;
        swing_foot_ = RFOOT;
        b_int_gain_left_ = false;
        b_force_int_gain_right_knee_ = true;
        return !_Do_Supp_Lift();
    case LsRm:
        base_pt_ = LFOOT;
        swing_foot_ = RFOOT;
        b_int_gain_left_ = false;
        b_force_int_gain_right_knee_ = true; 
        return !_Do_Supp_Land();
    case LsRs:
        base_pt_ = LFOOT;
        swing_foot_ = RFOOT;
        b_int_gain_left_ = false;
        return !_Land_Supp_transition();

    case RsDu:
        base_pt_ = RFOOT;
        swing_foot_ = LFOOT;
        return !_Supp();
    case RsTran:
        base_pt_ = RFOOT;
        swing_foot_ = LFOOT;
        b_int_gain_right_ = false;
        b_int_gain_left_ = true;
        b_force_int_gain_left_knee_ = true;
        return !_Supp_Lift_transition();
    case RsLl:
        base_pt_ = RFOOT;
        swing_foot_ = LFOOT;
        b_int_gain_right_ = false;
        b_force_int_gain_left_knee_ = true; 
        return !_Do_Supp_Lift();
    case RsLm:
        base_pt_ = RFOOT;
        swing_foot_ = LFOOT;
        b_int_gain_right_ = false;
        b_force_int_gain_left_knee_ = true;
        return !_Do_Supp_Land();
    case RsLs:
        base_pt_ = RFOOT;
        swing_foot_ = LFOOT;
        b_int_gain_right_ = false;
        return !_Land_Supp_transition();
    case STOP:
        // stance foot and swing foot are not changed
        b_int_gain_left_ = false;
        b_int_gain_right_ = false;
        return !_Stop();
    }
    return true;
} 


void Ctrl_Step::_printf_phase(int phase){
    switch(phase){
    case LsDu:        printf("**Phase: LSDU\n");        break;
    case LsTran:      printf("**Phase: LSTRAN\n");      break;
    case LsRl:        printf("**Phase: LSRL\n");        break;
    case LsRm:        printf("**Phase: LSRM\n");        break;
    case LsRs:        printf("**Phase: LSRLAND\n");        break;
    case RsDu:        printf("**Phase: RSDU\n");        break;
    case RsTran:      printf("**Phase: RSTRAN\n");      break;
    case RsLl:        printf("**Phase: RSLL\n");        break;
    case RsLm:        printf("**Phase: RSLM\n");        break;
    case RsLs:        printf("**Phase: RSLLAND\n");        break;
    case STOP:        printf("**Phase: Stop\n");        break;
    default: printf(" *** ERROR: PHASE ***\n"); break;
    }
}

//////////////////////////////////////////////////////////////
//////////////////// Ctrl 3D Step ////////////////////////////
//////////////////////////////////////////////////////////////
Virtual_Ctrl_Step::Virtual_Ctrl_Step(): Ctrl_Step(){
    printf("[Virtual Ctrl_Step] Contructed\n");
}

Virtual_Ctrl_Step::~Virtual_Ctrl_Step(){
}

void Virtual_Ctrl_Step::_get_smooth_changing_foot_force(const Vector & start_force, const Vector & end_force, double duration, double curr_time, Vector & ret_force){
    ret_force = Vector::Zero(6);
    // X
    ret_force[3] = sejong::smooth_changing(start_force[0], end_force[0], duration, curr_time);
    // Y           sejong::
    ret_force[4] = sejong::smooth_changing(start_force[1], end_force[1], duration, curr_time);
    // Z           sejong::
    ret_force[5] = sejong::smooth_changing(start_force[2], end_force[2], duration, curr_time);
}

bool Virtual_Ctrl_Step::_Supp_Lift_transition(){
    static bool b_transit(false);
    
    Vector gamma_single_contact, gamma_dual_contact;
    static Vect3 re_force_dual;

    if(!b_transit){
        std::vector<Task*> dual_contact_task_array;
        _set_dual_contact_task();
        dual_contact_task_array.push_back(dual_contact_task_);
        wbc_2_->MakeTorque(state_provider_->Q_,
                           state_provider_->Qdot_,
                           dual_contact_task_array, constraint_dual_,
                           model_,
                           gamma_dual_contact);
        Foot_Force_Calculator::Cal_FootForce(swing_foot_, gamma_dual_contact, re_force_dual);
        // sejong::pretty_print(re_force_dual, std::cout, "re force dual", "");
        b_transit = true;
    } 
    constraint_ = constraint_dual_;
    // Push down desired foot position
    foot_des_ = foot_pre_;
    foot_des_[2] = 0.0;
    foot_vel_des_ = Vector::Zero(3);


    switch(swing_foot_){
    case RFOOT:
        _set_single_contact_task();
        _get_smooth_changing_foot_force(re_force_dual, Vector::Zero(3), supp_lift_transition_time_, state_machine_time_, single_contact_rfoot_->force_);
        task_array_.push_back(single_contact_rfoot_);

        wbc_->MakeTorque(state_provider_->Q_,
                         state_provider_->Qdot_,
                         task_array_, constraint_left_, model_,
                         gamma_single_contact);
        break;
        
    case LFOOT:
        _set_single_contact_task();
        _get_smooth_changing_foot_force(re_force_dual, Vector::Zero(3), supp_lift_transition_time_, state_machine_time_, single_contact_lfoot_->force_);
        task_array_.push_back(single_contact_lfoot_);
        
        wbc_->MakeTorque(state_provider_->Q_,
                         state_provider_->Qdot_,
                         task_array_, constraint_right_, model_,
                         gamma_single_contact);
        break;
    default:
        printf("[Virtual Step] Incorrect Swing foot ID\n");
        break;
    }
    gamma_ = gamma_single_contact;

    if(state_machine_time_ > supp_lift_transition_time_){
        _end_action();
        _set_apex_foot_location();
        b_transit = false;
        return false;
    }
    state_machine_time_ = state_provider_->curr_time_ - start_time_;
    return true;
}


bool Virtual_Ctrl_Step::_Land_Supp_transition(){
    static bool b_transit(false);
    
    Vector gamma_single_contact, gamma_dual_contact;
    Vector re_force_single;
    static Vect3 re_force_dual;
    double pushing_ratio(0.05);
    std::vector<Task*> dual_contact_task_array;
    _set_dual_contact_task();

    if(!b_transit){
        _set_dual_contact_task();
        dual_contact_task_array.push_back(dual_contact_task_);
        wbc_2_->MakeTorque(state_provider_->Q_,
                           state_provider_->Qdot_,
                           dual_contact_task_array,
                           constraint_dual_, model_,
                           gamma_dual_contact);
        Foot_Force_Calculator::Cal_FootForce(swing_foot_, gamma_dual_contact, re_force_dual);
        re_force_dual[0] = 0.9*re_force_dual[0];
        re_force_dual[1] = 0.9*re_force_dual[1];
        re_force_dual[2] = 0.95*re_force_dual[2];
        // re_force_dual[2] = -40.0;
        b_transit = true;
    }
    constraint_ = constraint_dual_;
    // push down desired foot position
    foot_des_ = foot_pre_;
    foot_des_[2] = foot_pre_[2] - pushing_ratio * state_machine_time_;
    foot_vel_des_ = Vector::Zero(3);

    switch(swing_foot_){
    case RFOOT:
        _set_single_contact_task();
        _get_smooth_changing_foot_force(Vector::Zero(3), re_force_dual, land_supp_transition_time_, state_machine_time_, single_contact_rfoot_->force_);
        
        task_array_.push_back(single_contact_rfoot_);
        
        wbc_->MakeTorque(state_provider_->Q_,
                         state_provider_->Qdot_,
                         task_array_, constraint_left_, model_,
                         gamma_single_contact);
        re_force_single = single_contact_rfoot_ ->force_;
        break;
        
    case LFOOT:
        _set_single_contact_task();
        _get_smooth_changing_foot_force(Vector::Zero(3), re_force_dual, land_supp_transition_time_, state_machine_time_, single_contact_lfoot_->force_);
        
        task_array_.push_back(single_contact_lfoot_);
        
        wbc_->MakeTorque(state_provider_->Q_,
                         state_provider_->Qdot_,
                         task_array_, constraint_right_, model_,
                         gamma_single_contact);
        re_force_single = single_contact_lfoot_->force_;
        break;
    }

    bool both_contact(false);
    if(state_provider_->RFoot_Contact() && state_provider_->LFoot_Contact()){ both_contact = true; } 
    if( both_contact && (state_machine_time_ >= land_supp_transition_time_) ){
        printf("change to dual contact\n");
        _end_action();
        b_transit = false;
        return false;
    }
    gamma_ = gamma_single_contact;

    state_machine_time_ = state_provider_->curr_time_ - start_time_;
    return true;
}

void Virtual_Ctrl_Step::_set_dual_contact_task(){
    Vector des(5);
    Vector act(5);
    Vector vel_des(4);
    Vector vel_act(4);

    // Height
    double x = walking_length_x_ + state_provider_->CoM_pos_[0];
    double y = walking_length_y_ + state_provider_->CoM_pos_[1];
    double com_height = Terrain_Interpreter::GetTerrainInterpreter()->CoM_height(x, y);
    double st_foot_height = Terrain_Interpreter::GetTerrainInterpreter()->surface_height(walking_length_x_, walking_length_y_);
    des[0] = com_height - st_foot_height;
    
    double dz_dx = Terrain_Interpreter::GetTerrainInterpreter()->DzDx(x, y);
    double dz_dy = Terrain_Interpreter::GetTerrainInterpreter()->DzDy(x, y);
    double xdot = state_provider_->CoM_vel_[0];
    double ydot = state_provider_->CoM_vel_[1];
    vel_des[0] = dz_dx * xdot + dz_dy * ydot;

    act[0] = state_provider_->CoM_pos_[2];
    vel_act[0] = state_provider_->CoM_vel_[2];
 
   // Yaw
    vel_des[1] = 0.0;
    vel_act[1] = state_provider_->Qdot_[5];
    // Pitch
    vel_des[2] = 0.0;
    vel_act[2] = state_provider_->Qdot_[4];

    // Roll
    vel_des[3] = 0.0;
    vel_act[3] = state_provider_->Qdot_[3];

    des[1] = 1.0;
    des[2] = 0.0;
    des[3] = 0.0;
    des[4] = 0.0;

    act.segment(1,4) = state_provider_->getBodyOri();

    dual_contact_task_->SetTask(des, vel_des, act, vel_act);
    
    state_provider_->SaveTaskData(act, des, vel_act, vel_des);
}

void Virtual_Ctrl_Step::_set_single_contact_task(){
    Vector des(8);
    Vector act(8);
    Vector vel_des(6);
    Vector vel_act(6);
    
    // Height
    double x = walking_length_x_ + state_provider_->CoM_pos_[0];
    double y = walking_length_y_ + state_provider_->CoM_pos_[1];
    double com_height = Terrain_Interpreter::GetTerrainInterpreter()->CoM_height(x, y);
    double st_foot_height = Terrain_Interpreter::GetTerrainInterpreter()->surface_height(walking_length_x_, walking_length_y_);
    des[0] = com_height - st_foot_height;
    double dz_dx = Terrain_Interpreter::GetTerrainInterpreter()->DzDx(x, y);
    double dz_dy = Terrain_Interpreter::GetTerrainInterpreter()->DzDy(x, y);
    double xdot = state_provider_->CoM_vel_[0];
    double ydot = state_provider_->CoM_vel_[1];
    vel_des[0] = dz_dx * xdot + dz_dy * ydot;

    act[0] = state_provider_->CoM_pos_[2];
    vel_act[0] = state_provider_->CoM_vel_[2];
    // Pitch
    vel_des[1] = 0.0;
    vel_act[1] = state_provider_->Qdot_[4];

    // Roll
    vel_des[2] = 0.0;
    vel_act[2] = state_provider_->Qdot_[3];

    des[1] = 1.0;
    des[2] = 0.0;
    des[3] = 0.0;
    des[4] = 0.0;
    act.segment(1,4) = state_provider_->getBodyOri();

    for (int i(0); i<3; ++i){
        des[i+5] = foot_des_[i];
        vel_des[i+3] = foot_vel_des_[i];
    }
    act.segment(5, 3) = state_provider_->getFootPos(swing_foot_);
    vel_act.segment(3, 3) = state_provider_->getFootVel(swing_foot_);
    
    switch(swing_foot_){
    case LFOOT:
        single_contact_lfoot_->SetTask(des, vel_des, act, vel_act);
        break;
    case RFOOT:
        single_contact_rfoot_->SetTask(des, vel_des, act, vel_act);
        break;
    }
    state_provider_->SaveTaskData(act, des, vel_act, vel_des);
}
bool Virtual_Ctrl_Step::_Supp(){
    if(state_machine_time_ > supp_time_){
        _end_action();
        return false;
    }
    _set_dual_contact_task();

    task_array_.push_back(dual_contact_task_);

    constraint_ = constraint_dual_;
    wbc_->MakeTorque(state_provider_->Q_,
                     state_provider_->Qdot_,
                     task_array_, constraint_, model_, gamma_);

    foot_des_ = state_provider_->getFootPos(swing_foot_);
    state_machine_time_ = state_provider_->curr_time_ - start_time_;

    return true;
}
