#include "Process.h"
#include "constraint_library.h"
#include "task_library.h"
#include <iostream>
#include "Planner/terrain_interpreter.h"
#include "Controller_Hume.h"

#include "StateProvider.h"
#include "StateEstimator.h"
#include <WBOSC/WBOSC_Model.h>
#include "utils/utilities.h"
#include "Configuration.h"
#include "FootForce_Calculator.h"

using namespace sejong;

process::process(StateProvider * state_provider, WBOSC_Model* model){
    state_provider_ = state_provider;
    model_ = model;
}

stabilizing::stabilizing(StateProvider* state_provider, WBOSC_Model* model):
    process(state_provider, model),
    start_height_(0.0),
    stable_lifting_time_(5.5),
    stable_move_time_(3.5),
    lifting_height_(0.17),
    b_stabilized_(false),
    b_move_stabilized_(false),
    b_foot_move_stabilized_(false),
    b_transit_(false),
    count_(0),
    pos_des_(2),
    vel_des_(Vector::Zero(2)),
    ori_vel_des_(Vector::Zero(1)),
    height_pitch_des_(Vector::Zero(2)),
    height_pitch_vel_des_(Vector::Zero(2)),
    initial_pitch_(0.0),
    stable_rfoot_pos_(Vector::Zero(3)),
    stable_lfoot_pos_(Vector::Zero(3)){

#ifndef __RTAI__
    stable_lifting_time_ = 1.5;
    stable_move_time_ = 0.3;
#endif
    
    
    jpos_task_ = new Joint_Task();
    right_foot_task_ = new FOOT_Task(RFOOT);
    left_foot_task_ = new FOOT_Task( LFOOT);

    fixed_constraint_ = new Hume_Fixed_Constraint();
    dual_contact_ = new Hume_Contact_Both();

    COM_height_tilting_roll_task_ = new COM_Height_Tilting_Roll_Task();
    

    stable_rfoot_pos_[0] = -0.1;
    stable_lfoot_pos_[0] = -0.1;

    set_pitch_ = 0.0;

    jpos_init_ = Vector::Zero(model_->NAJ());
    jpos_des_ = Vector::Zero(model_->NAJ());
    jvel_des_ = Vector::Zero(model_->NAJ());
    jacc_des_ = Vector::Zero(model_->NAJ());

    jpos_stable_ = Vector::Zero(model_->NAJ());

    jpos_stable_[0] = 0.03;
    jpos_stable_[1] = -0.71;
    jpos_stable_[2] = 1.73;
    jpos_stable_[3] = -0.02;
    jpos_stable_[4] = -0.71;
    jpos_stable_[5] = 1.73;

}

void stabilizing::clear_process(){
    delete jpos_task_;
    delete right_foot_task_;
    delete left_foot_task_;
    
    delete fixed_constraint_;
    delete dual_contact_;

    delete COM_height_tilting_roll_task_;
}

bool stabilizing::do_process(Controller_Hume* controller){
    static double start_time(0.0);

    if(count_ == 0){
        start_time = state_provider_->curr_time_;
    }
    double curr_time = state_provider_->curr_time_ - start_time;

    ++count_;
    if(b_stabilized_) { return false; }
    // Zero Torque
    if(!b_stabilized_ && !(state_provider_->task_start_)){
        return _zero_torque_preparing_phase(controller);
    }
    ////////////////////////////////////////////////////////////////////
    // Joint Position Setting
    ////////////////////////////////////////////////////////////////////
    else if(!b_stabilized_ && !(b_move_stabilized_)){
        if(curr_time > stable_move_time_){
        // if(false){
            b_move_stabilized_ = true;
            count_ = 0;
            height_des_ = controller->pos_ini_[2];
            ori_des_ = controller->ori_ini_;
            start_height_ = controller->pos_ini_[2];
            printf("[Joint Position Stabilizing] End \n");
        }
        return _move_to_desired_jpos(curr_time, controller);
    }
    else if(!b_stabilized_ && !(b_foot_move_stabilized_)){
        if(curr_time>stable_move_time_){
        // if(false){
            b_foot_move_stabilized_ = true;
            count_ = 0;
            height_des_ = controller->pos_ini_[2];
            ori_des_ = controller->ori_ini_;
            start_height_ = controller->pos_ini_[2];
            printf("[Foot Position Stabilizing] End \n");
        }
        return _move_to_desired_foot_pos(curr_time, controller);
    }
    //////////////////////////////////////////////////////////////////////////////
    // Lifting the Body a little
    //////////////////////////////////////////////////////////////////////////////
    // else if (b_move_stabilized_ && !b_stabilized_ && curr_time < stable_lifting_time_){
    else if (true){
        return _stable_lifting_height_tilting_roll_task(controller, curr_time);
    }
    else{
        Terrain_Interpreter::GetTerrainInterpreter()->terrain_->CoM_h_ = controller->pos_ini_[2];

        b_stabilized_ = true;
        b_transit_ = false;
        count_ = 0;
        printf("[End of Stabilization]\n");
        return false;
    }
}
bool stabilizing::_move_to_desired_foot_pos(double curr_time, Controller_Hume* controller){
    _set_current_foot_pos_vel();

    static bool first(true);
    static Vector start_lfoot_pos(Vector::Zero(3));
    static Vector start_rfoot_pos(Vector::Zero(3));

    if(first){
        start_lfoot_pos = curr_lfoot_pos_;
        start_rfoot_pos = curr_rfoot_pos_;
        first = false;
    }
    Vector right_foot_vel_des(Vector::Zero(3));
    Vector left_foot_vel_des(Vector::Zero(3));

    Vector right_foot_des = start_rfoot_pos;
    Vector left_foot_des = start_lfoot_pos;
    
    // Right
    right_foot_des[0] = smooth_changing(start_rfoot_pos[0], stable_rfoot_pos_[0], stable_move_time_, curr_time);
    right_foot_vel_des[0] = smooth_changing_vel(start_rfoot_pos[0], stable_rfoot_pos_[0], stable_move_time_, curr_time);
    // Left
    left_foot_des[0] = smooth_changing(start_lfoot_pos[0], stable_lfoot_pos_[0], stable_move_time_, curr_time);
    left_foot_vel_des[0] = smooth_changing_vel(start_lfoot_pos[0], stable_lfoot_pos_[0], stable_move_time_, curr_time);

    
    right_foot_task_->SetTask(right_foot_des,
                              right_foot_vel_des,
                              curr_rfoot_pos_,
                              curr_rfoot_vel_);
    left_foot_task_->SetTask(left_foot_des,
                             left_foot_vel_des,
                             curr_lfoot_pos_,
                             curr_lfoot_vel_);

    (controller->task_array_).push_back(left_foot_task_);    
    (controller->task_array_).push_back(right_foot_task_);

    controller->constraint_ = fixed_constraint_;
    controller->constraint_->updateJcU(state_provider_->Q_, model_);

    _set_current_as_initial(controller);
    controller->wbc_ -> MakeTorque(state_provider_->Q_,
                                   state_provider_->Qdot_,
                                   controller->task_array_,
                                   fixed_constraint_,
                                   model_,
                                   controller->gamma_);

    return true;
}

bool stabilizing::_move_to_desired_jpos(double curr_time, Controller_Hume* controller){
    for (int i(0); i < model_->NAJ(); ++i){
            jpos_des_[i] = smooth_changing(jpos_init_[i], jpos_stable_[i], stable_move_time_, curr_time);
            jvel_des_[i] = smooth_changing_vel(jpos_init_[i], jpos_stable_[i], stable_move_time_, curr_time);
            jacc_des_[i] = smooth_changing_acc(jpos_init_[i], jpos_stable_[i], stable_move_time_, curr_time);
            
    }
    jpos_task_->SetTask(
        jpos_des_, jvel_des_,
        state_provider_->Q_.segment(model_->NPQ(), model_->NAJ()),
        state_provider_->Qdot_.segment(model_->NPQdot(), model_->NAJ() ) ,
        jacc_des_);

    controller->task_array_.push_back(jpos_task_);
    //Save Current Pose
    _set_current_as_initial(controller);
    
    controller->constraint_ = fixed_constraint_;    
    controller->wbc_->MakeTorque(state_provider_->Q_,
                                 state_provider_->Qdot_,
                                 controller->task_array_,
                                 controller->constraint_,
                                 model_,
                                 controller->gamma_);
    return true;
}

void stabilizing::_get_gamma_foot_control(Controller_Hume* controller, double curr_time, Vector & gamma_foot_control){
    _set_current_foot_pos_vel();

    static bool contact_happen (false);
    
    if(state_provider_->right_foot_contact_ && state_provider_->left_foot_contact_){
        contact_happen = true;
    }
    
    std::vector<Task*> task_array_foot;
    double push_down_speed(-0.07);
    double push_forward_speed(0.0);
    // Left Foot
    Vector left_foot_des = init_lfoot_pos_;
    Vector left_foot_vel_des(Vector::Zero(3));
    left_foot_des[2] = init_lfoot_pos_[2] + push_down_speed * curr_time ;

    static int contact_count(0);
    if(contact_happen){
        (left_foot_task_->force_).setZero();
        (right_foot_task_->force_).setZero();

        left_foot_task_->force_[2] = smooth_changing(-10.0, -30.0, 1.0, contact_count * SERVO_RATE);
        right_foot_task_->force_[2] = smooth_changing(-10.0, -30.0, 1.0, contact_count * SERVO_RATE);
        ++contact_count;

        // sejong::pretty_print(left_foot_task_->force_, std::cout, "left force", "");
        // sejong::pretty_print(right_foot_task_->force_, std::cout, "right force", "");

        // left_foot_task_->force_[2] = -20.0;
        // right_foot_task_->force_[2] = -20.0;

        // push_down_speed = -0.3;

    }

    left_foot_vel_des[2] = push_down_speed;
    left_foot_task_->SetTask(left_foot_des,
                             left_foot_vel_des,
                             curr_lfoot_pos_,
                             curr_lfoot_vel_);

    
    // Right Foot
    Vector right_foot_des = init_rfoot_pos_;
    Vector right_foot_vel_des(Vector::Zero(3));
    right_foot_des[2] = init_rfoot_pos_[2] + push_down_speed * curr_time;
    
    right_foot_vel_des[2] = push_down_speed;
    right_foot_task_->SetTask(right_foot_des,
                              right_foot_vel_des,
                              curr_rfoot_pos_,
                              curr_rfoot_vel_);

    // Controller Set
    task_array_foot.push_back(right_foot_task_);
    task_array_foot.push_back(left_foot_task_);
    
    controller->wbc_2_->MakeTorque(state_provider_->Q_,
                                   state_provider_->Qdot_,
                                   task_array_foot,
                                   fixed_constraint_,
                                   model_,
                                   gamma_foot_control);
}

bool stabilizing::_stable_lifting_height_tilting_roll_task(Controller_Hume* controller, double curr_time){
    // Compare two gamma
    Vector gamma_foot_control, gamma_dual_contact;
    Vect3 re_force_left, re_force_right;
    Vect3 re_force_left_dual, re_force_right_dual;
    Vector des(5); // Height, Quaternion
    Vector act(5);
    Vector vel_des(3); // Height, pitch, roll
    Vector vel_act(3);
    static double changing_time(0.0);

    double height_offset(0.0);
    double des_pitch, des_roll;
    //Task for Both Leg
    if(!b_transit_){
        // Foot Control
        _get_gamma_foot_control(controller, curr_time, gamma_foot_control);
        
        Foot_Force_Calculator::Cal_FootForce(RFOOT, gamma_foot_control, re_force_right);
        Foot_Force_Calculator::Cal_FootForce(LFOOT, gamma_foot_control, re_force_left);
        sejong::pretty_print(gamma_foot_control, std::cout, "gamma foot","");
        //Height
        des[0] = state_provider_->CoM_pos_[2];
        vel_des[0] = 0.0;
        start_height_ = des[0];
        // Pitch
        des_pitch = state_provider_->getBodyEulerZYX()[1];
        vel_des[1] = 0.0;
        initial_pitch_ = des_pitch;
        // Roll
        des_roll = state_provider_->getBodyEulerZYX()[2];
        vel_des[2] = 0.0;
        initial_roll_ = des_roll;

        controller->constraint_ = fixed_constraint_;
        controller->constraint_->updateJcU(state_provider_->Q_, model_);
        
    }
    else{
        state_provider_->hold_xy_ = true;
        // Height
        des[0] = smooth_changing(start_height_ + height_offset,
                                 start_height_ + lifting_height_ + height_offset,
                                 stable_lifting_time_ - changing_time,
                                 curr_time - changing_time);
        
        vel_des[0] = smooth_changing_vel(start_height_+ height_offset,
                                         start_height_ + lifting_height_ + height_offset,
                                         stable_lifting_time_ - changing_time,
                                         curr_time - changing_time);
        // Pitch
        des_pitch =  smooth_changing(initial_pitch_, 0.0, stable_lifting_time_ - changing_time, curr_time - changing_time);
        vel_des[1] = smooth_changing_vel(initial_pitch_, 0.0, stable_lifting_time_ - changing_time, curr_time - changing_time);
        // Roll
        des_roll =  smooth_changing(initial_roll_, 0.0, stable_lifting_time_ - changing_time, curr_time - changing_time);
        vel_des[2] = smooth_changing_vel(initial_roll_, 0.0, stable_lifting_time_ - changing_time, curr_time - changing_time);

        controller->constraint_ = dual_contact_;
        controller->constraint_->updateJcU(state_provider_->Q_, model_);
    }
    
    sejong::Quaternion des_quaternion;
    sejong::convert(0.0, des_pitch, des_roll, des_quaternion);
    des[0] = start_height_;
    
    des[1] = des_quaternion.w();
    des[2] = des_quaternion.x();
    des[3] = des_quaternion.y();
    des[4] = des_quaternion.z();
    // sejong::pretty_print(des_quaternion, std::cout, "des_quat", "");
    // sejong::pretty_print(des, std::cout, "des", "");
    act[0] = state_provider_->CoM_pos_[2];
    act.segment(1,4) = state_provider_->getBodyOri();
    
    vel_act[0] = state_provider_->CoM_vel_[2];
    vel_act[1] = state_provider_->Qdot_[4];
    vel_act[2] = state_provider_->Qdot_[3];
    
    controller->task_array_.clear();
    COM_height_tilting_roll_task_->SetTask(des, vel_des, act, vel_act);
    
    for (int i(0); i<5; ++i){
        state_provider_->task_des_[i] = des[i];
        state_provider_->task_curr_[i] = act[i];
    }
        
    controller->task_array_.push_back(COM_height_tilting_roll_task_);
    
    controller->wbc_->MakeTorque(state_provider_->Q_,
                                 state_provider_->Qdot_,
                                 controller->task_array_,
                                 dual_contact_,
                                 model_,
                                 gamma_dual_contact);
    // Test
    sejong::Vector cent_des, cent_vel_des;
    controller->CentroidalMomentControl(cent_des,
                                        cent_vel_des,
                                        gamma_dual_contact);
    static bool both_contact(false);
    
    if(state_provider_->right_foot_contact_ && state_provider_->left_foot_contact_){
        both_contact = true;
    }

    if(!b_transit_  && re_force_right[2] < -20.0 && re_force_left[2] < -20.0 && both_contact){
    // if(!b_transit_ && both_contact){
        Vect3 base_pt_pos;
        model_->getPosition(state_provider_->Q_,
                            controller->base_pt_, base_pt_pos);
        
        state_provider_->height_offset_ = 0.0;//-base_pt_pos[2];
        // start_height_ += -base_pt_pos[2];
        printf("[XZ Tilting Stabilization] Change to Dual Contact Constraint\n");
        b_transit_ = true;
        changing_time = curr_time;
    }
    // b_transit_ = false;
    if(b_transit_){
        controller->gamma_ = gamma_dual_contact;
    }
    else{
        controller->gamma_ = gamma_foot_control;
    }
    controller->pos_ini_[2] = des[0];
    controller->ori_ini_ = des_quaternion;

    return true;
}

void process::_set_current_as_initial(Controller_Hume* controller){

    controller->pos_ini_ = state_provider_->CoM_pos_;
    controller->ori_ini_ = state_provider_->getBodyOriQuat();

    controller->left_foot_ini_ = state_provider_->getFootPos(LFOOT);
    controller->right_foot_ini_ = state_provider_->getFootPos(RFOOT);

    init_lfoot_pos_ = controller->left_foot_ini_;
    init_rfoot_pos_ = controller->right_foot_ini_;
    for (int i(0); i<3 ; ++i){
        init_lfoot_pos_[i] -= state_provider_->Q_[i];
        init_rfoot_pos_[i] -= state_provider_->Q_[i];
    }
}

void stabilizing::_set_current_foot_pos_vel(){
    curr_lfoot_pos_ = state_provider_->getFootPos(LFOOT);
    curr_rfoot_pos_ = state_provider_->getFootPos(RFOOT);

    curr_lfoot_vel_ = state_provider_->getFootVel(LFOOT);
    curr_rfoot_vel_ = state_provider_->getFootVel(RFOOT);

    for(int i(0); i<3 ; ++i){
        curr_lfoot_pos_[i] -= state_provider_->Q_[i];
        curr_rfoot_pos_[i] -= state_provider_->Q_[i];
        curr_lfoot_vel_[i] -= state_provider_->Qdot_[i];
        curr_rfoot_vel_[i] -= state_provider_->Qdot_[i];
    }
}

bool stabilizing::_zero_torque_preparing_phase(Controller_Hume* controller){
    controller->gamma_ = Vector::Zero(model_->NAJ());

    controller->constraint_ = fixed_constraint_;    
    controller->constraint_->updateJcU(state_provider_->Q_, model_);

    for (int i(0); i< model_->NAJ(); ++i){
        jpos_init_[i] = state_provider_->Q_[i + model_->NPQ()];
    }
    count_ = 0;
    state_provider_->task_start_ = true;
    return true;
}
