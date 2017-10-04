#include "Controller_Hume.h"
#include <math.h>
#include "utils/pseudo_inverse.hpp"
#include "WBOSC/Constraint.hpp"
#include <utils/utilities.h>

Controller_Hume::Controller_Hume()
    : constraint_(NULL), motion_start_(false), phase_(10),
      pos_ini_(3), left_foot_ini_(3), right_foot_ini_(3),
      b_int_gain_right_(true), b_int_gain_left_(true),
      b_force_int_gain_left_knee_(false), b_force_int_gain_right_knee_(false),
      base_pt_(HIP){

    model_ = HumeModel::GetHumeModel();
    state_provider_ = StateProvider::GetStateProvider();
    
    wbc_ = new Controller_WBC(1);
    wbc_2_ = new Controller_WBC(2);
    // Test
    // wbc_->b_ignore_gravity_ = true;
    // wbc_2_->b_ignore_gravity_ = true;
    printf("[Controller Hume] Construction\n");
}

Controller_Hume::~Controller_Hume(){
    if(constraint_ !=NULL){
        delete constraint_;
    }
}
// CoM Height + 3 DoF Orientation
void Controller_Hume::CentroidalMomentControl_DoubleContact(
    const sejong::Vector & cent_des, 
    const sejong::Vector & cent_vel_des,
    sejong::Vector & fr){
    
    
}


void Controller_Hume::CentroidalMomentControl(const sejong::Vector & cent_des,
                                              const sejong::Vector & cent_vel_des,
                                              sejong::Vector & gamma){
    model_->UpdateModel(state_provider_->Q_sim_,
                        state_provider_->Qdot_sim_);
    
    sejong::Matrix Ainv;
    model_->getInverseMassInertia(Ainv);

    sejong::Matrix J_cent;
    model_->getCentroidJacobian(J_cent);

    /////// Foot Contact
    // sejong::Matrix J_foot(6, NUM_QDOT);
    // sejong::Matrix Jtmp;

    // model_->getFullJacobian(state_provider_->Q_, LFOOT, Jtmp);
    // J_foot.block(0, 0, 3, NUM_QDOT) = Jtmp.block(3,0, 3, NUM_QDOT);

    // model_->getFullJacobian(state_provider_->Q_, RFOOT, Jtmp);
    // J_foot.block(3, 0, 3, NUM_QDOT) = Jtmp.block(3,0, 3, NUM_QDOT);

    // sejong::Matrix Nc;

    // Nc = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Ainv * J_foot.transpose() * (J_foot * Ainv * J_foot.transpose()).inverse() * J_foot;

    // J_cent = J_cent * Nc;
  
    sejong::Matrix Lambda ( J_cent * Ainv * J_cent.transpose());
    Lambda = Lambda.inverse();

    sejong::Vect3 com_vel;
    model_->getCoMVelocity(state_provider_->Q_,
                           state_provider_->Qdot_,
                           com_vel);
    static sejong::Vect3 com_vel_pre = sejong::Vector::Zero(3);
    static sejong::Vector qdot_pre = sejong::Vector::Zero(NUM_QDOT);

    sejong::Vect3 com_acc = (com_vel - com_vel_pre)/SERVO_RATE;

    sejong::Matrix J_com;
    model_->getCoMJacobian(state_provider_->Q_, J_com);
    sejong::Matrix qacc = (state_provider_->Qdot_sim_ - qdot_pre)/SERVO_RATE;
    
    sejong::Vector JdotQdot = com_acc - J_com * qacc;

    sejong::pretty_print(com_vel, std::cout, "com_vel", "");
    sejong::pretty_print(com_vel_pre, std::cout, "com_pre","");
    sejong::pretty_print(com_acc, std::cout, "com_acc","");
    sejong::pretty_print(JdotQdot,std::cout, "Jdot qdot","");
    sejong::Vector mu_extra = Lambda.block(3,3,3,3) * JdotQdot;

    
    sejong::Vector coriolis, grav;
    model_->getCoriolis(coriolis);
    model_->getGravity(grav);
    
    sejong::Vector mu (Lambda * J_cent * Ainv * coriolis);
    sejong::Vector p (Lambda * J_cent * Ainv * grav);

    sejong::pretty_print(J_cent, std::cout, "Jcentroid","");
    sejong::pretty_print(J_com, std::cout, "J_com","");
    sejong::pretty_print(mu, std::cout, "mu", "");
    sejong::pretty_print(mu_extra, std::cout, "mu_extra", "");

    sejong::pretty_print(coriolis, std::cout, "coriolis","");
    sejong::pretty_print(p, std::cout, "grav", "");
    sejong::pretty_print(state_provider_->Qdot_sim_, std::cout,"Qdot","");
    sejong::pretty_print(state_provider_->Q_sim_, std::cout,"Q","");
    sejong::pretty_print(state_provider_->Qddot_sim_, std::cout, "Qddot","");
    sejong::pretty_print(qacc, std::cout, "q_acc","");

    sejong::pretty_print(Lambda, std::cout, "Lambda", "");

    com_vel_pre = com_vel;
    qdot_pre = state_provider_->Qdot_sim_;
    static int count(0);
    ++count;
    if(count > 2){
        exit(0);
    }
}

void Controller_Hume::_PreProcessing_Command(){
    // b_int_gain_right_ = true;
    // b_int_gain_left_ = true;
    // b_force_int_gain_left_knee_ = false;
    // b_force_int_gain_right_knee_ = false;
    task_array_.clear();

    // Test
    // Vect3 base_pt_pos;
    // Vect3 base_pt_vel;

    // model_->getVelocity(state_provider_->Q_,
    //                     state_provider_->Qdot_,
    //                     base_pt_, base_pt_vel);

    for (int i(0); i<3; ++i){
        // state_provider_->Q_[i] = -base_pt_pos[i];
        // state_provider_->Qdot_[i] = -base_pt_vel[i];

        // Upright Assumption
        // state_provider_->Q_[i+3] = 0.0;
    }
    // state_provider_->Q_[NUM_QDOT] = 1.0;
    // model_->UpdateModel(state_provider_->Q_,
    //                     state_provider_->Qdot_);

    // model_->getCoMPosition(state_provider_->Q_, state_provider_->CoM_pos_);
    // model_->getCoMVelocity(state_provider_->Q_,
    //                        state_provider_->Qdot_,
    //                        state_provider_->CoM_vel_);

    state_provider_->Body_Euler_ori_.setZero();
}

void Controller_Hume::_PostProcessing_Command(std::vector<double> & command){
    command.resize(model_->NAJ());
    for (int i(0); i < model_->NAJ(); ++i){
        command[i] = gamma_(i);
    }
}
