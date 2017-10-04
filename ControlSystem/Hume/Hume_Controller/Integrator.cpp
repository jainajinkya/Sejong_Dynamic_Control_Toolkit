#include "Integrator.h"
#include <utils/utilities.h>
#include <iostream>
#include "Hume_Model/Hume_Model.h"
#include "StateProvider.h"

StateProvider* _state_provider = StateProvider::GetStateProvider();
HumeModel* _hume_model = HumeModel::GetHumeModel();

Integrator::Integrator(double dt): dt_(dt){
}
Integrator::~Integrator(){}

void Integrator::ForwardDynamics(Eigen::Matrix<double, NUM_Q, 1> & Q,
                                 Eigen::Matrix<double, NUM_QDOT, 1> & Qdot,
                                 const Eigen::Matrix<double, NUM_ACT_JOINT, 1>  & torque_command,
                                 const sejong::Matrix & Jc){
    
    Eigen::Matrix<double, NUM_QDOT,1> torque;
    torque.setZero();
    torque.tail(NUM_ACT_JOINT) = torque_command;

    sejong::Matrix MassInv_;
    sejong::Vector Coriolis_;
    sejong::Vector Gravity_;

    _hume_model->getInverseMassInertia(MassInv_);
    _hume_model->getCoriolis(Coriolis_);
    _hume_model->getGravity(Gravity_);

    sejong::Matrix J_constraint = Jc;
    
    if(_state_provider->right_foot_contact_
       && _state_provider->left_foot_contact_){

        sejong::Matrix J_tmp(6, NUM_QDOT);
        J_tmp.setZero();
        J_constraint = sejong::Matrix(6, NUM_QDOT);

        _hume_model->getFullJacobian(_state_provider->Q_, RFOOT, J_tmp);
        J_constraint.block(0,0, 3, NUM_QDOT) = J_tmp.block(3,0, 3, NUM_QDOT);
        
        J_tmp.setZero();
        _hume_model->getFullJacobian(_state_provider->Q_, LFOOT, J_tmp);
        J_constraint.block(3,0, 3, NUM_QDOT) = J_tmp.block(3,0, 3, NUM_QDOT);
    }
    else if (_state_provider->right_foot_contact_){
        sejong::Matrix J_tmp(6, NUM_QDOT);
        J_tmp.setZero();

        _hume_model->getFullJacobian(_state_provider->Q_, RFOOT, J_tmp);
        J_constraint = J_tmp.block(3,0, 3, NUM_QDOT);
    }
    else if (_state_provider->left_foot_contact_){
        sejong::Matrix J_tmp(6, NUM_QDOT);
        J_tmp.setZero();

        _hume_model->getFullJacobian(_state_provider->Q_, LFOOT, J_tmp);
        J_constraint = J_tmp.block(3,0, 3, NUM_QDOT);
    }
    
    sejong::Matrix ltmp = J_constraint * MassInv_ * J_constraint.transpose();
    sejong::Matrix Lambda = ltmp.inverse();

    sejong::Vector Qddot;
    if(false){
        
        sejong::Matrix Nc;
        Nc = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - MassInv_ * J_constraint.transpose()*Lambda*J_constraint;
        
        Qddot = MassInv_ * Nc.transpose()*(torque - Coriolis_ - Gravity_ );

        if(StateProvider::GetStateProvider()->hold_xy_){
            Qddot.head(2) += ( 150.0*(-Q.head(2)) + 15.0 *(-Qdot.head(2)) );
            Qddot[5] += (150.5 * (-Q[5]) + 0.03*(Qdot[5]));
            Qdot[5] = 0.0;
            Q[5] = 0.0;
        }

        Qdot = Qdot + Qddot * dt_;// - 0.1*Qdot*dt_;

    }
    else{
        Qddot = MassInv_ * (torque - Coriolis_ - Gravity_);

        // if(StateProvider::GetStateProvider()->hold_xy_){
        //     Qddot.head(2) += ( 150.0*(-Q.head(2)) + 15.0 *(-Qdot.head(2)) );
        //     Qddot[5] += (150.5 * (-Q[5]) + 0.03*(Qdot[5]));
        //     Qdot[5] = 0.0;
        //     Q[5] = 0.0;
        // }

        Qdot = Qdot + Qddot * dt_;// - 0.1*Qdot*dt_;
        sejong::Vector rf_vel = Lambda * (J_constraint * Qdot);
        Qdot = Qdot - MassInv_ * J_constraint.transpose() * rf_vel;
        // sejong::Vector x_vel = Jc*Qdot;
        // std::cout<<"x vel:\n"<<x_vel<<std::endl;
    }
    // Qdot.tail(NUM_ACT_JOINT) -= 0.1*Qdot.tail(NUM_ACT_JOINT)*dt_;// + Qddot.tai * dt_;// - 0.1*Qdot*dt_;

    // if(StateProvider::GetStateProvider()->contact_constraint_){
    // }
    _ExtrinsicIntegrateConfiguration(Q, Qdot);
}

void Integrator::_ExtrinsicIntegrateConfiguration(Eigen::Matrix<double, NUM_Q, 1> & Q,
                                                  const Eigen::Matrix<double, NUM_QDOT, 1> & Qdot){
    // Position
    for (int i(0); i<3; ++i){
        Q[i] = Q[i] + Qdot[i] * dt_;
    }
    // Orientation
    sejong::Quaternion dq;
    sejong::Vector theta(3);
    theta[0] = Qdot[3] * dt_;
    theta[1] = Qdot[4] * dt_;
    theta[2] = Qdot[5] * dt_;
    sejong::convert(theta, dq);
    
    sejong::Quaternion q(Q[NUM_QDOT], Q[3], Q[4], Q[5]);

    sejong::Quaternion q_n = sejong::QuatMultiply(dq, q);
    
    Q[3] = q_n.x();
    Q[4] = q_n.y();
    Q[5] = q_n.z();
    Q[NUM_QDOT] = q_n.w();

    // Joint
    for (int i(0); i<NUM_ACT_JOINT; ++i){
        Q[i + 6] = Q[i + 6] + Qdot[i + 6] * dt_;
    }
}
