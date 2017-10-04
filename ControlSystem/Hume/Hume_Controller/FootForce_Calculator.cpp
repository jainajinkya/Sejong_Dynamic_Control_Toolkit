#include "FootForce_Calculator.h"
#include "Hume_Model/Hume_Model.h"
#include "StateProvider.h"

sejong::Matrix Jsbar_tr_U_tr_dual_contact_;
sejong::Vector Jsbar_tr_grav_coriolis_dual_contact_;


void Foot_Force_Calculator::Cal_FootForce(SJLinkID _link_id, const Vector& torque, Vect3 & _force){
    if(Jsbar_tr_grav_coriolis_dual_contact_.rows()<6 || Jsbar_tr_U_tr_dual_contact_.rows()<6){
        _force.setZero();
        return ;
    }
    Vector force =
        Jsbar_tr_U_tr_dual_contact_ * torque - Jsbar_tr_grav_coriolis_dual_contact_;

    int idx_offset(0);
    if(_link_id == LFOOT){
        idx_offset = 3;
    }
    for (int i(0); i< 3; ++i){
        _force[i] = force[idx_offset + i];
    }
}


void Foot_Force_Calculator::run(){
    while(true){
        Prepare_FootForce_Computation();
        usleep(5000);
    }
}

Foot_Force_Calculator::Foot_Force_Calculator(){
    model_ = HumeModel::GetHumeModel();

    U_ = sejong::Matrix::Zero(NUM_ACT_JOINT, NUM_QDOT);
    U_.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(model_->NAJ(), model_->NAJ());

}

Foot_Force_Calculator::~Foot_Force_Calculator(){}

void Foot_Force_Calculator::Prepare_FootForce_Computation(){
    if(!StateProvider::GetStateProvider()->initialized_){
        return;
    }
    sejong::Matrix Jsbar, ainv, Lambda, Jtmp;
    Vector grav;
    Vector coriolis;

    sejong::Matrix Js(sejong::Matrix::Zero(6, model_->NQdot()));

    //Right Leg
    model_->getFullJacobian(StateProvider::GetStateProvider()->Q_, RFOOT, Jtmp);
    Js.block(0,0,3, model_->NQdot()) = Jtmp.block(3,0, 3, model_->NQdot());
    
    //Left Leg
    model_->getFullJacobian(StateProvider::GetStateProvider()->Q_, LFOOT, Jtmp);
    Js.block(3,0,3, model_->NQdot()) = Jtmp.block(3,0, 3, model_->NQdot());

    model_->getGravity(grav);
    model_->getInverseMassInertia(ainv);
    model_->getCoriolis(coriolis);

    sejong::Matrix lambda_tmp (Js * ainv * Js.transpose());
    Lambda = lambda_tmp.inverse();
    
    Jsbar = ainv * Js.transpose() * Lambda;

    Jsbar_tr_U_tr_dual_contact_ = Jsbar.transpose() * U_.transpose();
    Jsbar_tr_grav_coriolis_dual_contact_ = Jsbar.transpose() * (grav + coriolis);
}

