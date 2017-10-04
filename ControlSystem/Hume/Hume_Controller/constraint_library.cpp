#include "constraint_library.h"

#include <iostream>
#include <fstream>
#include <WBOSC/WBOSC_Model.h>
#include "utils/utilities.h"
#include "StateProvider.h"

namespace sejong {
////////////   Fixed Constraint  ////////////////
    bool Hume_Fixed_Constraint::updateJcU(const Vector & conf, WBOSC_Model* _model){
        Jc_ = Matrix::Zero(_model->NPQdot(), _model->NQdot());
        Jc_.block(0,0, _model->NPQdot(), _model->NPQdot()) = Matrix::Identity(_model->NPQdot(), _model->NPQdot());

        //Update U
        U_ = Matrix::Zero(_model->NAJ(), _model->NQdot());
        U_.block(0, _model->NPQdot(), _model->NAJ(), _model->NAJ()) = Matrix::Identity(_model->NAJ(), _model->NAJ());

        // StateProvider::GetStateProvider()->contact_constraint_ = false;
        return true;
    }
//////////// RThigh Fixed Constraint  ////////////////
    bool Hume_RThigh_Fixed_Constraint::updateJcU(const Vector & conf, WBOSC_Model* _model){
        Jc_ = Matrix::Zero(6, _model->NQdot());
        Jc_.setZero();
        _model->getFullJacobian(conf, RThigh, Jc_);

        //Update U
        U_ = Matrix::Zero(_model->NAJ(), _model->NQdot());
        U_.block(0, _model->NPQdot(), _model->NAJ(), _model->NAJ()) = Matrix::Identity(_model->NAJ(), _model->NAJ());
        return true;
    }

////////////   No Constraint  ////////////////
    bool Hume_No_Constraint::updateJcU(const Vector & conf, WBOSC_Model* _model){
        Jc_ = Matrix::Zero(1, _model->NQdot());
        //Update U
        U_ = Matrix::Zero(_model->NAJ(), _model->NQdot());
        U_.block(0, _model->NPQdot(), _model->NAJ(), _model->NAJ()) = Matrix::Identity(_model->NAJ(), _model->NAJ());
                
        // StateProvider::GetStateProvider()->contact_constraint_ = false;
        return true;
    }

////////////   Both Constraint  ////////////////
    Hume_Contact_Both::Hume_Contact_Both(): WBC_Constraint(){
        b_internal_ = true;
        num_constraint_ = 6;
        b_time_derivative_J_ = false;
    }
        
    bool Hume_Contact_Both::updateJcU(const Vector & conf, WBOSC_Model * _model)
    {
        Matrix Jfull(6, _model->NQdot());
        Jfull.setZero();
        Jc_ = Matrix::Zero(6, _model->NQdot());
        //Right Leg
        _model->getFullJacobian(conf, RFOOT, Jfull);
        Jc_.block(0,0,3, _model->NQdot()) = Jfull.block(3,0,3,_model->NQdot());
        //Left Leg
        Jfull.setZero();
        _model->getFullJacobian(conf, LFOOT, Jfull);
        Jc_.block(3,0,3,_model->NQdot()) = Jfull.block(3,0,3,_model->NQdot());

        //Update U
        U_ = Matrix::Zero(_model->NAJ(), _model->NQdot());
        U_.block(0, _model->NPQdot(), _model->NAJ(), _model->NAJ()) = Matrix::Identity(_model->NAJ(), _model->NAJ());
        
        Update_Internal_Matrix(conf, _model);

        StateProvider::GetStateProvider()->contact_constraint_ = true;
        return true;
    }

    void Hume_Contact_Both::Update_Internal_Matrix(const Vector & q, WBOSC_Model* _model){
        W_int_ = Matrix::Zero(1,6);
        Matrix R = Matrix::Zero(3,3);
        Vect3 lfoot_pos, rfoot_pos;
        _model->getPosition(q,  LFOOT, lfoot_pos);
        _model->getPosition(q,  RFOOT, rfoot_pos);

        Vector foot_dist ( rfoot_pos-lfoot_pos );
        
        double length(0.0);
        for (int i(0); i< 3; ++i){
            length += pow(foot_dist[i], 2.0);
        }
        length = sqrt(length);
        
        // R(0,0) = foot_dist[0]/ length;
        // R(0,1) = foot_dist[1]/ length;
        // R(1,0) = -R(0,1);
        // R(1,1) = R(0, 0);
        // R(2,2) = 1;

        R.block(0,0, 1, 3) = (foot_dist/ length).transpose();
        R(1,0) = -R(0,1);
        R(1,1) = R(0,0);
        Eigen::Vector3d x(R(0,0), R(0,1), R(0,2));
        Eigen::Vector3d y(R(1,0), R(1,1), R(1,2));
        Eigen::Vector3d x_y(x.cross(y));
        R(2,0) = x_y(0);
        R(2,1) = x_y(1);
        R(2,2) = x_y(2);
        
        Matrix d_t = Matrix::Zero(3,6);
        d_t.block(0, 0, 3, 3) = Matrix::Identity(3,3);
        d_t.block(0,3, 3,3) = -Matrix::Identity(3,3);
        
        Matrix S = Matrix::Zero(1,3);
        S(0, 0) = 1.0;

        W_int_ = S * R * d_t;
    }
    void Hume_Contact_Both::set_constraint_weight(const Vector & weight){
        constraint_weight_ = weight;
        b_constraint_weight_ = true;
    }
    void Hume_Contact_Both::getDesInternalForce(Vector & des_int_force){
        des_int_force = Vector::Zero(1);
        des_int_force[0] = 0.0;
    }

////////  Point Contact
    Hume_Point_Contact::Hume_Point_Contact(SJLinkID id): WBC_Constraint(), foot_(id) {
        b_internal_ = false;
        b_re_force_ctrl_ = false;
        num_constraint_ = 3;
        b_time_derivative_J_ = false;
    }

    bool Hume_Point_Contact::updateJcU(const Vector & conf, WBOSC_Model * _model){
        Matrix Jfull;
        switch(foot_){
        case RFOOT:
            //Right Leg
            _model->getFullJacobian(conf, RFOOT, Jfull);
            break;
        case LFOOT:
            //Left leg
            _model->getFullJacobian(conf, LFOOT, Jfull);
            break;
        default:
            printf("[Hume Point Contact] Wrong link ID \n");
        }
        Jc_ = Jfull.block(3,0, 3, _model->NQdot());
        
        //Update U
        U_ = Matrix::Zero(_model->NAJ(), _model->NQdot());
        U_.block(0, _model->NPQdot(), _model->NAJ(), _model->NAJ()) = Matrix::Identity(_model->NAJ(), _model->NAJ());
        if(b_constraint_weight_){
            for(int i(0); i<3; ++i){
                Jc_.block(i,0, 1, _model->NQdot()) = constraint_weight_[i] * Jc_.block(i, 0, 1, _model->NQdot());
            }
        }
        StateProvider::GetStateProvider()->contact_constraint_ = true;
        return true;
    }
    Hume_Point_Contact_Left::Hume_Point_Contact_Left():Hume_Point_Contact(LFOOT){}
    Hume_Point_Contact_Right::Hume_Point_Contact_Right():Hume_Point_Contact(RFOOT){}

}
