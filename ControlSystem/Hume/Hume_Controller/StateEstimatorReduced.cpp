#include "StateEstimatorReduced.h"


StateEstimatorReduced::StateEstimatorReduced()
    :
    StateEstimator<NUM_OBS_REDUCE, NUM_STATE_REDUCE, NUM_STATE_DOT_REDUCE, NUM_NOISE_REDUCE>(){

    MoCap_receiver_ = new MoCapReceiver();
    MoCap_receiver_->start();

    x_est_.setZero();
    x_pred_.setZero();

    // L_.setIdentity();
    L_.setZero();
}

StateEstimatorReduced::~StateEstimatorReduced(){ }

void StateEstimatorReduced::Initialize_Estimator(_DEF_SENSOR_DATA_){
    x_est_.setZero();
    x_pred_.setZero();

    virtual_configuration_.setZero();
    virtual_vel_.setZero();

    for(int i(0); i < NUM_ACT_JOINT; ++i){
        virtual_configuration_[i + 6] = jpos[i];
    }
    x_est_[2] = 0.76;
    x_pred_[2] = 0.76;
    
    x_est_[9] = 1.0;
    x_pred_[9] = 1.0;
    virtual_configuration_[NUM_QDOT] = 1.0;
    _UpdateStateProvider(_VAR_SENSOR_DATA_);
}

void StateEstimatorReduced::_GenerateObservationVector(_DEF_SENSOR_DATA_){
    acc_ << accelerometer[0], accelerometer[1], accelerometer[2];
    omega_<< ang_vel[0], ang_vel[1], ang_vel[2];
    // std::cout<<"acc:\n"<<acc_<<std::endl;
    // std::cout<<"omega:\n"<<omega_<<std::endl;
    // MoCap Data
    Eigen::Matrix<double, 3*NUM_LED_OBS, 1> MoCap_Data;
    MoCap_receiver_->getMoCapData(MoCap_Data, LED_visibility_);
    for(int i(0); i< 3 * NUM_LED_REDUCE; ++i) obs_[i] = MoCap_Data[i];
}

void StateEstimatorReduced::_GenerateObservationModel(){
    z_.setZero();
    H_.setZero();
    R_.setIdentity();

    R_*=2.5;
    // R_/=SERVO_RATE;
    
    // LED Visibility Check
    for (int i(0); i<NUM_LED_REDUCE; ++i){
        if(!LED_visibility_[i]){
            R_.block<3,3>(3*i, 3*i) *= 100000.0;
        }
    }
    // LED Position
    sejong::Vect3 pos;
    int led_id = LED_BODY_0;
    int idx_offset(0);
    for(int i(0); i<NUM_LED_REDUCE; ++i){
        model_->getPosition(virtual_configuration_, led_id, pos);
        z_.segment(idx_offset, 3) = pos;

        idx_offset +=3;
        ++led_id;
    }
    //************ Make 'H'  ****************
    // LED
    sejong::Matrix J_LED;
    for (int i(0); i<NUM_LED_REDUCE; ++i){
        model_->getFullJacobian(virtual_configuration_, LED_BODY_0 +i, J_LED);
        H_.block<3, 6>(3*i, 0) = J_LED.block(3,0, 3, 6);
    }
}

void StateEstimatorReduced::_GeneratePredictionModel(){
    // Orienation
    sejong::Quaternion dq;
    sejong::Vector theta(3);
    theta = (omega_ + x_est_.segment(13,3) ) * SERVO_RATE;
    sejong::convert(theta, dq);

    // Test
    // double rot = M_PI/6.0;
    // sejong::Quaternion q_test(cos(rot/2.0), 0, 0, sin(rot/2.0));
    // Eigen::Matrix<double, 3,3> C_test(q_test);
    // sejong::pretty_print(q_test, std::cout, "q_test","");
    // std::cout<<"C_test:\n"<<C_test<<std::endl;

    
    sejong::Quaternion q(x_est_[9],
                         x_est_[6],
                         x_est_[7],
                         x_est_[8]);

    sejong::Quaternion q_n = sejong::QuatMultiply(q, dq);

    // Accleration
    Eigen::Matrix<double, 3,1> grav;
    grav.setZero();
    grav[2] = -9.8;
    Eigen::Matrix<double, 3,3> C(q);
    // std::cout<<"acc:\n"<<acc_<<std::endl;
    Eigen::Matrix<double,3,1> acceleration =
        C*(acc_ - x_est_.segment(10,3)) + grav;
        // 0.0 * grav;
    
    // Position
    x_pred_.head(3) = x_est_.head(3) + x_est_.segment(3,3)*SERVO_RATE;

    // std::cout<<"acc:\n"<<acc_<<std::endl;
    // std::cout<<"acceleration: \n"<<acceleration<<std::endl;
    
    x_pred_.segment(3,3) = x_est_.segment(3,3) + acceleration*SERVO_RATE;

    x_pred_[6] = q_n.x();
    x_pred_[7] = q_n.y();
    x_pred_[8] = q_n.z();
    x_pred_[9] = q_n.w();
    
    L_.block<3,3>(0,0).setIdentity();
    L_.block<3,3>(3,3) = -C;
    // L_.block<3,3>(6,6) = sejong::Matrix::Identity(3,3);
    L_.block<3,3>(6,6) = C;
    L_.block<6,6>(9,9).setIdentity();
    
    
    F_.setIdentity();
    Qc_.setIdentity();
    Qc_ *= 5.0;
    // Qc_.block<9,9>(0,0)*=10.5;
    // position
    F_.block<3, 3>(0, 3) = sejong::Matrix::Identity(3, 3) * SERVO_RATE;

    Eigen::Matrix<double, 3,3> acc_skew;
    acc_skew<<
        0.0, -acc_[2], acc_[1],
        acc_[2], 0.0, -acc_[0],
        -acc_[1], acc_[0], 0.0;

    // Velocity
    F_.block<3,3>(3, 6) = -C * acc_skew * SERVO_RATE;
    F_.block<3,3>(3, 9) = -C * SERVO_RATE;

    // Quaternion
    // Eigen::Matrix<double, 3,3> w_mt;
    // w_mt <<
    //     0.0, -omega_[2], omega_[1],
    //     omega_[2], 0.0, -omega_[0],
    //     -omega_[1], omega_[0], 0.0;
    // F_.block<3,3>(6, 6) = -w_mt *SERVO_RATE;
    // F_.block<3,3>(6, 12) = -sejong::Matrix::Identity(3,3)*SERVO_RATE;
    F_.block<3,3>(6,12) = C * SERVO_RATE;

    // std::cout<<"F:\n"<<F_<<std::endl;
}

void StateEstimatorReduced::_IntegrateState(){
    x_est_.head(6) = x_pred_.head(6) + err_.head(6);

    // Orientation Update
    sejong::Quaternion dq;
    sejong::Vector theta;
    theta = err_.segment(6, 3);
    sejong::convert(theta, dq);
    sejong::Quaternion q(x_pred_[9],
                         x_pred_[6],
                         x_pred_[7],
                         x_pred_[8]);

    sejong::Quaternion q_n = sejong::QuatMultiply(dq, q);

    x_est_[6] = q_n.x();
    x_est_[7] = q_n.y();
    x_est_[8] = q_n.z();
    x_est_[9] = sqrt(1 - q_n.x()* q_n.x() - q_n.y() * q_n.y() - q_n.z()*q_n.z());

    x_est_.segment(9,3) = x_pred_.segment(9,3) + err_.segment(9,3);
    x_est_.tail(3) = x_pred_.tail(3) + err_.tail(3);
}

void StateEstimatorReduced::_UpdateStateProvider(_DEF_SENSOR_DATA_){
    virtual_configuration_.head(3) = x_est_.head(3);
    virtual_configuration_.segment(3,3) = x_est_.segment(6,3);
    virtual_configuration_[NUM_QDOT] = x_est_[9];

    virtual_vel_.head(3) = x_est_.segment(3,3);
    virtual_vel_.segment(3,3) = omega_;

    
    state_provider_->Q_ = virtual_configuration_;
    state_provider_->Qdot_ = virtual_vel_;
    state_provider_->Q_[2] = virtual_configuration_[2];// + state_provider_->height_offset_;
    
    state_provider_->Q_est_.head(3) = x_est_.head(3);
    state_provider_->Q_est_.segment(3,3) = x_est_.segment(6, 3);
    state_provider_->Q_est_[NUM_QDOT] = x_est_[9];
    state_provider_->Qdot_est_.head(3) = x_est_.segment(3,3);

    // state_provider_->Q_.segment(3,3) = x_est_.segment(6,3);
    // state_provider_->Q_[NUM_QDOT] = x_est_[9];

    // state_provider_->Qdot_.head(3) = x_est_.segment(3,3);
    // state_provider_->Qdot_.segment(3,3) = omega_;
    
    model_->UpdateModel(state_provider_->Q_,
                        state_provider_->Qdot_);

    //Contact 
    state_provider_->right_foot_contact_ = _right_foot_contact; 
    state_provider_->left_foot_contact_ = _left_foot_contact;
    state_provider_->Foot_Contact_[0] = (int)_right_foot_contact;
    state_provider_->Foot_Contact_[1] = (int)_left_foot_contact;

    // CoM
    model_->getCoMPosition(state_provider_->Q_, state_provider_->CoM_pos_);
    model_->getCoMVelocity(state_provider_->Q_,
                           state_provider_->Qdot_,
                           state_provider_->CoM_vel_);

    // Foot Pos & Vel
    model_->getPosition(state_provider_->Q_, RFOOT,
                        state_provider_->rfoot_pos_);
    model_->getPosition(state_provider_->Q_, LFOOT,
                        state_provider_->lfoot_pos_);
    
    model_->getVelocity(state_provider_->Q_,
                        state_provider_->Qdot_, RFOOT,
                        state_provider_->rfoot_vel_);
    model_->getVelocity(state_provider_->Q_,
                        state_provider_->Qdot_, LFOOT,
                        state_provider_->lfoot_vel_);
}
