#include "StateEstimatorVirtual.h"


StateEstimatorVirtual::StateEstimatorVirtual()
    :
    StateEstimator<NUM_OBS_VIRTUAL, NUM_STATE_VIRTUAL, NUM_STATE_DOT_VIRTUAL, NUM_NOISE_VIRTUAL>(), J_(6, NUM_QDOT){

    MoCap_receiver_ = new MoCapReceiver();
    MoCap_receiver_->start();

    x_est_.setZero();
    x_pred_.setZero();

    L_.setZero();

    J_.setZero();
    (J_.block<6,6>(0,0)).setIdentity();
    printf("[State Estimator Virtual] Contructed\n");
}

StateEstimatorVirtual::~StateEstimatorVirtual(){ }

void StateEstimatorVirtual::Initialize_Estimator(_DEF_SENSOR_DATA_){
    x_est_.setZero();
    x_pred_.setZero();

    virtual_configuration_.setZero();
    virtual_vel_.setZero();

    for(int i(0); i < NUM_ACT_JOINT; ++i){
        virtual_configuration_[i + 6] = jpos[i];
    }
    x_est_[2] = 0.76;
    x_pred_[2] = 0.76;
    
    x_est_[6] = 1.0;
    x_pred_[6] = 1.0;

    virtual_configuration_[NUM_QDOT] = 1.0;

    _UpdateStateProvider(_VAR_SENSOR_DATA_);
}

void StateEstimatorVirtual::_GenerateObservationVector(_DEF_SENSOR_DATA_){
    // IMU data
    for(int i(0); i<3; ++i){
        obs_[i] = ang_vel[i];
        obs_[i+3] = accelerometer[i];
    }
    omega_<< ang_vel[0], ang_vel[1], ang_vel[2];
    acc_ << accelerometer[0], accelerometer[1], accelerometer[2];
    
    // MoCap Data
    Eigen::Matrix<double, 3*NUM_LED_OBS, 1> MoCap_Data;
    MoCap_receiver_->getMoCapData(MoCap_Data, LED_visibility_);
    for(int i(0); i< 3 * NUM_LED_REDUCE; ++i) obs_[i + 6] = MoCap_Data[i];    
}

void StateEstimatorVirtual::_GenerateObservationModel(){
    z_.setZero();
    H_.setZero();
    R_.setIdentity();

    // R_*=2.5;
    // R_/=SERVO_RATE;
    
    int idx_offset(0);
    sejong::Quaternion q_quat(x_pred_[6],x_pred_[3],x_pred_[4],x_pred_[5]);
    Eigen::Matrix<double, 3,3> C(q_quat);

    // Ang Velocity
    z_.head(3) = C.transpose() * x_pred_.segment(10,3);
    idx_offset += 3;
    
    // Acceleration
    Eigen::Matrix<double, 3,1> acc;
    sejong::Matrix Ainv;
    sejong::Vector grav;
    sejong::Vector coriolis;
    model_->getInverseMassInertia(Ainv);
    model_->getGravity(grav);
    model_->getCoriolis(coriolis);

    sejong::Matrix J_lin = J_.block<3, NUM_QDOT>(0,0);
    sejong::Matrix Lambda_inv = J_lin * Ainv * J_lin.transpose();
    sejong::Matrix Lambda = Lambda_inv.inverse();
    sejong::Matrix Jbar = Ainv * J_lin.transpose() * Lambda;
    sejong::Vector p = Jbar.transpose() * grav;
    sejong::Vector mu = Jbar.transpose() * coriolis;

    acc = Lambda_inv * (x_pred_.segment(13, 3) - p - mu);
    sejong::Vector vertical_acc(3);
    vertical_acc.setZero();
    vertical_acc[2] = 9.81;
    z_.segment(3,3) = C.transpose() * (acc + vertical_acc);
    // std::cout<<"acc:\n"<<acc<<std::endl;
    
    idx_offset += 3;
    
    // LED Position
    sejong::Vect3 pos;
    int led_id = LED_BODY_0;

    // LED Visibility Check
    for (int i(0); i<NUM_LED_REDUCE; ++i){
        if(!LED_visibility_[i]){
            R_.block<3,3>(idx_offset + 3*i, idx_offset + 3*i) *= 100000.0;
        }
    }
    
    for(int i(0); i<NUM_LED_REDUCE; ++i){
        model_->getPosition(virtual_configuration_, led_id, pos);
        z_.segment(idx_offset, 3) = pos;

        idx_offset +=3;
        ++led_id;
    }
    //************ Make 'H'  ****************
    // Ang Velocity
    sejong::Matrix w_mt(3,3);
    w_mt <<
        0.0, -x_pred_[7 + 5], x_pred_[7 + 4],
        x_pred_[7 + 5], 0.0, -x_pred_[7 + 3],
        -x_pred_[7 + 4], x_pred_[7 + 3], 0.0;

    H_.block<3,3>(0, 0) = C.transpose() * w_mt;
    H_.block<3,3>(0, 10) = C.transpose();

    // Acceleration
    Eigen::Matrix<double, 3,3> acc_mt;
    acc_mt<<
        0.0, -acc[2], acc[1],
        acc[2], 0.0, -acc[0],
        -acc[1], acc[0], 0.0;
    H_.block<3,3>(3, 3) = C.transpose() * acc_mt;
    H_.block<3, 3>(3, 12) = C.transpose() * Lambda_inv;
    
    // LED
    sejong::Matrix J_LED;
    for (int i(0); i<NUM_LED_REDUCE; ++i){
        model_->getFullJacobian(virtual_configuration_, LED_BODY_0 +i, J_LED);
        H_.block<3, 6>(3*i + 6, 0) = J_LED.block(3,0, 3, 6);
    }
}

void StateEstimatorVirtual::_GeneratePredictionModel(){
    // Position
    x_pred_.head(3) = x_est_.head(3) + x_est_.segment(7,3) * SERVO_RATE;
    
    // Orienation
    sejong::Quaternion dq;
    sejong::Vector theta(3);
    theta = x_est_.segment(10,3) * SERVO_RATE;
    sejong::convert(theta, dq);

    sejong::Quaternion q(x_est_[6],
                         x_est_[3],
                         x_est_[4],
                         x_est_[5]);

    sejong::Quaternion q_n = sejong::QuatMultiply(dq, q);

    x_pred_[3] = q_n.x();
    x_pred_[4] = q_n.y();
    x_pred_[5] = q_n.z();
    x_pred_[6] = q_n.w();

    
    // Velocity
    sejong::Matrix Ainv;
    sejong::Vector grav;
    sejong::Vector coriolis;
    model_->getInverseMassInertia(Ainv);
    model_->getGravity(grav);
    model_->getCoriolis(coriolis);

    Eigen::Matrix<double, 6, 1> acc;
    sejong::Matrix Lambda_inv = J_ * Ainv * J_.transpose();
    sejong::Matrix Lambda = Lambda_inv.inverse();
    sejong::Matrix Jbar = Ainv * J_.transpose() * Lambda;
    sejong::Vector p = Jbar.transpose() * grav;
    sejong::Vector mu = Jbar.transpose() * coriolis;

    sejong::Vector F_ext = x_est_.segment(13, 6);

    acc = Lambda_inv * (F_ext - p - mu);
    
    x_pred_.segment(7,6) = x_est_.segment(7,6) + acc * SERVO_RATE;

    
    // Force
    Eigen::Matrix<double, 6, 1> F_ext_pred;
    F_ext_pred = grav.head(6);
    
    x_pred_.tail(6) = F_ext_pred;

    
    F_.setIdentity();
    Qc_.setIdentity();
    Qc_.block<6,6>(12, 12) *= 100.0;
    // Position
    F_.block<3, 3>(0, 3) = sejong::Matrix::Identity(3, 3) * SERVO_RATE;

    // Orienation
    F_.block<3,3>(3, 6) = sejong::Matrix::Identity(3, 3) * SERVO_RATE;

    // Velocity
    F_.block<6,6>(6, 12) = Lambda_inv * SERVO_RATE;
}

void StateEstimatorVirtual::_IntegrateState(){
    // Test
    // err_.setZero();
    
    x_est_.head(3) = x_pred_.head(3) + err_.head(3);

    // Orientation Update
    sejong::Quaternion dq;
    sejong::Vector theta;
    theta = err_.segment(3, 3);
    sejong::convert(theta, dq);
    sejong::Quaternion q(x_pred_[6],
                         x_pred_[3],
                         x_pred_[4],
                         x_pred_[5]);

    sejong::Quaternion q_n = sejong::QuatMultiply(dq, q);

    x_est_[3] = q_n.x();
    x_est_[4] = q_n.y();
    x_est_[5] = q_n.z();
    x_est_[6] = sqrt(1 - q_n.x()* q_n.x() - q_n.y() * q_n.y() - q_n.z()*q_n.z());

    x_est_.segment(7,12) = x_pred_.segment(7,12) + err_.segment(6,12);
}

void StateEstimatorVirtual::_UpdateStateProvider(_DEF_SENSOR_DATA_){
    virtual_configuration_.head(6) = x_est_.head(6);
    virtual_configuration_[NUM_QDOT] = x_est_[6];

    // virtual_vel_.head(6) = x_est_.segment(7, 6);
    
    state_provider_->Q_ = virtual_configuration_;
    state_provider_->Qdot_ = virtual_vel_;
    // state_provider_->Q_[2] = virtual_configuration_[2];// + state_provider_->height_offset_;
    
    state_provider_->Q_est_.head(6) = x_est_.head(6);
    state_provider_->Q_est_[NUM_QDOT] = x_est_[6];
    state_provider_->Qdot_est_.head(6) = x_est_.segment(7,6);

    // state_provider_->Q_.segment(3,3) = x_est_.segment(6,3);
    // state_provider_->Q_[NUM_QDOT] = x_est_[9];

    // state_provider_->Qdot_.head(3) = x_est_.segment(3,3);
    // state_provider_->Qdot_.segment(3,3) = omega_;

    integrator_->J_cm_ = J_;
    integrator_->F_ext_ = x_est_.segment(13, 6);
    
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
