#include "StateEstimatorFull.h"
#include <utils/DataManager.h>
#include "Hume_Model/Hume_Model.h"
#include "StateProvider.h"

StateEstimatorFull::StateEstimatorFull()
    :StateEstimator<NUM_OBS_FULL, NUM_STATE_FULL, NUM_STATE_DOT_FULL, NUM_NOISE_FULL>(),
    torque_est_save_(NUM_ACT_JOINT),
    pre_lcontact_(false),
    pre_rcontact_(false)
{

    torque_est_save_.setZero();

    DataManager::GetDataManager()->RegisterData(&torque_est_save_, SJ_VEC, "torque_est", NUM_ACT_JOINT);

    // L_.setZero();
    // L_.block<NUM_QDOT, NUM_QDOT>(0, NUM_QDOT).setIdentity();
    L_.setIdentity();
    
    MoCap_receiver_ = new MoCapReceiver();
    MoCap_receiver_->start();
}
StateEstimatorFull::~StateEstimatorFull(){
    delete MoCap_receiver_;
}


void StateEstimatorFull::_GenerateObservationVector(_DEF_SENSOR_DATA_){
    int i(0);
    int idx_offset(0);

    // Joint Position
    for(i = 0; i< NUM_ACT_JOINT; ++i)  obs_[idx_offset + i] = jpos[i];
    idx_offset += i;
    // Torque
    for(i = 0; i< NUM_ACT_JOINT; ++i)  obs_[idx_offset + i] = torque[i];
    idx_offset += i;
    // Angular Velocity
    for(i = 0; i< 3; ++i)  obs_[idx_offset + i] = ang_vel[i];
    idx_offset += i;
    // Acceleration of IMU
    for(i = 0; i<3; ++i) obs_[idx_offset + i] = accelerometer[i];
    idx_offset += i;
    // MoCap Data
    Eigen::Matrix<double, 3*NUM_LED_OBS, 1> MoCap_Data;
    MoCap_receiver_->getMoCapData(MoCap_Data, LED_visibility_);
    for(i = 0; i< 3 * NUM_LED_OBS; ++i) obs_[idx_offset + i] = MoCap_Data[i];

    impulse_ = false;
    if(!pre_lcontact_ && _left_foot_contact){
        impulse_ = true;
    }
    if(!pre_rcontact_ && _right_foot_contact){
        impulse_ = true;
    }
    
}

void StateEstimatorFull::_GenerateObservationModel(){
    z_.setZero();
    H_.setZero();
    R_.setIdentity();
    R_*=0.05;
    // R_.block<NUM_ACT_JOINT,NUM_ACT_JOINT>(0,0) *= 0.1;
    // Torque
    R_.block<NUM_ACT_JOINT,NUM_ACT_JOINT>(NUM_ACT_JOINT, NUM_ACT_JOINT) *= 1.5;
    // IMU
    if(impulse_){
        R_.block<6,6>(2*NUM_ACT_JOINT,2*NUM_ACT_JOINT) *= 1000.5;
        R_.block<NUM_ACT_JOINT,NUM_ACT_JOINT>(NUM_ACT_JOINT, NUM_ACT_JOINT) *= 5.5;
    }
    else{
        R_.block<6,6>(2*NUM_ACT_JOINT,2*NUM_ACT_JOINT) *= 1.5;
        R_.block<NUM_ACT_JOINT,NUM_ACT_JOINT>(NUM_ACT_JOINT, NUM_ACT_JOINT) *= 1.5;

    }


    // LED Visibility Check
    for (int i(0); i<NUM_LED_OBS; ++i){
        if(!LED_visibility_[i]){
            R_.block<3,3>(2*NUM_ACT_JOINT + 6 + 3*i, 2*NUM_ACT_JOINT + 6 + 3*i) *= 100000.0;
        }
    }
    
    // R.block<6,6>(2*NUM_ACT_JOINT, 2*NUM_ACT_JOINT) *= SERVO_RATE;
    // LED 
    // R.block<3*NUM_LED_OBS, 3*NUM_LED_OBS>(2*NUM_ACT_JOINT + 6, 2*NUM_ACT_JOINT + 6) *= SERVO_RATE;
    R_/=SERVO_RATE;
    int idx_offset(0);
    // Joint Position
    z_.head(NUM_ACT_JOINT) = x_pred_.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
    idx_offset += NUM_ACT_JOINT;
    // Joint Torque
    z_.segment(idx_offset, NUM_ACT_JOINT) = x_pred_.segment(NUM_Q + NUM_QDOT, NUM_ACT_JOINT);
    idx_offset += NUM_ACT_JOINT;
    // Angular Velocity
    sejong::Quaternion q_body (x_pred_[NUM_QDOT],
                               x_pred_[3],
                               x_pred_[4],
                               x_pred_[5]);
    Eigen::Matrix3d Ori_imu(q_body);

    z_.segment(idx_offset, 3) = Ori_imu.transpose() * x_pred_.segment(NUM_Q+3, 3);
    idx_offset += 3;
    
    // Acceleration
    sejong::Matrix Ainv;
    sejong::Vector grav;
    sejong::Vector coriolis;
    model_->getInverseMassInertia(Ainv);
    model_->getGravity(grav);
    model_->getCoriolis(coriolis);

    sejong::Vector Qddot = Ainv * Nc_.transpose() * (U_.transpose() * x_pred_.segment(NUM_Q + NUM_QDOT, NUM_ACT_JOINT) - coriolis - grav);
    sejong::Matrix Jimu;
    model_->getFullJacobian(virtual_configuration_, IMU, Jimu);

    sejong::Vect3 vertical;
    vertical.setZero();
    vertical[2] = 9.81;
    sejong::Vect3 Acc = Jimu.block(3,0,3,NUM_QDOT) * Qddot + vertical;;
    z_.segment(idx_offset, 3) = Ori_imu.transpose() * Acc;
    idx_offset += 3;

    // LED Position
    sejong::Vect3 pos;
    int led_id = LED_BODY_0;
    for(int i(0); i<NUM_LED_OBS; ++i){
        model_->getPosition(virtual_configuration_, led_id, pos);
        z_.segment(idx_offset, 3) = pos;

        idx_offset +=3;
        ++led_id;
    }

    //************ Make 'H'  ****************
    // Joint Position
    H_.block<NUM_ACT_JOINT, NUM_ACT_JOINT>(0, NUM_VIRTUAL).setIdentity();
    // Joint Torque
    H_.block<NUM_ACT_JOINT, NUM_ACT_JOINT>(NUM_ACT_JOINT, 2*NUM_QDOT).setIdentity();
    // Angular Velocity
    sejong::Matrix w_mt(3,3);
    w_mt <<
        0.0, -x_pred_[NUM_Q + 5], x_pred_[NUM_Q + 4],
        x_pred_[NUM_Q + 5], 0.0, -x_pred_[NUM_Q + 3],
        -x_pred_[NUM_Q + 4], x_pred_[NUM_Q + 3], 0.0;
    
    H_.block<3,3>(2*NUM_ACT_JOINT, 3) = Ori_imu.transpose() * w_mt;
    H_.block<3,3>(2*NUM_ACT_JOINT, 3 + NUM_QDOT) = Ori_imu.transpose();

    //Acceleration
    Eigen::Matrix<double, 3,3> acc_mt;
    acc_mt<<
        0.0, -Acc[2], Acc[1],
        Acc[2], 0.0, -Acc[0],
        -Acc[1], Acc[0], 0.0;
    H_.block<3,3>(2*NUM_ACT_JOINT + 3, 3) = Ori_imu.transpose() * acc_mt;
    H_.block<3, NUM_ACT_JOINT>(2*NUM_ACT_JOINT + 3, 2*NUM_QDOT) = Ori_imu.transpose() * Jimu.block(3,0,3, NUM_QDOT) * Ainv * Nc_.transpose() * U_.transpose();
    // LED
    sejong::Matrix J_LED;
    for (int i(0); i<NUM_LED_OBS; ++i){
        model_->getFullJacobian(virtual_configuration_, LED_BODY_0 +i, J_LED);
        H_.block<3, NUM_QDOT>(2*NUM_ACT_JOINT + 6 + 3*i, 0) =
            J_LED.block(3,0, 3, NUM_QDOT);
    }
}

void StateEstimatorFull::_GeneratePredictionModel(){
    Eigen::Matrix<double, NUM_Q, 1> config(x_est_.head(NUM_Q));
    Eigen::Matrix<double, NUM_QDOT, 1> vel(x_est_.segment(NUM_Q, NUM_QDOT));

    integrator_->ForwardDynamics(config, vel, torque_, Jc_);

    x_pred_.head(NUM_Q) = config;
    x_pred_.segment(NUM_Q, NUM_QDOT) = vel;
    x_pred_.segment(NUM_Q + NUM_QDOT, NUM_ACT_JOINT) = torque_;
    
    F_.setZero();
    Qc_.setIdentity();

    Qc_.block<NUM_ACT_JOINT, NUM_ACT_JOINT>(NUM_VIRTUAL, NUM_VIRTUAL) *= 0.1;
    Qc_.block<NUM_QDOT, NUM_QDOT>(NUM_QDOT, NUM_QDOT) *= 0.5;
    
    // Normal Euler Integration
    F_.block<2*NUM_QDOT,2*NUM_QDOT>(0,0).setIdentity();
    F_.block<NUM_QDOT, NUM_QDOT>(0, NUM_QDOT) =
        sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) * SERVO_RATE;

    sejong::Matrix Ainv;
    sejong::Vector grav;
    sejong::Vector coriolis;
    model_->getInverseMassInertia(Ainv);
    model_->getGravity(grav);
    model_->getCoriolis(coriolis);

    F_.block(NUM_QDOT, NUM_QDOT, NUM_QDOT, NUM_ACT_JOINT) =
        Ainv * Nc_.transpose() * U_.transpose() * SERVO_RATE;

    F_.block(NUM_QDOT, NUM_QDOT + NUM_ACT_JOINT, NUM_QDOT, 1) =
        - Nc_.transpose() * (coriolis + grav) * SERVO_RATE;

    // Quaternion
    Eigen::Matrix<double, 3,3> w_mt;
    w_mt.setZero();
    w_mt <<
        0.0, -x_est_[NUM_Q + 5], x_est_[NUM_Q + 4],
        x_est_[NUM_Q + 5], 0.0, -x_est_[NUM_Q + 3],
        -x_est_[NUM_Q + 4], x_est_[NUM_Q + 3], 0.0;
    F_.block(3,3, 3,3) += (-w_mt *SERVO_RATE);

    // L
    L_.block<NUM_QDOT, NUM_ACT_JOINT>(NUM_QDOT, 2*NUM_QDOT) =
        Ainv * Nc_.transpose() * U_.transpose();
}
void StateEstimatorFull::_IntegrateState(){

    x_est_.head(3) = x_pred_.head(3) + err_.head(3);
    // Orientation Update
    sejong::Quaternion dq;
    sejong::Vector theta;
    theta = err_.segment(3, 3);
    sejong::convert(theta, dq);
    sejong::Quaternion q(x_pred_[NUM_QDOT],
                         x_pred_[3],
                         x_pred_[4],
                         x_pred_[5]);

    sejong::Quaternion q_n = sejong::QuatMultiply(dq, q);

    x_est_[3] = q_n.x();
    x_est_[4] = q_n.y();
    x_est_[5] = q_n.z();
    x_est_[NUM_QDOT] = sqrt(1 - q_n.x()* q_n.x() - q_n.y() * q_n.y() - q_n.z()*q_n.z());

    // Joint Position
    x_est_.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = x_pred_.segment(NUM_VIRTUAL, NUM_ACT_JOINT) + err_.segment(NUM_VIRTUAL, NUM_ACT_JOINT);

    // State Velocity, Torque
    x_est_.segment(NUM_Q, NUM_QDOT) = x_pred_.segment(NUM_Q, NUM_QDOT) + err_.segment(NUM_QDOT, NUM_QDOT);
    x_est_.segment(NUM_Q + NUM_QDOT, NUM_ACT_JOINT) = x_pred_.segment(NUM_Q + NUM_QDOT, NUM_ACT_JOINT) + err_.segment(2*NUM_QDOT, NUM_ACT_JOINT);

    torque_est_save_ = x_est_.segment(NUM_Q + NUM_QDOT, NUM_ACT_JOINT);

}

void StateEstimatorFull::_UpdateStateProvider(_DEF_SENSOR_DATA_){
    // virtual_configuration_.head(6) = x_est_.head(6);
    // virtual_configuration_[NUM_QDOT] = x_est_[NUM_QDOT];
    // virtual_vel_.head(6) = x_est_.segment(NUM_Q, 6);
    virtual_configuration_ = x_est_.head(NUM_Q);
    virtual_vel_ = x_est_.segment(NUM_Q, NUM_QDOT);
    
    state_provider_->Q_ = virtual_configuration_;
    state_provider_->Qdot_ = virtual_vel_;

    state_provider_->Q_[2] = virtual_configuration_[2];// + state_provider_->height_offset_;
    
    state_provider_->Q_est_ = x_est_.head(NUM_Q);
    state_provider_->Qdot_est_ = x_est_.segment(NUM_Q, NUM_QDOT);

    model_->UpdateModel(state_provider_->Q_,
                        state_provider_->Qdot_);

    //Contact
    pre_rcontact_ = _right_foot_contact;
    pre_lcontact_ = _left_foot_contact;
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

void StateEstimatorFull::Initialize_Estimator(_DEF_SENSOR_DATA_){
    x_est_.setZero();
    x_pred_.setZero();

    virtual_configuration_.setZero();
    virtual_vel_.setZero();

    for(int i(0); i < NUM_ACT_JOINT; ++i){
        x_est_[i + 6] = jpos[i];
        x_pred_[i + 6] = jpos[i];

        virtual_configuration_[i + 6] = jpos[i];
    }
    Eigen::Matrix<double, 3*NUM_LED_OBS, 1> MoCap_Data;

    x_est_[NUM_QDOT] = 1.0;
    x_pred_[NUM_QDOT] = 1.0;

    virtual_configuration_[NUM_QDOT] = 1.0;

    _UpdateStateProvider(_VAR_SENSOR_DATA_);

    sejong::Matrix Ainv;
    model_->getInverseMassInertia(Ainv);
    Eigen::Matrix<double, NUM_VIRTUAL, NUM_QDOT> Jc;
    Jc.setZero();
    Jc.block<NUM_VIRTUAL, NUM_VIRTUAL>(0, 0).setIdentity();
    Eigen::Matrix<double,NUM_VIRTUAL, NUM_VIRTUAL> L(Jc * Ainv * Jc.transpose());
    Nc_ = Eigen::Matrix<double,NUM_QDOT, NUM_QDOT>::Identity() - Ainv * Jc.transpose() * L.inverse() * Jc;
}
