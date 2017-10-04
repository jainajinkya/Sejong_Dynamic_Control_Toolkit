#include "StateProvider.h"
#include <utils/DataManager.h>

#define TASK_LEN 10

StateProvider* StateProvider::GetStateProvider(){
    static StateProvider state_provider_;
    return &state_provider_;
}

StateProvider::StateProvider(): change_imu_mode_(false),
                                task_start_(false),
                                hold_xy_(false),
                                initialized_(false),
                                local_ang_vel_(3),
                                left_LED_x_(5),
                                left_LED_y_(5),
                                left_LED_z_(5),
                                right_LED_x_(5),
                                right_LED_y_(5),
                                right_LED_z_(5),
                                Body_Euler_ori_(3), system_count_(0),
                                // Arbitrary Large number
                                task_des_(TASK_LEN),
                                task_curr_(TASK_LEN),
                                task_vel_des_(TASK_LEN),
                                task_vel_curr_(TASK_LEN),
                                kalman_filter_state_(20),
                                torque_input_(NUM_ACT_JOINT),
                                Q_sim_(NUM_Q),
                                Qdot_sim_(NUM_QDOT),
                                Qddot_sim_(NUM_QDOT),
                                Q_(NUM_Q),
                                Qdot_(NUM_QDOT),
                                Q_est_(NUM_Q),
                                Qdot_est_(NUM_QDOT),
                                Foot_Contact_(2),
                                height_offset_(0.0),
                                contact_constraint_(false)
                                
{
    task_des_.setZero();
    task_curr_.setZero();
    task_vel_des_.setZero();
    task_vel_curr_.setZero();
    
    led_cond_.resize(10, false);
    attraction_loc_ = sejong::Vector::Zero(2);
    DataManager::GetDataManager()->RegisterData(&Q_, SJ_VEC, "config", NUM_Q);
    DataManager::GetDataManager()->RegisterData(&Qdot_, SJ_VEC, "qdot", NUM_QDOT);
    DataManager::GetDataManager()->RegisterData(&Q_est_, SJ_VEC, "config_est", NUM_Q);
    DataManager::GetDataManager()->RegisterData(&Qdot_est_, SJ_VEC, "qdot_est", NUM_QDOT);
    DataManager::GetDataManager()->RegisterData(&Foot_Contact_, SJ_VEC, "foot_contact", 2);
    DataManager::GetDataManager()->RegisterData(&curr_time_, DOUBLE, "time");

    DataManager::GetDataManager()->RegisterData(&task_des_, SJ_VEC, "task_des", TASK_LEN);
    DataManager::GetDataManager()->RegisterData(&task_curr_, SJ_VEC, "task_act", TASK_LEN);
    DataManager::GetDataManager()->RegisterData(&task_vel_des_, SJ_VEC, "task_vel_des", TASK_LEN);
    DataManager::GetDataManager()->RegisterData(&task_vel_curr_, SJ_VEC, "task_vel_act", TASK_LEN);

}

void StateProvider::SaveTaskData(const Vector & act, const Vector & des,
                                 const Vector & vel_act, const Vector & vel_des){
    for(int i(0); i<act.rows(); ++i){
        task_des_[i] = des[i];
        task_curr_[i] = act[i];
    }
    for(int i(0); i<vel_act.rows(); ++i){
        task_vel_des_[i]  = vel_des[i];
        task_vel_curr_[i] = vel_act[i];
    }
}

const Vect3 & StateProvider::getFootPos(SJLinkID id){
    if(id == RFOOT) return rfoot_pos_;
    if(id == LFOOT) return lfoot_pos_;
}

const Vect3 & StateProvider::getFootVel(SJLinkID id){
    if(id == RFOOT) return rfoot_vel_;
    if(id == LFOOT) return lfoot_vel_;
}
Vect4 StateProvider::getBodyOri(){
    Vect4 ori;
    ori[0] = Q_[NUM_QDOT];
    ori[1] = Q_[3];
    ori[2] = Q_[4];
    ori[3] = Q_[5];

    return ori;
}

sejong::Quaternion StateProvider::getBodyOriQuat(){
    Quaternion ori;
    ori.w() = Q_[NUM_QDOT];
    ori.x() = Q_[3];
    ori.y() = Q_[4];
    ori.z() = Q_[5];

    return ori;
}
