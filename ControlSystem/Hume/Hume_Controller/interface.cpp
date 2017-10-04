#include "interface.h"
#include <stdio.h>

#include <math.h>
#include <utils/wrap_eigen.hpp>
#include <utils/utilities.h>
#include "Simple_Test.h"
#include "HumeSystem.h"
#include "Controller_Hume.h"
#include <WBOSC/Constraint.hpp>

#include "Integrator.h"

#include <Plotting/Hume_Plotting.h>
#include <utils/comm_udp.h>
#include <utils/DataManager.h>

#include "StateProvider.h"

#ifdef __RTAI__
 #include <rtai_sem.h>
 #include <rtai_sched.h>
#else
 #include <chrono>
#endif

// #define SIMPLE_TEST_EXPERIMENT

// std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
// std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl;
// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() <<std::endl;

Hume_Interface::Hume_Interface():
    socket_(0), count_(0), running_time_(0.0),
    torque_command_(NUM_ACT_JOINT), torque2DAC_ratio_(NUM_ACT_JOINT), jvel_save_(NUM_ACT_JOINT),
    motor_theta_(NUM_ACT_JOINT), motor_theta_cmd_(NUM_ACT_JOINT),
    raw_dac_cmd_save_(NUM_ACT_JOINT), raw_dac_des_save_(NUM_ACT_JOINT),
    theta2motor_ratio_(NUM_ACT_JOINT), initial_jpos_(NUM_ACT_JOINT),
    spring_constant_(NUM_ACT_JOINT),
    motor_set_pt_(NUM_ACT_JOINT),
    ang_vel_save_(3)
{
    // Joint to Motor theta ratio
    theta2motor_ratio_[0] = 100000.0;
    theta2motor_ratio_[1] = -68103.0;
    theta2motor_ratio_[2] = -66705.0;
    theta2motor_ratio_[3] = -140000.0;
    theta2motor_ratio_[4] = -69500.0;
    theta2motor_ratio_[5] = -66828.0;

    // Spring Constant
    spring_constant_[0] = 440.0;
    spring_constant_[1] = 360.0;
    spring_constant_[2] = 415.0;
    spring_constant_[3] = 440.0;
    spring_constant_[4] = 450.0;
    spring_constant_[5] = 455.0;

    
    torque2DAC_ratio_.setZero();
    // RIGHT
    torque2DAC_ratio_[0] = 1.0/0.0973;
    torque2DAC_ratio_[1] = -1.0/0.0628 * 1.0;
    torque2DAC_ratio_[2] = -1.0/0.0695 * 1.0;
    // LEFT
    torque2DAC_ratio_[3] = -1.0/0.0977;
    torque2DAC_ratio_[4] = -1.0/0.0399 * 1.0;
    torque2DAC_ratio_[5] = 1.0/0.0674 * 8.0;

    
#ifdef SIMPLE_TEST_EXPERIMENT
    // test_ = new DAC_Torque_Test();
    // test_ = new Joint_Position_Test();
    test_ = new Motor_Position_Test();
    // test_ = new Joint_Position_Test_Motor_Theta_Ctrl();
#else
    hume_sys_ = new HUME_System();

    DataManager::GetDataManager()->RegisterData(&torque_command_, SJ_VEC, "torque_command", NUM_ACT_JOINT);
    DataManager::GetDataManager()->RegisterData(&raw_dac_cmd_save_, SJ_VEC, "raw_dac_cmd", NUM_ACT_JOINT);
    DataManager::GetDataManager()->RegisterData(&raw_dac_des_save_, SJ_VEC, "raw_dac_des", NUM_ACT_JOINT);
    DataManager::GetDataManager()->RegisterData(&motor_theta_, SJ_VEC, "motor_theta", NUM_ACT_JOINT);
    DataManager::GetDataManager()->RegisterData(&motor_theta_cmd_, SJ_VEC, "motor_theta_command", NUM_ACT_JOINT);
    DataManager::GetDataManager()->RegisterData(&ang_vel_save_, SJ_VEC, "imu_ang_vel", 3);
#endif


#ifndef __RTAI__    
    DataManager::GetDataManager()->RegisterData(&quat_sensor_, QUATERNION, "quat_sense", 4);
    DataManager::GetDataManager()->RegisterData(&jvel_save_, SJ_VEC, "jvel", NUM_ACT_JOINT);
#endif
    printf("[Hume Interface] Contruct\n");
}

Hume_Interface::~Hume_Interface(){
}

void Hume_Interface::GetCommand(const std::vector <double> & jpos,
                                const std::vector<double> & jvel, 
                                const std::vector<double> & torque,
                                const std::vector<double> & motor_theta,
                                const std::vector<signed int> & raw_dac_cmd,
                                double * ori_mtx,
                                const std::vector<double> & ang_vel, 
                                const std::vector<double> & accelerometer, 
                                const std::vector<double> & magnetometer, 
                                bool _left_foot_contact, 
                                bool _right_foot_contact, 
                                std::vector<signed int> & gain_kp, 
                                std::vector<signed int> & gain_ki,
                                std::vector<signed int> & motor_gain_kv,
                                std::vector<double> & command, // Joint Position or Torque 
                                std::vector<signed int> & dac_desired){
    
#ifdef SIMPLE_TEST_EXPERIMENT
    test_->TestCommand( jpos,
                        torque,
                        motor_theta,
                        raw_dac_cmd,
                        gain_kp, 
                        gain_ki,
                        motor_gain_kv,
                        command, 
                        dac_desired);
    DataManager::GetDataManager()->start();
    return ;
#else
    if(!_Initialization(_VAR_SENSOR_DATA_)){

        state_estimator_.UpdateState(_VAR_SENSOR_DATA_);

        hume_sys_->getTorqueInput( gain_kp,
                                   gain_ki,
                                   motor_gain_kv,
                                   torque_command_);

        state_estimator_.Prediction(torque_command_,
                                     // hume_sys_->controller_->constraint_->Nc_,
                                     hume_sys_->controller_->constraint_->Jc_);
    }
#endif
    
#ifdef __RTAI__
    if(count_<PREPARING_COUNT){
        start_count_ = nano2count(rt_get_cpu_time_ns());
    }

    running_time_ = (1.0e-9) * (double)(count2nano( nano2count(rt_get_cpu_time_ns() ) - start_count_));

    _PhysicalRobotCommand_Generation(StateProvider::GetStateProvider()->Q_,
                                     torque_command_, command, dac_desired);

    for(int i(0); i<NUM_ACT_JOINT; ++i){
        raw_dac_des_save_[i] = dac_desired[i];
        raw_dac_cmd_save_[i] = raw_dac_cmd[i];
        motor_theta_[i] = motor_theta[i];
        motor_theta_cmd_[i] = command[i];
    }
#else
    Eigen::Matrix3d ori_SO3;
    ori_SO3 <<
        ori_mtx[0], ori_mtx[1], ori_mtx[2],
        ori_mtx[3], ori_mtx[4], ori_mtx[5],
        ori_mtx[6], ori_mtx[7], ori_mtx[8];
    sejong::Quaternion q(ori_SO3);
    quat_sensor_ = q;
    
    for (int i(0); i < NUM_ACT_JOINT; ++i){
        jvel_save_[i] = jvel[i];
    }

    running_time_ = (double)(count_) * SERVO_RATE;
    _SRLibCommand_Generation(StateProvider::GetStateProvider()->Q_,
                             torque_command_, command, dac_desired);
#endif
    if(count_%30 == 3){
        _SendData();
    }
    ++count_;
    for(int i(0); i<3; ++i){
        ang_vel_save_[i] = ang_vel[i];
    }
    StateProvider::GetStateProvider()->curr_time_ = running_time_;
    StateProvider::GetStateProvider()->system_count_ = count_;
}

bool Hume_Interface::_Initialization(_DEF_SENSOR_DATA_){
    if(count_ < 3){
        state_estimator_.Initialize_Estimator(_VAR_SENSOR_DATA_);
        torque_command_.setZero();

        for (int i(0); i<NUM_ACT_JOINT; ++i){
            motor_set_pt_[i] = motor_theta[i];
            initial_jpos_[i] = jpos[i];
        }
        return true;
    }
    StateProvider::GetStateProvider()->initialized_ = true;
    DataManager::GetDataManager()->start();
    return false;
}

void Hume_Interface::_PhysicalRobotCommand_Generation(const sejong::Vector & Q,
                                                      const sejong::Vector & torque_command,
                                                      std::vector<double> & des_pos,
                                                      std::vector<int> & dac_desired){
    des_pos.resize(NUM_ACT_JOINT);
    dac_desired.resize(NUM_ACT_JOINT, 0.0);

    for (int i(0); i<NUM_ACT_JOINT; ++i){
        des_pos[i] = theta2motor_ratio_[i] * (Q[i + NUM_VIRTUAL] + (torque_command[i] / spring_constant_[i]) - initial_jpos_[i]) + motor_set_pt_[i] ;
        dac_desired[i] = 0.0;
    }
}
void Hume_Interface::_PhysicalRobotCommand_Generation_Torque(const sejong::Vector & torque_command,
                                                      std::vector<double> & des_torque,
                                                      std::vector<int> & dac_desired){
    des_torque.resize(NUM_ACT_JOINT);
    dac_desired.resize(NUM_ACT_JOINT, 0.0);

    for (int i(0); i<NUM_ACT_JOINT; ++i){
        des_torque[i] = torque_command[i];
        dac_desired[i] = (int)(torque2DAC_ratio_[i] * torque_command[i]);
    }
}

void Hume_Interface::_SRLibCommand_Generation(const sejong::Vector & Q,
                                              const sejong::Vector & torque_command,
                                              std::vector<double> & des_jpos,
                                              std::vector<int> & dac_desired){
    des_jpos.resize(NUM_ACT_JOINT);
    dac_desired.resize(NUM_ACT_JOINT);
    
    for (int i(0); i < NUM_ACT_JOINT; ++i){
        des_jpos[i] = Q[i + NUM_VIRTUAL];
        dac_desired[i] = (int)(torque_command[i] * FLOATING_PT);
    }
}

void Hume_Interface::_SendData(){
    PLT_PRTC::Plot_data data;
    
    for (int i(0); i<NUM_Q; ++i){
        data.tot_conf[i] = StateProvider::GetStateProvider()->Q_[i];
        data.tot_conf[i + NUM_Q] = StateProvider::GetStateProvider()->Q_sim_[i];
    }
    data.time = running_time_;
    
    COMM::send_data(socket_, PORT_PLOT, &data, sizeof(PLT_PRTC::Plot_data), IP_ADDR_MYSELF);
}
