#include "Simple_Test.h"
#include <stdio.h>
#include <Configuration.h>
#include <math.h>

#include <utils/utilities.h>
#include <utils/DataManager.h>
#include <Filter/filters.h>

#ifdef __RTAI__
 #include <rtai_sem.h>
 #include <rtai_sched.h>
#endif

Simple_Test::Simple_Test():count_(0), running_time_(0.0){
    
}
Simple_Test::~Simple_Test(){}

void Simple_Test::TestCommand(_SIMPLE_TEST_VAR_DECR_){
    // command
    dac_desired.resize(NUM_ACT_JOINT, 0);
    command.resize(NUM_ACT_JOINT, 0.0);
    // Gains
    jpos_gain_kp.resize(NUM_ACT_JOINT, 0);
    jpos_gain_ki.resize(NUM_ACT_JOINT, 0);
    motor_gain_kv.resize(NUM_ACT_JOINT, 0);
    
    motor_gain_kv[0] = 11;
    motor_gain_kv[1] = 13;
    motor_gain_kv[2] = 8;
    motor_gain_kv[3] = 11;
    motor_gain_kv[4] = 13;
    motor_gain_kv[5] = -8;

    // Right
    jpos_gain_kp[0] = 2500;
    jpos_gain_ki[0] = 11;
    jpos_gain_kp[1] = 600;
    jpos_gain_ki[1] = 10;
    jpos_gain_kp[2] = 600;
    jpos_gain_ki[2] = 10;
    // Left
    jpos_gain_kp[3] = -1500;
    jpos_gain_ki[3] = -10;
    jpos_gain_kp[4] = -600;
    jpos_gain_ki[4] = -15;
    jpos_gain_kp[5] = -550;
    jpos_gain_ki[5] = -10;

    DoTest(jpos, torque, motor_theta, raw_dac_cmd,
           jpos_gain_kp, jpos_gain_ki, motor_gain_kv,
           command, dac_desired);

    ++count_;
#ifdef __RTAI__
    if(count_<PREPARING_COUNT){
        start_count_ = nano2count(rt_get_cpu_time_ns());
    }
    running_time_ = (1.0e-9) * (double)(count2nano(nano2count(rt_get_cpu_time_ns()) - start_count_));
#endif

}

Joint_Position_Test_Motor_Theta_Ctrl::Joint_Position_Test_Motor_Theta_Ctrl():Simple_Test(), save_tmp(15),joint_idx(2) , motor_kp(NUM_ACT_JOINT), motor_ki(NUM_ACT_JOINT), theta2motor_ratio_(NUM_ACT_JOINT), motor_set_pt_(NUM_ACT_JOINT), spring_constant_(NUM_ACT_JOINT){
    initial_count_ = 3;
    save_tmp.setZero();
    theta2motor_ratio_.setZero();
    
    char file_name[50];
    sprintf(file_name, "joint%i_pos_test_w_motor_pos_ctrl", joint_idx);
    DataManager::GetDataManager()->RegisterData(&save_tmp, SJ_VEC, file_name, save_tmp.rows());
    // Right
    motor_kp[0] = 1000;
    motor_ki[0] = 50;
    motor_kp[1] = 1500;
    motor_ki[1] = 100;
    motor_kp[2] = 3500;
    motor_ki[2] = 100;
    // Left
    motor_kp[3] = 2000;
    motor_ki[3] = 50;
    motor_kp[4] = 3500;
    motor_ki[4] = 100;
    motor_kp[5] = 4500;
    motor_ki[5] = 100;

    theta2motor_ratio_[0] =  167000.0;
    theta2motor_ratio_[1] =  -68103.0;
    theta2motor_ratio_[2] =  -66705.0;

    theta2motor_ratio_[3] = -169850.0;
    theta2motor_ratio_[4] = -69500.0;
    theta2motor_ratio_[5] = -66828.0;

    error_sum_ = 0.0;
// Additional mass aluminuim Shank    
    mass_ = 0.31;
    r_ = 0.165;
    double Inertia(0.00538);
    l_ = 0.35;
    add_mass_ = 4.840;
// carbon Shank
    // add_mass_ = 0.0;
    // l_ = 0.0;
    // r_ = 0.31;
    // mass_ = 0.201;
    // double Inertia(0.00338);
    
    Itot_ = Inertia + mass_ * r_ * r_ + add_mass_*l_*l_;
    g_ = 9.81;

    Kp_ = 350.0;
    Kd_ = 80.0;
    Ki_ = 50.0;

    spring_constant_[2] = 435.0;
    spring_constant_[5] = 455.0;
    
    printf("[Joint Position Test with Motor Position Ctrl] Construct\n");    
}
Joint_Position_Test_Motor_Theta_Ctrl::~Joint_Position_Test_Motor_Theta_Ctrl(){}

void Joint_Position_Test_Motor_Theta_Ctrl::DoTest(_SIMPLE_TEST_VAR_DECR_){
    // left abduction deactivated
    motor_gain_kv[3] = 0;

    // Initialization
    if(count_<initial_count_){
        for (int i(0); i<NUM_ACT_JOINT; ++i){
            motor_set_pt_[i] = motor_theta[i];
        }
        offset_ = jpos[joint_idx];
        q_ = jpos[joint_idx];
        qdot_ = 0.0;
    }
    
    for (int i(0); i<NUM_ACT_JOINT; ++i){
        jpos_gain_kp[i] = motor_kp[i];
        jpos_gain_ki[i] = motor_ki[i];
    }

    double amp(0.2);
    double omega(2*M_PI*2.0);

    // Desired Joint Position
    double des_jpos = 1.1 + amp * sin(omega * running_time_);
    double des_jvel = amp * omega * cos(omega * running_time_);
    double des_acc = -amp * omega * omega * sin( omega * running_time_);

    double error(des_jpos - q_);
    error_sum_ += (error * SERVO_RATE);
    double qddot = des_acc + Kp_*error + Kd_*(des_jvel - qdot_) + Ki_*error_sum_;

    double q1(jpos[joint_idx-1]);
    double q2(jpos[joint_idx]);

    double grav = mass_*g_*r_*cos(q1 + q2) + add_mass_ * g_ *l_*cos(q1 + q2);
    double tau = Itot_ * qddot + grav;    

    q_ = q_ + qdot_*SERVO_RATE;
    qdot_ = qdot_ + qddot * SERVO_RATE;

    double motor_pos_command = q_ + tau/spring_constant_[joint_idx];
    double des_motor_pos = theta2motor_ratio_[joint_idx] * (motor_pos_command - offset_) + motor_set_pt_[joint_idx];
    for( int i(0); i<NUM_ACT_JOINT; ++i){
        command[i] = motor_set_pt_[i];
    }
    command[joint_idx] = des_motor_pos;
    
    // Save Data
    save_tmp[0] = running_time_;
    save_tmp[1] = des_jpos;
    save_tmp[2] = jpos[joint_idx];
    save_tmp[3] = command[joint_idx];
    save_tmp[4] = motor_theta[joint_idx];
    save_tmp[5] = offset_;
    save_tmp[6] = motor_set_pt_[joint_idx];
    save_tmp[7] = raw_dac_cmd[joint_idx];
    save_tmp[8] = q_;
    save_tmp[9] = qdot_;
    save_tmp[10] = torque[joint_idx];
    save_tmp[11] = tau;
    save_tmp[12] = grav;
    save_tmp[13] = motor_pos_command;
}

//////////////////////////////////////////////////////////
//////////////   Motor Position Test /////////////////////
//////////////////////////////////////////////////////////

Motor_Position_Test::Motor_Position_Test():Simple_Test(), save_tmp(8),joint_idx(0) , motor_kp(NUM_ACT_JOINT), motor_ki(NUM_ACT_JOINT), theta2motor_ratio_(NUM_ACT_JOINT){
    initial_count_ = 3;
    save_tmp.setZero();
    theta2motor_ratio_.setZero();
    
    char file_name[50];
    sprintf(file_name, "motor%i_pos_test", joint_idx);
    DataManager::GetDataManager()->RegisterData(&save_tmp, SJ_VEC, file_name, save_tmp.rows());
    // Right
    motor_kp[0] = 2000;
    motor_ki[0] = 50;
    motor_kp[1] = 1500;
    motor_ki[1] = 100;
    motor_kp[2] = 1500;
    motor_ki[2] = 100;
    // Left
    motor_kp[3] = 2000;
    motor_ki[3] = 50;
    motor_kp[4] = 1500;
    motor_ki[4] = 100;
    motor_kp[5] = 4500;
    motor_ki[5] = 100;

    theta2motor_ratio_[0] =  120000.0;
    theta2motor_ratio_[1] =  -68103.0;
    theta2motor_ratio_[2] =  -66705.0;

    // theta2motor_ratio_[3] = -169850.0;
    theta2motor_ratio_[3] = -149850.0;

    theta2motor_ratio_[4] = -69500.0;
    theta2motor_ratio_[5] = -66828.0;
    
    printf("[Motor Position Test] Construct\n");    
}
Motor_Position_Test::~Motor_Position_Test(){}

void Motor_Position_Test::DoTest(_SIMPLE_TEST_VAR_DECR_){
    for (int i(0); i<NUM_ACT_JOINT; ++i){
        if(i != joint_idx){
            motor_gain_kv[i] = 0;
        }
    }

    if(count_<initial_count_){
        motor_set_pt_ = motor_theta[joint_idx];
        offset_ = jpos[joint_idx];
    }
    
    for (int i(0); i<NUM_ACT_JOINT; ++i){
        jpos_gain_kp[i] = motor_kp[i];
        jpos_gain_ki[i] = motor_ki[i];
    }

    double amp(0.1);
    double omega(2*M_PI * 2.7);

    // Desired Joint Position
    double des_jpos = offset_ + amp * sin(omega * running_time_);

    double des_motor_pos = theta2motor_ratio_[joint_idx] * (des_jpos - offset_) + motor_set_pt_;
    command[joint_idx] = des_motor_pos;
    
    // Save Data
    save_tmp[0] = running_time_;
    save_tmp[1] = des_jpos;
    save_tmp[2] = jpos[joint_idx];
    save_tmp[3] = command[joint_idx];
    save_tmp[4] = motor_theta[joint_idx];
    save_tmp[5] = offset_;
    save_tmp[6] = motor_set_pt_;
    save_tmp[7] = raw_dac_cmd[joint_idx];
}

//////////////////////////////////////////////////////////
//////////////   Joint Position Test /////////////////////
//////////////////////////////////////////////////////////

Joint_Position_Test::Joint_Position_Test():Simple_Test(), save_tmp(7),joint_idx(3), torque2DAC_ratio_(NUM_ACT_JOINT) {
    save_tmp.setZero();
    char file_name[50];
    sprintf(file_name, "joint%i_pos_test", joint_idx);
    DataManager::GetDataManager()->RegisterData(&save_tmp, SJ_VEC, file_name, save_tmp.rows());
    
    // RIGHT
    torque2DAC_ratio_[0] = 1.0/0.0973;
    torque2DAC_ratio_[1] = -1.0/0.0628;
    torque2DAC_ratio_[2] = -1.0/0.0695;
    // LEFT
    torque2DAC_ratio_[3] = -1.0/0.0977;
    torque2DAC_ratio_[4] = -1.0/0.0399;
    torque2DAC_ratio_[5] = 1.0/0.0674;

    jvel_filter_ = new deriv_lp_filter(2*M_PI*80.0, SERVO_RATE);
    jvel_filter_->clear();
    printf("[Joint Position Test] Construct\n");    
}
Joint_Position_Test::~Joint_Position_Test(){
    delete jvel_filter_;
}

void Joint_Position_Test::DoTest(_SIMPLE_TEST_VAR_DECR_){
    command[2] = 1.0;
    command[5] = 1.0;

    double offset(0.0);
    double amp(0.15);
    double omega(3.0);

    // Desired Joint Position
    double des_jpos = offset + amp * sin(omega * running_time_);
    double des_jvel = amp * omega * cos(omega * running_time_);

    double acc = -amp*omega*omega*sin(omega *running_time_);
    command[joint_idx] = des_jpos;
    
    //WBOSC
    jvel_filter_->input(jpos[joint_idx]);
    double inertia(0.026);
    double kp(150.0);
    double kv(20.0);
    double wbosc_input(acc +
                       kp*(des_jpos - jpos[joint_idx]) +
                       kv*(des_jvel - jvel_filter_->output()) );    
    // dac_desired[joint_idx] = torque2DAC_ratio_[joint_idx] *
    //     inertia * wbosc_input;
    dac_desired[joint_idx] = 0;

    
    // Save Data
    save_tmp[0] = running_time_;
    save_tmp[1] = command[joint_idx];
    save_tmp[2] = jpos[joint_idx];
    save_tmp[3] = dac_desired[joint_idx];
    save_tmp[4] = des_jvel;
    save_tmp[5] = jvel_filter_->output();
    save_tmp[6] = raw_dac_cmd[joint_idx];
}

DAC_Torque_Test::DAC_Torque_Test():Simple_Test(), save_tmp(4), joint_idx(0){
    save_tmp.setZero();
    char file_name[50];
    sprintf(file_name, "joint%i_dac_test", joint_idx);
    DataManager::GetDataManager()->RegisterData(&save_tmp, SJ_VEC, file_name, 4);
    
    printf("[DAC Torque Test] Construct\n");
}
DAC_Torque_Test::~DAC_Torque_Test(){}

void DAC_Torque_Test::DoTest(_SIMPLE_TEST_VAR_DECR_){
    command[2] = 1.0;
    command[5] = 1.0;

    signed int signal = 0;

    signed int initial = 200;
    signed int gap = 30;
    double stay_time = 2.0;
    static double exp_start_time(6.0);
    int step = 5;
    static int step_count(0);
    static bool going_up (true);
    static int trial(0);
    signal = initial + gap * step_count;

    if(running_time_ < exp_start_time){
        signal = initial;
    }
    else if(running_time_ - exp_start_time > stay_time){
        if(going_up){
            ++step_count;
            ++trial;
            exp_start_time = running_time_;
            printf("Next dac singal: %i, step count: %i / %i \n", initial+ gap * step_count, trial, 2*step + 2);
            if(step_count > step){
                going_up =  false;
            }
        }
        else{
            --step_count;
            ++trial;
            exp_start_time = running_time_;

            if (step_count < 0){
                exit(0);
            }
            printf("Next dac singal: %i, step count: %i / %i \n", initial+ gap * step_count, trial, 2*step+2);

        }
    }
    motor_gain_kv[joint_idx] = 0;
    dac_desired[joint_idx] = signal;
    
    save_tmp[0] = running_time_;
    save_tmp[1] = dac_desired[joint_idx];
    save_tmp[2] = torque[joint_idx];
    save_tmp[3] = raw_dac_cmd[joint_idx];
}
