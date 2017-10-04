#ifndef HUME_INTERFACE
#define HUME_INTERFACE

#include <vector>
#include <utils/wrap_eigen.hpp>
#include <Configuration.h>

// #include "StateEstimator.h"
// #include "StateEstimatorKin.h"
#include "StateEstimatorReduced.h"
#include "StateEstimatorFull.h"
#include "StateEstimatorVirtual.h"

class Simple_Test;
class HUME_System;

class Hume_Interface{
public:
    Hume_Interface();
    ~Hume_Interface();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // command [0 ~ 5]: Torque command
    // command [6 ~ 11]: Jpos command
    void GetCommand(const std::vector <double> & jpos,
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
                    std::vector<signed int> & jpos_gain_kp, 
                    std::vector<signed int> & jpos_gain_ki,
                    std::vector<signed int> & motor_gain_kv,
                    std::vector<double> & command, 
                    std::vector<signed int> & dac_desired);


    HUME_System* GetHumeSystem(){ return hume_sys_; }
    
private:
    void _PhysicalRobotCommand_Generation(const sejong::Vector & Q,
                                          const sejong::Vector & torque_command,
                                          std::vector<double> & des_pos,
                                          std::vector<int> & dac_desired);

    void _PhysicalRobotCommand_Generation_Torque(const sejong::Vector & torque_command,
                                                 std::vector<double> & des_jpos,
                                                 std::vector<int> & dac_desired);
    
    void _SRLibCommand_Generation(const sejong::Vector & Q,
                                  const sejong::Vector & torque_command,
                                  std::vector<double> & des_jpos,
                                  std::vector<int> & dac_desired);

    bool _Initialization(_DEF_SENSOR_DATA_);
    
    void _SendData();

    int socket_;
    sejong::Vector jvel_save_;
    sejong::Vector ang_vel_save_;
    
    sejong::Vector torque2DAC_ratio_;
    sejong::Vector torque_command_;
    sejong::Quaternion quat_sensor_;

    int count_;
    double running_time_;

    // StateEstimatorFull state_estimator_;
    StateEstimatorReduced state_estimator_;
    // StateEstimatorVirtual state_estimator_;
    
    Simple_Test* test_;
    HUME_System* hume_sys_;

    long long start_count_;
    long long rt_count_;

    sejong::Vector raw_dac_cmd_save_;
    sejong::Vector raw_dac_des_save_;
    sejong::Vector motor_theta_;
    sejong::Vector motor_theta_cmd_;

    sejong::Vector theta2motor_ratio_;
    sejong::Vector initial_jpos_;
    sejong::Vector motor_set_pt_;
    sejong::Vector spring_constant_;
};


#endif
