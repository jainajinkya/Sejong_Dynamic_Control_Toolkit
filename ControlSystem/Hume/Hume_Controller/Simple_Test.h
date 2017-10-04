#ifndef SIMPLE_TEST
#define SIMPLE_TEST

#include <vector>
#include <utils/wrap_eigen.hpp>


#define _SIMPLE_TEST_VAR_DECR_ const std::vector <double> & jpos, const std::vector<double> & torque, const std::vector<double> motor_theta, const std::vector<signed int> & raw_dac_cmd, std::vector<signed int> & jpos_gain_kp, std::vector<signed int> & jpos_gain_ki, std::vector<signed int> & motor_gain_kv, std::vector<double> & command, std::vector<signed int> & dac_desired

class Simple_Test{
public:
    Simple_Test();
    virtual ~Simple_Test();
    
    void TestCommand(_SIMPLE_TEST_VAR_DECR_);

protected:
    virtual void DoTest(_SIMPLE_TEST_VAR_DECR_) = 0;
    
    int count_;
    long long start_count_;
    double running_time_;
};

class filter;

class Joint_Position_Test: public Simple_Test{
public:
    Joint_Position_Test();
    virtual ~Joint_Position_Test();
    
    virtual void DoTest(_SIMPLE_TEST_VAR_DECR_);
private:
    filter* jvel_filter_;
    sejong::Vector torque2DAC_ratio_;
    sejong::Vector save_tmp;
    int joint_idx;
};

class Joint_Position_Test_Motor_Theta_Ctrl: public Simple_Test{
    public:
    Joint_Position_Test_Motor_Theta_Ctrl();
    virtual ~Joint_Position_Test_Motor_Theta_Ctrl();
    
    virtual void DoTest(_SIMPLE_TEST_VAR_DECR_);
private:
    int initial_count_;
    sejong::Vector motor_set_pt_;
    double offset_;
    
    sejong::Vector theta2motor_ratio_;
    sejong::Vector motor_kp;
    sejong::Vector motor_ki;
    sejong::Vector save_tmp;
    int joint_idx;

    double q_;
    double qdot_;
    double Itot_;
    double g_;
    double mass_;
    double error_sum_;
    double r_;
    double l_;
    double add_mass_;
    sejong::Vector spring_constant_;

    double Kp_;
    double Kd_;
    double Ki_;
};

class Motor_Position_Test: public Simple_Test{
public:
    Motor_Position_Test();
    virtual ~Motor_Position_Test();
    
    virtual void DoTest(_SIMPLE_TEST_VAR_DECR_);
private:
    int initial_count_;
    int motor_set_pt_;
    double offset_;
    
    sejong::Vector theta2motor_ratio_;
    sejong::Vector motor_kp;
    sejong::Vector motor_ki;
    sejong::Vector save_tmp;
    int joint_idx;
};

class DAC_Torque_Test: public Simple_Test{
public:
    DAC_Torque_Test();
    virtual ~DAC_Torque_Test();

    virtual void DoTest(_SIMPLE_TEST_VAR_DECR_);
private:
    sejong::Vector save_tmp;
    int joint_idx;
};

#endif
