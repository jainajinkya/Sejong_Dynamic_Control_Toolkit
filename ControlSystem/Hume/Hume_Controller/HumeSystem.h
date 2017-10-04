#ifndef HUME_SYSTEM
#define HUME_SYSTEM

#include <vector>
#include "utils/wrap_eigen.hpp"

#include <Scenario/Parameter.h>
#include "Configuration.h"

class Controller_Hume;
class Factory;
class Sejong_Thread;
class Foot_Force_Calculator;

class HUME_System : public object
{
public:
    HUME_System();
    ~HUME_System();
    
    void getTorqueInput(std::vector<signed int> & pos_gain_kp, 
                        std::vector<signed int> & pos_gain_ki,
                        std::vector<signed int> & motor_gain_kv,
                        sejong::Vector & torque_command);
    
    void setCommand(std::vector<double> & command, sejong::Vector & torque_command);
    void Safty_Stop( const std::vector<double> & jvel, std::vector<double> & command);

    int hume_system_count_;
    long long start_count_;
    double curr_time_;

    bool b_sagittal_motion_;

    std::vector<double> command_;

    sejong::Vector pos_gain_kp_;
    sejong::Vector pos_gain_ki_;
    sejong::Vector motor_gain_kv_;
    sejong::Vector torque_limit_;

    void setGain(std::vector<signed int> & kp,
                 std::vector<signed int> & ki,
                 std::vector<signed int> & kv);

    // Because of the simulator they are in public members
    Controller_Hume*  controller_;

protected:
    Factory* hume_factory_;
    Foot_Force_Calculator* foot_force_calculator_;
};

#endif
