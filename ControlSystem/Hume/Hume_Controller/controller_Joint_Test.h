#ifndef CONTROLLER_JOINT_TEST
#define CONTROLLER_JOINT_TEST

#include "Controller_Hume.h"

class Ctrl_Joint_Test: public Controller_Hume{
public:
    Ctrl_Joint_Test();
    virtual ~Ctrl_Joint_Test();
    
public:
    virtual void getCurrentCommand(std::vector<double> & command);
    
public:
    Task * jpos_task_;
    WBC_Constraint * fixed_constraint_;

    sejong::Vector jpos_ini_;

    SJLinkID observing_foot_;
    
protected:
    sejong::Vector des_acc_;
    sejong::Vector des_;
    sejong::Vector vel_des_;

    sejong::Vector omega_;
    sejong::Vector amp_;
    sejong::Vector offset_;
    sejong::Vector phase_shift_;
    
    double start_time_;
    double state_machine_time_;
    int count_command_;

    // Function
    void _Test_Joint();
    void _SetDesJPos();

    int initialization_count_;
};


#endif
