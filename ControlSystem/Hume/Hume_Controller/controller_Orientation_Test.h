#ifndef CONTROLLER_ORIENTATION_TEST
#define CONTROLLER_ORIENTATION_TEST

#include "Controller_Hume.h"

class Ctrl_Orientation_Test : public Controller_Hume{
public:
    Ctrl_Orientation_Test();
    virtual ~Ctrl_Orientation_Test();
    virtual void getCurrentCommand(std::vector<double> & command);
public:
    Task * single_contact_task_;
    WBC_Constraint* constraint_single_;

    double start_time_;
    double state_machine_time_;

    Vector foot_ini_;
    
    Vector foot_des_;
    Vector foot_vel_des_;
    // pitch, roll
    Vector omega_;
    Vector amp_;
    Vector offset_;
    int count_command_;

    SJLinkID swing_foot_;

    Vector EulerZYX_ini_;
    
protected:
    void _Set_Desired_Foot();
    void _Set_Test_Task();
    void _Test_Orientation();
    void _Disable_Fixed_Leg(sejong::Vector & command);
};

#endif
