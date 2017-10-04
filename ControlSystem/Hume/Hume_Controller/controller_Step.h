#ifndef CONTROLLER_STEP_TEST
#define CONTROLLER_STEP_TEST

#include "Controller_Hume.h"

enum State_Machine{
    LsDu = 0,  LsTran = 1, LsRl = 2, LsRm = 3, LsRs = 4, RsDu = 5, RsTran = 6, RsLl = 7, RsLm = 8, RsLs = 9, STOP = -1
};

class Ctrl_Step: public Controller_Hume{
public:
    Ctrl_Step();
    virtual ~Ctrl_Step();
    
public:
    virtual void getCurrentCommand(std::vector<double> & command);
    
public:
    Task * dual_contact_task_;
    Task * single_contact_lfoot_;
    Task * single_contact_rfoot_;

    WBC_Constraint* constraint_dual_;
    WBC_Constraint* constraint_left_;
    WBC_Constraint* constraint_right_;

    sejong::Vector foot_des_;
    sejong::Vector foot_vel_des_;

    sejong::Vector foot_pre_;

    int num_step_;
    double state_machine_time_;

    double lifting_time_;
    double landing_time_;
    double supp_time_;
    double lifting_height_;
    double supp_lift_transition_time_;
    double land_supp_transition_time_;

    SJLinkID swing_foot_;

    sejong::Vector landing_loc_;

    double walking_length_x_;
    double walking_length_y_;
    
protected:
    double start_time_;
    sejong::Vector apex_foot_loc_;
    int count_command_;
    double x_init_;
    
    // Function
    bool _IsEndCurrState(const int curr_phase);
    virtual bool _Supp() = 0;
    virtual bool _Do_Supp_Land() = 0;
    virtual bool _Supp_Lift_transition() = 0;
    virtual bool _Land_Supp_transition() = 0;
    virtual bool _Do_Supp_Lift() = 0;    
    virtual bool _Stop(){ return false; }
    
    virtual void _set_apex_foot_location(){}
    virtual void _set_land_location() = 0;

    virtual void _set_dual_contact_task() = 0;
    virtual void _set_single_contact_task() = 0;
    
    virtual void _end_action();
    void _printf_phase(int phase);
};

class Virtual_Ctrl_Step: public Ctrl_Step{
public:
    Virtual_Ctrl_Step();
    virtual ~Virtual_Ctrl_Step();

public:
    virtual bool _Supp_Lift_transition();
    virtual bool _Land_Supp_transition();
    virtual bool _Supp();

protected:
    virtual void _set_dual_contact_task();
    virtual void _set_single_contact_task();
    void _get_smooth_changing_foot_force(const Vector & start_force, const Vector & end_force, double duration, double curr_time, Vector & ret_force);

};

#endif
