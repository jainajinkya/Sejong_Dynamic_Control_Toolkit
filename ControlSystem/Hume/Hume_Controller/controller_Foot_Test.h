#ifndef CONTROLLER_FOOT_TEST
#define CONTROLLER_FOOT_TEST

#include "Controller_Hume.h"
#include "Configuration.h"

class Ctrl_Foot_Test: public Controller_Hume{
public:
    Ctrl_Foot_Test( SJLinkID );
    virtual ~Ctrl_Foot_Test();

public:
    virtual void getCurrentCommand(std::vector<double> & command);

    void _save_initial_pos();
public:
    Task* foot_task_;
    SJLinkID target_foot_;

    sejong::Vector Foot_des_;
    sejong::Vector Foot_vel_des_;
    double omega_;
    double amp_;
    double phase_shift_;
    sejong::Vector center_pt_;
    
protected:
    sejong::Vector Foot_ini_;
    void _MakeTrajectory();
};

class Ctrl_Right_Foot_Test: public Ctrl_Foot_Test{
public:
    Ctrl_Right_Foot_Test();
    virtual ~Ctrl_Right_Foot_Test(){}
};

class Ctrl_Left_Foot_Test: public Ctrl_Foot_Test{
public:
    Ctrl_Left_Foot_Test();
    virtual ~Ctrl_Left_Foot_Test(){}
};

#endif





