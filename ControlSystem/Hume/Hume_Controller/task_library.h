#ifndef TASK_LIBRARY
#define TASK_LIBRARY

#include <WBOSC/Task.h>

class COM_Height_Tilting_Roll_Task: public Task{
    public:
    COM_Height_Tilting_Roll_Task();
    virtual ~COM_Height_Tilting_Roll_Task(){}

    virtual sejong::Matrix getJacobian(const sejong::Vector & conf, const WBOSC_Model* model);
    virtual sejong::Vector getCommand();

public:
    // Height & Pitch & Roll
    sejong::Vector Kp_;
    sejong::Vector Kd_;
    sejong::Vector Ki_;
    sejong::Vector error_sum_;
};

class COM_Height_Yaw_Tilting_Roll_Task: public Task{
    public:
    COM_Height_Yaw_Tilting_Roll_Task();
    virtual ~COM_Height_Yaw_Tilting_Roll_Task(){}

    virtual sejong::Matrix getJacobian(const sejong::Vector & conf, const WBOSC_Model* model);
    virtual sejong::Vector getCommand();

public:
    // Height & Yaw & Pitch & Roll
    Vector Kp_;
    Vector Kd_;
    Vector Ki_;
    Vector error_sum_;
};

class COM_Height_Tilting_Roll_Foot_Task: public Task{
public:
    COM_Height_Tilting_Roll_Foot_Task(SJLinkID);
    virtual ~COM_Height_Tilting_Roll_Foot_Task(){}

    virtual sejong::Matrix getJacobian(const Vector & conf, const WBOSC_Model* model);
    virtual Vector getCommand();

public:
    // Foot (LFOOT or RFOOT)
    SJLinkID link_id_;    
    //
    Vector error_sum_;
    Vector Ki_;
    
    Vector Kp_;
    Vector Kd_;
};

class COM_Height_Tilting_Roll_Foot_Task_Left: public COM_Height_Tilting_Roll_Foot_Task{
public:
    COM_Height_Tilting_Roll_Foot_Task_Left( );
    virtual ~COM_Height_Tilting_Roll_Foot_Task_Left(){}
};

class COM_Height_Tilting_Roll_Foot_Task_Right: public COM_Height_Tilting_Roll_Foot_Task{
public:
    COM_Height_Tilting_Roll_Foot_Task_Right();
    virtual ~COM_Height_Tilting_Roll_Foot_Task_Right(){}
};

class FOOT_Task: public Task{
public:
    FOOT_Task(SJLinkID);
    virtual ~FOOT_Task(){}

    virtual sejong::Matrix getJacobian(const Vector & conf, const WBOSC_Model* model);
    virtual Vector getCommand();

public:
    SJLinkID link_id_;    

    Vector Kp_;
    Vector Kd_;
    Vector Ki_;

    Vector error_sum_;

    Vector curr_foot_pos_;
    Vector curr_foot_vel_;

};

class Joint_Task: public Task{
public:
    Joint_Task();
    virtual ~Joint_Task(){}

    virtual sejong::Matrix getJacobian(const Vector & conf, const WBOSC_Model* model);
    virtual Vector getCommand();

public:
    Vector Kp_;
    Vector Kd_;
    Vector Ki_;
    Vector error_sum_;
    // From Right Abduction
    Vector jpos_des_;
    Vector jvel_des_;
};

#endif
