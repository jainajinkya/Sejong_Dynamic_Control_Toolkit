#ifndef PROCESS
#define PROCESS

#include "HumeSystem.h"
#include <WBOSC/Constraint.hpp>
#include <WBOSC/Task.h>

#include "utils/wrap_eigen.hpp"


using namespace sejong;

class StateProvider;
class Controller_Hume;
class WBOSC_Model;

class process{
public:
    process(StateProvider* state_provider, WBOSC_Model* model);
    virtual ~process(){}

    virtual void clear_process() = 0;
    virtual bool do_process(Controller_Hume* controller) = 0;
    void _set_current_as_initial(Controller_Hume* controller);
public:
    StateProvider* state_provider_;
    WBOSC_Model* model_;

    Vector init_lfoot_pos_;
    Vector init_rfoot_pos_;
};

class stabilizing : public process{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    stabilizing(StateProvider*, WBOSC_Model*);
    virtual ~stabilizing(){}

    virtual void clear_process();
    virtual bool do_process(Controller_Hume* controller);
    bool _zero_torque_preparing_phase(Controller_Hume* controller);
    void _set_current_foot_pos_vel();
    bool _move_to_desired_jpos(double curr_time, Controller_Hume * controller);
    bool _move_to_desired_foot_pos(double curr_time, Controller_Hume* controller);
    void _get_gamma_foot_control(Controller_Hume* controller, double curr_time, Vector & gamma_foot_control);

    bool _stable_lifting_height_tilting_roll_task(Controller_Hume* controller, double curr_time);
    
public:
    double set_pitch_;
    double stable_lifting_time_;
    double stable_move_time_;
    double lifting_height_;
    double start_height_;
    double start_x_;
    double initial_pitch_;
    double initial_roll_;
    
    bool b_stabilized_;
    bool b_move_stabilized_;
    bool b_foot_move_stabilized_;
    bool b_transit_; // Floating to Dual Contact
    int count_;
    Task* COM_height_tilting_roll_task_;
    Task* jpos_task_;
    Task* left_foot_task_;
    Task* right_foot_task_;

    
    WBC_Constraint* dual_contact_;
    WBC_Constraint* fixed_constraint_;

    double height_des_;
    Vector pos_des_;
    Vector vel_des_;
    sejong::Quaternion ori_des_;
    Vector ori_vel_des_;

    Vector height_pitch_des_; // Height & Pitch
    Vector height_pitch_vel_des_;

    Vector jpos_stable_;
    Vector stable_rfoot_pos_;
    Vector stable_lfoot_pos_;
    Vector jpos_des_;
    Vector jvel_des_;
    Vector jacc_des_;
    Vector jpos_init_;
    
    // From Primatic Joint
    Vector curr_lfoot_pos_;
    Vector curr_rfoot_pos_;
    Vector curr_lfoot_vel_;
    Vector curr_rfoot_vel_;

};

#endif
