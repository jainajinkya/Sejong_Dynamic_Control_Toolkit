#ifndef CONTROLLER_3D_WALKING
#define CONTROLLER_3D_WALKING

#include "Controller_Hume.h"
#include "controller_Step.h"

class Planner;
class Planner_Info;

class Ctrl_3D_Walking : public Virtual_Ctrl_Step{
public:
    Ctrl_3D_Walking();
    virtual ~Ctrl_3D_Walking();
    virtual void getCurrentCommand(std::vector<double> & command);

public:
    bool CalculateLandingLocation(const Vector & stance_foot_loc, double current_time, double local_switch_time);
    
///////  Member Variable ///////////////////
    Planner* planner_;
    Planner_Info* planner_info_;
    
    double step_width_;

    double middle_time_x_;
    double middle_time_y_;

    double switching_time_;
    double achieving_ratio_;
    std::vector<Vector> com_pos_trj_supp_;
    std::vector<Vector> com_vel_trj_supp_;

    Vector com_vel_ave_;
    Vector body_vel_pre_;
    Vector body_vel_pre_2_;
    Vector body_vel_pre_3_;

    double previous_time_;
    
    double lifting_vel_;
    double remaining_time_;

    Vector a, b, c, d;

    double const_landing_time_;
    double com_x_offset_;
    double com_y_offset_;
    double com_z_offset_;
    double tot_transition_time_;

protected:
    double vel_est_;
    double pos_est_; // Not used
    void _set_foot_x_traj_coeff(const Vector & curr_vel);
    // virtual void _set_dual_contact_task();
    virtual bool _Do_Supp_Lift();
    virtual bool _Do_Supp_Land();
    virtual bool _Stop();
    virtual void _set_land_location();
    virtual void _set_apex_foot_location();

    void _Calculate_CoM_Velocity_MoCap(Vector & com_vel);
    void _CoM_Position_From_Kalman_Filter(Vect3 & com_pos);
    
    int initialization_count_;
};


#endif
