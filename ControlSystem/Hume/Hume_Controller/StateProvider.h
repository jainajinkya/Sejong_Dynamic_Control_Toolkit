#ifndef STATE_PROVIDER_H 
#define STATE_PROVIDER_H 

#include "utils/wrap_eigen.hpp"
#include "Configuration.h"

using namespace sejong;

class StateProvider{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static StateProvider* GetStateProvider();
    
    ~StateProvider(){}

    bool contact_constraint_;
    
    bool hold_xy_;
    bool task_start_;
    bool initialized_;

    double height_offset_;
    double curr_time_;
    int system_count_;
    
    Vect3 rfoot_force_;
    Vect3 lfoot_force_;
    Vect3 curr_RFoot_force_;
    Vect3 curr_LFoot_force_;

    bool  right_foot_contact_;
    bool  left_foot_contact_;
    bool RFoot_Contact() { return right_foot_contact_; }
    bool LFoot_Contact() { return left_foot_contact_; }
    
    Vect3 rfoot_pos_;
    Vect3 lfoot_pos_;
    Vect3 rfoot_vel_;
    Vect3 lfoot_vel_;
    
    const Vect3 & getFootPos(SJLinkID id);
    const Vect3 & getFootVel(SJLinkID id);

    const Vect3 & getBodyEulerZYX(){ return Body_Euler_ori_; }

    Vector Q_sim_;
    Vector Qdot_sim_;
    Vector Qddot_sim_;
    
    Vector Q_;
    Vector Qdot_;
    Vector Q_est_;
    Vector Qdot_est_;
    Vector Foot_Contact_;
    Vector curr_torque_;

    Vect4 getBodyOri();
    sejong::Quaternion getBodyOriQuat();

    Vect3 Body_Euler_ori_;
    Vect3 Body_ang_vel_;
    Vect3 Body_vel_led_;
    Vect3 Body_pos_orientation_calculator_;
    
    Vect3 CoM_pos_;
    Vect3 CoM_vel_;
    Vect3 CoM_vel_filtered_;
    
    Vect3 local_ang_vel_;
    Vect3 imu_lin_acc_;
    
    bool change_imu_mode_;
    Vector attraction_loc_;
    // Just large enough number: To save task set
    Vector task_des_;
    Vector task_curr_;
    Vector task_vel_des_;
    Vector task_vel_curr_;
    double task_time_;
    void SaveTaskData(const Vector & act, const Vector & des,
                      const Vector & vel_act, const Vector & vel_des);
    
    
    Vector kalman_plant_observation_;
    Vector kalman_filter_state_;
    Vect3 com_kalman_filter_;

    std::vector<bool> led_cond_;

    Vector left_LED_x_;
    Vector left_LED_y_;
    Vector left_LED_z_;

    Vector right_LED_x_;
    Vector right_LED_y_;
    Vector right_LED_z_;

    // Update from Controller
    int phase_;
    std::vector<double> torque_input_;
private:
    StateProvider();
};


#endif
