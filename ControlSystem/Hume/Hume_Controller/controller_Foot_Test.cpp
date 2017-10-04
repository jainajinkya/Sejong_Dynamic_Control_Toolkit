#include "controller_Foot_Test.h"
#include "constraint_library.h"
#include "task_library.h"
#include "StateProvider.h"
#include <fstream>

Ctrl_Foot_Test::Ctrl_Foot_Test(SJLinkID _target_foot):
    Controller_Hume(),
    target_foot_(_target_foot),
    Foot_des_(3),
    Foot_vel_des_(3),
    omega_(3.1415),
    amp_(0.05),
    phase_shift_(0.0),
    center_pt_(3) {
    center_pt_(0) = 0.1;
    center_pt_(1) =  0.13;
    center_pt_(2) = -0.73;
    constraint_ = new Hume_Fixed_Constraint();
    foot_task_ = new FOOT_Task(target_foot_);
}

Ctrl_Right_Foot_Test::Ctrl_Right_Foot_Test():Ctrl_Foot_Test(RFOOT){ }

Ctrl_Left_Foot_Test::Ctrl_Left_Foot_Test():Ctrl_Foot_Test(LFOOT){ }

Ctrl_Foot_Test::~Ctrl_Foot_Test(){
    delete foot_task_;
}

void Ctrl_Foot_Test::getCurrentCommand(std::vector<double> & command){
    _PreProcessing_Command();
    _save_initial_pos();
    _MakeTrajectory();

    task_array_.push_back(foot_task_);
    wbc_->MakeTorque(state_provider_->Q_,
                     state_provider_->Qdot_,
                     task_array_, constraint_, model_, gamma_);

    _PostProcessing_Command(command);
}

void Ctrl_Foot_Test::_save_initial_pos(){
    static bool init(false);

    if(!init){
        Foot_ini_ = state_provider_ -> getFootPos(target_foot_);
        init = true;
    }
}

void Ctrl_Foot_Test::_MakeTrajectory(){
    double t = state_provider_->curr_time_;
    Foot_des_(0) = Foot_ini_[0] + amp_*sin(omega_ * t);  
    Foot_des_(1) = Foot_ini_[1] ;   
    Foot_des_(2) = Foot_ini_[2] + amp_*cos(omega_ * t);

    Foot_vel_des_(0) = amp_*omega_*cos(omega_ * t);
    Foot_vel_des_(1) = 0.0;
    Foot_vel_des_(2) = -amp_*omega_*sin(omega_ * t);

    foot_task_->SetTask(Foot_des_, Foot_vel_des_,
                        state_provider_->getFootPos(target_foot_),
                        state_provider_->getFootVel(target_foot_));
}
