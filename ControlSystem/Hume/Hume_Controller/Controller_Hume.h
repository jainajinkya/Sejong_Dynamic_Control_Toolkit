#ifndef _Controller_Hume_
#define _Controller_Hume_

#include <vector>
#include "Scenario/Parameter.h"
#include <WBOSC/WBOSC.h>

#include "StateProvider.h"
#include "Hume_Model/Hume_Model.h"


using namespace sejong;
using namespace std;

class process;
class WBOSC_Constraint;
class Task;

class Controller_Hume : public object{
public:
    Controller_Hume();
    virtual ~Controller_Hume();
    
public:
    virtual void getCurrentCommand(std::vector<double> & command) = 0;

    bool Is_Integral_Gain_Right() { return b_int_gain_right_; }
    bool Is_Integral_Gain_Left() { return b_int_gain_left_; }
    bool Force_Integral_Gain_Left_Knee() { return b_force_int_gain_left_knee_;}
    bool Force_Integral_Gain_Right_Knee() { return b_force_int_gain_right_knee_; }

    
    void CentroidalMomentControl(const sejong::Vector & cent_des,
                                 const sejong::Vector & cent_vel_des,
                                 sejong::Vector & gamma);
    void CentroidalMomentControl_DoubleContact(
        const sejong::Vector & cent_des, // CoM Height + 3 DoF Orientation
        const sejong::Vector & cent_vel_des,
        sejong::Vector & fr);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Controller_WBC* wbc_;
    Controller_WBC* wbc_2_;

    Vector gamma_;
    std::vector<Task*> task_array_;
    WBC_Constraint * constraint_;

    sejong::Quaternion ori_ini_;
    
    Vector pos_ini_;
    Vector left_foot_ini_;
    Vector right_foot_ini_;

    Vector RFoot_force_;
    Vector LFoot_force_;

    //Stabilizing Process
    process * pro_stable_;
    bool motion_start_;
    int phase_;

    bool b_int_gain_right_;
    bool b_int_gain_left_;
    bool b_force_int_gain_left_knee_;
    bool b_force_int_gain_right_knee_;

    SJLinkID base_pt_;

protected:
    HumeModel* model_;
    StateProvider * state_provider_;
    
    void _PreProcessing_Command();
    void _PostProcessing_Command(std::vector<double> & command);
};

#endif
