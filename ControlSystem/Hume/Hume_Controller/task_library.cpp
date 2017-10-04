#include "task_library.h"
#include "Hume_Model/Hume_Model.h"
#include "utils/utilities.h"

///////////////////////////////////////////////////////
//////////         COM Height Tilting Roll
///////////////////////////////////////////////////////
COM_Height_Tilting_Roll_Task::COM_Height_Tilting_Roll_Task(): Task(){
    num_control_DOF_ = 3;
    error_sum_ = sejong::Vector::Zero(num_control_DOF_);
    force_ = sejong::Vector::Zero(num_control_DOF_);
    Ki_ = sejong::Vector::Zero(num_control_DOF_); 
    Kp_ = sejong::Vector::Zero(num_control_DOF_); 
    Kd_ = sejong::Vector::Zero(num_control_DOF_); 

    Kp_[0] = 130.0;
    Kp_[1] = 130.0;
    Kp_[2] = 130.0;

    Ki_[0] = 20.0;
    Ki_[1] = 20.0;
    Ki_[2] = 2.0;

    Kd_[0] = 10.0;
    Kd_[1] = 10.0;
    Kd_[2] = 10.0;

    declareParameter("Kp", & Kp_);
    declareParameter("Kd", & Kd_);
    declareParameter("Ki", & Ki_);
}

sejong::Matrix COM_Height_Tilting_Roll_Task::getJacobian(const Vector & conf, const WBOSC_Model* model){
    sejong::Matrix J = sejong::Matrix::Zero(3, model->NQdot());
    sejong::Matrix Jcom;
    ((HumeModel*)model)->getCoMJacobian(conf, Jcom); // 3 X N
    // sejong::pretty_print(Jcom, std::cout, "Jcom", "");
    J.block(0,0, 1, model->NQdot()) = Jcom.block(2, 0, 1, model->NQdot());
    
    sejong::Matrix Jw;
    model->getFullJacobian(conf, HIP, Jw);
    // sejong::pretty_print(Jw, std::cout, "J task", "");
    // Tilting
    J.block(1,0, 1, model->NQdot()) = Jw.block(1,0, 1, model->NQdot());
    // Roll
    J.block(2,0, 1, model->NQdot()) = Jw.block(0,0, 1, model->NQdot());
    // sejong::pretty_print(J, std::cout, "J task", "");
    return J;
}

sejong::Vector COM_Height_Tilting_Roll_Task::getCommand(){
    sejong::Vector input(3);
    // COM
    input(0) = Kp_(0)*(des_[0] - act_[0])
        + Kd_(0)*(vel_des_[0] - vel_act_[0]);

    // Orientation error
    Quaternion act_ori( act_[1], act_[2], act_[3], act_[4]);
    Quaternion des_ori( des_[1], des_[2], des_[3], des_[4] );
    Quaternion errQuat;
    errQuat = des_ori* act_ori.inverse();
    // errQuat = act_ori.inverse()* des_ori;

    Vector errso3;
    sejong::convert(errQuat, errso3);
    // sejong::pretty_print(errso3, std::cout, "errso3", "");
    // Pitch
    input[1] = Kp_[1]*errso3[1] + Kd_[1] * (vel_des_[1] - vel_act_[1]);

    // Roll
    input[2] = Kp_[2]*errso3[0] + Kd_[2] * (vel_des_[2] - vel_act_[2]);
    
    error_sum_[0] += (des_[0] - act_[0])*SERVO_RATE;
    error_sum_[1] += errso3[1]*SERVO_RATE;
    error_sum_[2] += errso3[0]*SERVO_RATE;
    
    for(int i(0); i< num_control_DOF_; ++i){
        error_sum_[i] = crop_value(error_sum_[i], -10.0, 10.0, "[(COM) Height & Tilting & Roll] error sum");
        input[i] += error_sum_[i] * Ki_[i];
    }
        
    double limit = 1000.0;
    for (int i(0); i< num_control_DOF_; ++i){
        input[i] = crop_value(input[i], -limit, limit, "(COM) Height & Tilting & Roll");
    }
    // Post Process
    _PostProcess_Task();
    // _debug_show_task_setup();

    return input;
}

///////////////////////////////////////////////////////
//////////         COM Height Yaw Tilting Roll
///////////////////////////////////////////////////////
COM_Height_Yaw_Tilting_Roll_Task::COM_Height_Yaw_Tilting_Roll_Task():Task()
{
    // COM height, Body Yaw (Rz), Tilting (Ry), Body Roll (Rx)
    num_control_DOF_ = 4;
    error_sum_ = sejong::Vector::Zero(num_control_DOF_);
    force_ = sejong::Vector::Zero(num_control_DOF_);
    Ki_ = sejong::Vector::Zero(num_control_DOF_);
    Kp_ = sejong::Vector(num_control_DOF_);
    Kd_ = sejong::Vector(num_control_DOF_);

    declareParameter("Kp", & Kp_);
    declareParameter("Kd", & Kd_);
    declareParameter("Ki", & Ki_);
}

sejong::Matrix COM_Height_Yaw_Tilting_Roll_Task::getJacobian(const Vector & conf, const WBOSC_Model* model){
    sejong::Matrix J = sejong::Matrix::Zero(num_control_DOF_, model->NQdot());
    sejong::Matrix Jcom;
    ((HumeModel*)model)->getCoMJacobian(conf, Jcom);
    J.block(0,0, 1, model->NQdot()) = Jcom.block(2, 0, 1, model->NQdot());

    sejong::Matrix Jw;
    model->getFullJacobian(conf, HIP, Jw);
    // Yaw
    J.block(1,0, 1, model->NQdot()) = Jw.block(2, 0, 1, model->NQdot());
    // Pitch
    J.block(2,0, 1, model->NQdot()) = Jw.block(1, 0, 1, model->NQdot());
    // Roll
    J.block(3,0, 1, model->NQdot()) = Jw.block(0, 0, 1, model->NQdot());

    return J;
}

sejong::Vector COM_Height_Yaw_Tilting_Roll_Task::getCommand(){
    sejong::Vector input(num_control_DOF_);
    // COM
    input[0] = Kp_[0]*(des_[0] - act_[0]) + Kd_[0]*(vel_des_[0] - vel_act_[0]);
    // Yaw, Pitch, Roll
    Quaternion act_ori( act_[1], act_[2], act_[3], act_[4]);
    Quaternion des_ori( des_[1], des_[2], des_[3], des_[4] );
    Quaternion errQuat;
    errQuat = des_ori* act_ori.inverse();
    Vector errso3;
    sejong::convert(errQuat, errso3);

    for (int i(1); i<4; ++i){
        input[i] = Kp_[i]*errso3[3-i] + Kd_[i] * (vel_des_[i] - vel_act_[i]);
    }
    // _debug_show_task_setup();
    error_sum_[0] += (des_[0] - act_[0])*SERVO_RATE;
    error_sum_[1] += errso3[2]*SERVO_RATE;
    error_sum_[2] += errso3[1]*SERVO_RATE;
    error_sum_[3] += errso3[0]*SERVO_RATE;
    
    for(int i(0); i< num_control_DOF_; ++i){
        error_sum_[i] = crop_value(error_sum_[i], -30.0, 30.0, "[(COM) Height & Yaw & Tilting & Roll] error sum");
        input[i] += error_sum_[i] * Ki_[i];
    }
        
    double limit = 1000.0;
    for (int i(0); i< num_control_DOF_; ++i){
        input[i] = crop_value(input[i], -limit, limit, "(COM) Height & Yaw & Tilting & Roll");
    }
    // Post Process
    _PostProcess_Task();
    // _debug_show_task_setup();
    return input;
}

COM_Height_Tilting_Roll_Foot_Task_Left::COM_Height_Tilting_Roll_Foot_Task_Left():COM_Height_Tilting_Roll_Foot_Task(LFOOT){
}

COM_Height_Tilting_Roll_Foot_Task_Right::COM_Height_Tilting_Roll_Foot_Task_Right():COM_Height_Tilting_Roll_Foot_Task(RFOOT){
}

///////////////////////////////////////////////////////
//////////     COM HEIGHT TILTING Roll FOOT TASK
///////////////////////////////////////////////////////
COM_Height_Tilting_Roll_Foot_Task::COM_Height_Tilting_Roll_Foot_Task(SJLinkID link_id): Task(), link_id_(link_id)
{
    // Integral Gain
    num_control_DOF_ = 6;
    error_sum_ = sejong::Vector::Zero(num_control_DOF_);
    Ki_ = sejong::Vector::Zero(num_control_DOF_);
    // PD Gain
    Kp_ = sejong::Vector(num_control_DOF_);
    Kd_ = sejong::Vector(num_control_DOF_);

    force_ = sejong::Vector::Zero(num_control_DOF_);

    declareParameter("Kp", & Kp_);
    declareParameter("Kd", & Kd_);
    declareParameter("Ki", & Ki_);
}

sejong::Matrix COM_Height_Tilting_Roll_Foot_Task::getJacobian(const Vector & conf, const WBOSC_Model* model){
    sejong::Matrix J = sejong::Matrix::Zero(num_control_DOF_, model->NQdot());
    sejong::Matrix J_com, J_foot, J_body;
    ((HumeModel*)model)->getCoMJacobian(conf, J_com);
    J.block(0,0, 1, model->NQdot()) = J_com.block(2,0, 1, model->NQdot());
    
    model->getFullJacobian(conf, HIP, J_body);
    // Pitch
    J.block(1, 0, 1, model->NQdot()) = J_body.block(1, 0, 1, model->NQdot());
    // Roll
    J.block(2, 0, 1, model->NQdot()) = J_body.block(0, 0, 1, model->NQdot());

    model->getFullJacobian(conf, link_id_, J_foot);
    J.block(3, 0, 3, model->NQdot()) = J_foot.block(3,0, 3, model->NQdot());
    // pretty_print(J, std::cout, "Jfull", "");
    return J;
}


sejong::Vector COM_Height_Tilting_Roll_Foot_Task::getCommand(){
    sejong::Vector input(num_control_DOF_);
    // Body Height
    input(0) =
        Kp_(0)*(des_[0] - act_[0])
        + Kd_(0)*(vel_des_[0] - vel_act_[0]);

    // Pitch & Roll
    Quaternion act_ori( act_[1], act_[2], act_[3], act_[4]);
    Quaternion des_ori( des_[1], des_[2], des_[3], des_[4] );
    Quaternion errQuat;
    // errQuat = des_ori.inverse()* act_ori;
    errQuat = des_ori * act_ori.inverse();
    Vector errso3;
    sejong::convert(errQuat, errso3);

    // Pitch
    input[1] = Kp_[1]*errso3[1] + Kd_[1] * (vel_des_[1] - vel_act_[1]);
    // Roll
    input[2] = Kp_[2]*errso3[0] + Kd_[2] * (vel_des_[2] - vel_act_[2]);

    
    // Foot 
    for (int i(3); i< 6; ++i){
        input[i] = Kp_[i] * (des_[i+2] - act_[i+2]) + Kd_[i] * (vel_des_[i] - vel_act_[i]);
    }

    //
    error_sum_[0] += (des_[0] - act_[0])*SERVO_RATE;
    error_sum_[1] += (errso3[1])*SERVO_RATE;
    error_sum_[2] += (errso3[0])*SERVO_RATE;

    // Foot
    for (int i(0); i< 3; ++i){
        error_sum_[i+3] += (des_[i+5] - act_[i+5])*SERVO_RATE;
    }
  
    // Integral Control
    for(int i(0); i< num_control_DOF_; ++i){
        error_sum_[i] = crop_value(error_sum_[i], -10.0, 10.0, "[COM height tiltingroll foot]error sum");
        input[i] += Ki_[i] * error_sum_[i];
    }
    
    double limit = 1050.0;
    for (int i(0); i< num_control_DOF_; ++i){
        input[i] = crop_value(input[i], -limit, limit, "COM height_tilting_roll & Foot");
    }
    // Post Process
    _PostProcess_Task();
    // _debug_show_task_setup();
    return input;
}

///////////////////////////////////////////////////////
//////////         FOOT TASK
///////////////////////////////////////////////////////
//task_foot
FOOT_Task::FOOT_Task(SJLinkID link_id):
    Task(), link_id_(link_id), error_sum_(Vector::Zero(3))
{
    num_control_DOF_ = 3;
    Kp_ = Vector(num_control_DOF_);
    Kd_ = Vector(num_control_DOF_);
    Ki_ = Vector(num_control_DOF_);
    force_ = sejong::Vector::Zero(num_control_DOF_);

    for (int i(0); i< 3; ++i){
        Kp_(i) = 20.0;
        Kd_(i) = 10.0;
        Ki_(i) = 10.0;
    }
}

Matrix FOOT_Task::getJacobian(const Vector & conf, const WBOSC_Model* model){
    Matrix J;
    model->getFullJacobian(conf, link_id_, J);    
    return J.block(3,0, 3, model->NQdot());
}

Vector FOOT_Task::getCommand(){
    Vector input(3);
    for (int i(0); i< 3; ++i){
        input(i) =
            Kp_[i] * (des_[i] - act_[i]) +
            Kd_[i] * (vel_des_[i] - vel_act_[i]);
        
        error_sum_[i] += (des_[i] - act_[i])*SERVO_RATE;
    }

    // Integral Control
    for (int i(0); i <3 ; ++i){
        error_sum_[i] = crop_value(error_sum_[i], -100.5, 100.5, "[Foot Task] error sum");
        input[i] += Ki_[i] * error_sum_[i];
        // Crop
        input[i] = crop_value(input[i], -650.0, 650.0, "Foot");
    }

    // Post Process
    _PostProcess_Task();
    // _debug_show_task_setup(input);

    return input;
}

///////////////////////////////////////////////////////
//////////         Joint TASK
///////////////////////////////////////////////////////
Joint_Task::Joint_Task():Task(){
    num_control_DOF_ = 6;
    Kp_ = Vector::Zero(num_control_DOF_);
    Kd_ = Vector::Zero(num_control_DOF_);
    Ki_ = Vector::Zero(num_control_DOF_);
    error_sum_ = Vector::Zero(num_control_DOF_);
    force_ = sejong::Vector::Zero(num_control_DOF_);

    for (int i(0); i< num_control_DOF_; ++i){
        Kp_[i] = 170.0;
        Kd_[i] = 10.0;
        Ki_[i] = 50.0;
    }

    declareParameter("Kp", & Kp_);
    declareParameter("Kd", & Kd_);
    declareParameter("Ki", & Ki_);
}

Matrix Joint_Task::getJacobian(const Vector & conf, const WBOSC_Model* model){
    Matrix J = Matrix::Zero(model->NAJ(), model->NQdot());
    J.block(0, model->NPQdot(), model->NAJ(), model->NAJ()) = Matrix::Identity(model->NAJ(), model->NAJ());
    return J;
}

Vector Joint_Task::getCommand(){
    Vector input(num_control_DOF_);
    for (int i(0); i< num_control_DOF_; ++i){
        error_sum_[i] += (des_[i] - act_[i]) * SERVO_RATE;
        input[i] = feedforward_[i] + Kp_[i]*(des_[i] - act_[i]) + Kd_[i]*(vel_des_[i] - vel_act_[i]);
    }
    for( int i(0); i< num_control_DOF_; ++i){
        error_sum_[i] = crop_value(error_sum_[i], -55.0, 55.0, "error sum jpos control");
        input[i] += Ki_[i]*error_sum_[i];
        input[i] = crop_value(input[i], -150.0, 150.0, "Joint Task");
    }
    
    _PostProcess_Task();
    // _debug_show_task_setup(input);

    return input;
}

