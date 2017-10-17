#ifndef WBC_TASK
#define WBC_TASK

#include <Utils/wrap_eigen.hpp>

class Task{
public:
  Task(int dim):b_set_task_(false), dim_task_(dim){}
  virtual ~Task(){}

  void getCommand(sejong::Vector & op_cmd){  op_cmd = op_cmd_; }
  void getTaskJacobian(sejong::Matrix & Jt){ Jt = Jt_; }
  void getTaskJacobianDotQdot(sejong::Vector & JtDotQdot) {
    JtDotQdot = JtDotQdot_;
  }

  bool UpdateTask(void* pos_des, const sejong::Vector & vel_des, const sejong::Vector & acc_des){
    bool ret(true);
    if(_UpdateCommand(pos_des, vel_des, acc_des)){
      if(_UpdateTaskJacobian()){
        if(_UpdateTaskJDotQdot()){
        } else { return false; }
      }else{ return false; }
    }else { return false; }

    b_set_task_ = true;
    return ret;
  }

  int getDim(){ return dim_task_; }
protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const sejong::Vector & vel_des,
                              const sejong::Vector & acc_des) = 0;
  // Update Jt_
  virtual bool _UpdateTaskJacobian() = 0;
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot() = 0;

  sejong::Vector op_cmd_;
  sejong::Vector JtDotQdot_;
  sejong::Matrix Jt_;

  bool b_set_task_;
  int dim_task_;
};

#endif
