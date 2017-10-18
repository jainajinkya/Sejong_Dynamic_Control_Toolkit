#ifndef DRACO_ROT_WBDC_BODY_TASK
#define DRACO_ROT_WBDC_BODY_TASK

#include <WBDC/WBDC_Task.hpp>

class StateProvider;
class DracoModel;

class BodyTask: public WBDC_Task{
public:
  BodyTask(int dim);
  virtual ~BodyTask();

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const sejong::Vector & vel_des,
                              const sejong::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();

  double Kp_, Kd_;

  StateProvider* sp_;
  DracoModel* model_;
};

#endif
