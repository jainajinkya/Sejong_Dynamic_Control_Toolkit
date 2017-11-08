#ifndef WBDC_COM_BODY_ORIENTATION_TASK
#define WBDC_COM_BODY_ORIENTATION_TASK

#include <WBDC/WBDC_Task.hpp>

class StateProvider;
class RobotModel;

class CoMBodyOriTask: public WBDC_Task{
public:
  CoMBodyOriTask(); // 4 dim: Rx, Ry, Rz, Z
  virtual ~CoMBodyOriTask();

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
  RobotModel* model_;
};

#endif
