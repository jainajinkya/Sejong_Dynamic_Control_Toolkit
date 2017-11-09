#ifndef WBDC_COM_BODY_ORIENTATION_FOOT_TASK
#define WBDC_COM_BODY_ORIENTATION_FOOT_TASK

#include <WBDC/WBDC_Task.hpp>

class StateProvider;
class RobotModel;

// CoM_{x, y, z}, BodyOri_{Rx, Ry, Rz}, Foot (x, y, z)
class BodyFootTask: public WBDC_Task{
public:
  BodyFootTask(int swing_foot); // 4 dim: Rx, Ry, Rz, Z
  virtual ~BodyFootTask();

  sejong::Vector Kp_vec_;
  sejong::Vector Kd_vec_;

protected:
  int swing_foot_;

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
