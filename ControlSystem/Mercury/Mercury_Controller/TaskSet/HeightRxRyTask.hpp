#ifndef WBDC_HEIGHT_RX_RY_TASK
#define WBDC_HEIGHT_RX_RY_TASK

#include <WBDC/WBDC_Task.hpp>

class StateProvider;
class RobotModel;

class HeightRxRyTask: public WBDC_Task{
public:
  HeightRxRyTask(); // 4 dim: Rx, Ry, Rz, Z
  virtual ~HeightRxRyTask();

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
