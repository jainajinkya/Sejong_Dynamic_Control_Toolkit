#ifndef WBOSC_TASK
#define WBOSC_TASK

#include <string>
#include <Utils/wrap_eigen.hpp>

class Task{
public:
  Task();
  virtual ~Task(){}

  virtual sejong::Matrix getJacobian() = 0;
  virtual sejong::Vector getCommand() = 0;

  void SetTask(const sejong::Vector & , const sejong::Vector &,
               const sejong::Vector & , const sejong::Vector & );

  void SetTask(const sejong::Vector & , const sejong::Vector &,
               const sejong::Vector & , const sejong::Vector &,
               const sejong::Vector & feedforward);

  const sejong::Vector & getForce() { return force_; }

  double crop_value(double value, double min, double max, std::string source);
  bool IsTaskSet(){ return b_settask_; }

  sejong::Vector force_;
  bool b_settask_;

  sejong::Vector Kp_;
  sejong::Vector Kd_;
  sejong::Vector Ki_;

protected:
  void _debug_show_task_setup();
  void _debug_show_task_setup(const sejong::Vector & input);

  sejong::Vector des_;
  sejong::Vector vel_des_;
  sejong::Vector act_;
  sejong::Vector vel_act_;
  sejong::Vector feedforward_;
  int num_control_DOF_;

  virtual void _PostProcess_Task();
};


#endif
