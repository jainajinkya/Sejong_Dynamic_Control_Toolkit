#ifndef WHOLE_BODY_CONTROL_TASK
#define WHOLE_BODY_CONTROL_TASK

#include <Task.hpp>

class WBDC_Task: public Task{
public:
  WBDC_Task(int dim);
  virtual ~WBDC_Task();

  int getRelaxed_Dim(){ return dim_relaxed_;}
  void setRelaxedOpCtrl(const std::vector<bool> & relaxed_op);

protected:
  int dim_relaxed_;
  sejong::Matrix S_del_;
};

#endif
