#ifndef DOUBLE_TRANSITION_CONTACT
#define DOUBLE_TRANSITION_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>

class RobotModel;
class StateProvider;

class DoubleTransitionContact: public WBDC_ContactSpec{
public:
  DoubleTransitionContact();
  virtual ~DoubleTransitionContact();

  void setFzUpperLimit(double lim);

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  void _setU(double mu, sejong::Matrix & U);

  RobotModel* model_;
  StateProvider* sp_;
};


#endif
