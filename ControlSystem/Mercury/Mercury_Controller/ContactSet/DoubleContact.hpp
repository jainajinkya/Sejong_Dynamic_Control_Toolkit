#ifndef DOUBLE_CONTACT
#define DOUBLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>

class RobotModel;
class StateProvider;

class DoubleContact: public WBDC_ContactSpec{
public:
  DoubleContact();
  virtual ~DoubleContact();

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
