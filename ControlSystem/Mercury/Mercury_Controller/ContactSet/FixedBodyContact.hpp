#ifndef FIXED_BODY_CONTACT
#define FIXED_BODY_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotModel;
class StateProvider;

class FixedBodyContact: public WBDC_ContactSpec{
public:
  FixedBodyContact();
  virtual ~FixedBodyContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();

  RobotModel* model_;
  StateProvider* sp_;
};

#endif
