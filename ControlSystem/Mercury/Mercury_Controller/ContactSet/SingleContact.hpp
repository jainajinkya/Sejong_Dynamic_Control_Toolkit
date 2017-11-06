#ifndef SINGLE_CONTACT
#define SINGLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotModel;
class StateProvider;

class SingleContact: public WBDC_ContactSpec{
public:
  SingleContact(int contact_pt);
  virtual ~SingleContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();

  RobotModel* model_;
  StateProvider* sp_;

  int contact_pt_;
};

#endif
