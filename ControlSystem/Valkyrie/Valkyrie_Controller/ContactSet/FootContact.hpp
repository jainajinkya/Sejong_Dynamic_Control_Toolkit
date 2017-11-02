#ifndef VALKYRIE_FOOT_CONTACT
#define VALKYRIE_FOOT_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotModel;
class StateProvider;

class FootContact: public WBDC_ContactSpec{
public:
  FootContact(int dim);
  virtual ~FootContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();

  RobotModel* model_;
  StateProvider* sp_;
};

#endif
