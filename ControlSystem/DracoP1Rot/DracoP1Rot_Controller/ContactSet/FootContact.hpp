#ifndef DRACO_ROT_FOOT_CONTACT
#define DRACO_ROT_FOOT_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class DracoModel;
class StateProvider;

class FootContact: public WBDC_ContactSpec{
public:
  FootContact(int dim);
  virtual ~FootContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateWf();

  DracoModel* model_;
  StateProvider* sp_;
};

#endif
