#ifndef MERCURY_FOOT_CONTACT
#define MERCURY_FOOT_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class MercuryModel;
class StateProvider;

class FootContact: public WBDC_ContactSpec{
public:
  FootContact(int dim);
  virtual ~FootContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  MercuryModel* model_;
  StateProvider* sp_;
};

#endif
