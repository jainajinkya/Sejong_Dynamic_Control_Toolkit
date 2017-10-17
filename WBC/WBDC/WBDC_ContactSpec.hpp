#ifndef WHOLE_BODY_DYNAMIC_CONTROL_CONTACT_SPEC
#define WHOLE_BODY_DYNAMIC_CONTROL_CONTACT_SPEC

#include <ContactSpec.hpp>

class WBDC_ContactSpec:public ContactSpec{
public:
  WBDC_ContactSpec(int dim):ContactSpec(dim){}
  virtual ~WBDC_ContactSpec(){}

  virtual int getDimRFConstratint() { return Uf_.rows(); }

protected:
  sejong::Matrix Uf_;
};
#endif
