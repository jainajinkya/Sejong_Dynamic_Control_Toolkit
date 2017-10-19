#ifndef WHOLE_BODY_DYNAMIC_CONTROL_CONTACT_SPEC
#define WHOLE_BODY_DYNAMIC_CONTROL_CONTACT_SPEC

#include <ContactSpec.hpp>

class WBDC_ContactSpec:public ContactSpec{
public:
  WBDC_ContactSpec(int dim):ContactSpec(dim){}
  virtual ~WBDC_ContactSpec(){}

  virtual int getDimRFConstratint() { return Uf_.rows(); }
  void getRFConstraintMtx(sejong::Matrix & Uf){ Uf = Uf_; }

protected:
  virtual bool _AdditionalUpdate(){
    _UpdateUf();
    return true;
  }
  virtual bool _UpdateUf() = 0;

  sejong::Matrix Uf_;
};
#endif
