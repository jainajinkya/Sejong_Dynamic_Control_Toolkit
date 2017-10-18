#ifndef WHOLE_BODY_DYNAMIC_CONTROL_CONTACT_SPEC
#define WHOLE_BODY_DYNAMIC_CONTROL_CONTACT_SPEC

#include <ContactSpec.hpp>

class WBDC_ContactSpec:public ContactSpec{
public:
  WBDC_ContactSpec(int dim):ContactSpec(dim){}
  virtual ~WBDC_ContactSpec(){}

  virtual int getDimRFConstratint() { return Uf_.rows(); }
  void getRFConstraintMtx(sejong::Matrix & Uf){ Uf = Uf_; }
  void getCMProjectionMtx(sejong::Matrix & Wf){ Wf = Wf_; }
protected:
  virtual bool _AdditionalUpdate(){
    _UpdateUf();
    _UpdateWf();
    return true;
  }
  virtual bool _UpdateUf() = 0;
  virtual bool _UpdateWf() = 0;

  sejong::Matrix Wf_; // cam only
  sejong::Matrix Uf_;
};
#endif
