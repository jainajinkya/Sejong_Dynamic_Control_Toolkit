#ifndef WHOLE_BODY_DYNAMIC_CONTROL_SLIP_CONTACT
#define WHOLE_BODY_DYNAMIC_CONTROL_SLIP_CONTACT

class WBDC_Slip_ContactSpec: public ContactSpec{
public:
  WBDC_Slip_ContactSpec(int dim):ContactSpec(dim){}
  virtual ~WBDC_Slip_ContactSpec(){}

  virtual int getDimRFConstratint() { return Uf_.rows(); }

protected:
  virtual bool _AdditionalUpdate(){
    return _UpdateUf();
  }
  virtual bool _UpdateUf() = 0;
  
  sejong::Matrix Uf_;
};

#endif
