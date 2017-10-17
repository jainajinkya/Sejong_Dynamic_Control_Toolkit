#ifndef CONTACT_SPEC
#define CONTACT_SPEC

class ContactSpec{
public:
  ContactSpec(int dim):dim_contact_(dim){}
  virtual ~ContactSpec(){}

  void getContactJacobian(sejong::Matrix & Jc){ Jc = Jc_; }
  int getDim(){ return dim_contact_; }

  virtual bool UpdateContactSpec() = 0;

protected:
  sejong::Matrix Jc_;
  int dim_contact_;
};
#endif
