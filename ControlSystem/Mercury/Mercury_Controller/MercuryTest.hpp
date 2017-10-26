#ifndef MERCURY_TEST
#define MERCURY_TEST

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class MercuryController;

class MercuryTest{
public:
  MercuryTest();
  virtual ~MercuryTest();

  virtual void TestInitialization() = 0;
  void getTorqueInput(sejong::Vector & gamma);

protected:
  virtual int _NextPhase(const int & phase) = 0;

  bool b_first_visit_;
  int phase_;
  std::vector<MercuryController*> state_list_;
};


#endif
