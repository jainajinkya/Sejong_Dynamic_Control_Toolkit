#ifndef DRACO_TEST
#define DRACO_TEST

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

class DracoController;

class DracoTest{
public:
  DracoTest();
  virtual ~DracoTest();

  virtual void TestInitialization() = 0;
  void getTorqueInput(sejong::Vector & gamma);

protected:
  virtual int _NextPhase(const int & phase) = 0;

  int phase_;
  std::vector<DracoController*> state_list_;
};


#endif
