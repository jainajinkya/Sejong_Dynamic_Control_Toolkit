#ifndef DOUBLE_CONTACT_BOUNDING
#define DOUBLE_CONTACT_BOUNDING

#include <WBDC/WBDC_ContactSpec.hpp>

class RobotModel;
class StateProvider;

class DoubleContactBounding: public WBDC_ContactSpec{
public:
  DoubleContactBounding(int trans_pt);
  virtual ~DoubleContactBounding();

  void setFzUpperLimit(double lim);

protected:
  int trans_pt_;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  void _setU(double mu, sejong::Matrix & U);

  RobotModel* model_;
  StateProvider* sp_;
};


#endif
