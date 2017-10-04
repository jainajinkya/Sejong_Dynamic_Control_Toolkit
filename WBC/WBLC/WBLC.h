#ifndef WBLC_H
#define WBLC_H
// Whole Body Locomotion Controller

///////////////////////////////////////////////////////
// WARNNING: The last task of task array must be
//           Centroidal Angular Moment Task
///////////////////////////////////////////////////////

#include <vector>
#include <Utils/wrap_eigen.hpp>

using namespace sejong;
using namespace std;

class ReactionForceCalculator;
class ContactWrenchCalculator;
class Task;

class WBLC{
public:
  WBLC();
  ~WBLC();

  bool MakeTorque(const vector<Task*> & task_array,
                  const vector<Task*> & task_array_last,
                  ReactionForceCalculator * rf_cal,
                  const sejong::Matrix & A,
                  const sejong::Matrix & Ainv,
                  const sejong::Vector & grav,
                  const sejong::Vector & coriolis,
                  Vector& gamma);

  //test for wrench
  bool MakeTorque(const vector<Task*> & task_array,
                  const vector<Task*> & task_array_last,
                  ContactWrenchCalculator * rf_cal,
                  const sejong::Matrix & A,
                  const sejong::Matrix & Ainv,
                  const sejong::Vector & grav,
                  const sejong::Vector & coriolis,
                  Vector& gamma);

private:
  void _PrintDebug(double i);
  sejong::Vector Fr_;
  Matrix U_;
  Matrix jac;
};


#endif
