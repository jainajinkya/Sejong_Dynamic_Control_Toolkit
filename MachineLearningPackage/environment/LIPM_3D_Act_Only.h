#ifndef LIPM_3D_ACT_ONLY_H
#define LIPM_3D_ACT_ONLY_H

#include "LIPM_3D.h"

class LIPM_3D_Act_Only : public LIPM_3D{
public:
  LIPM_3D_Act_Only();
  virtual ~LIPM_3D_Act_Only();
  // Return True if nx state is Terminal
  virtual bool Transition(const sejong::Vector & state,
                          const sejong::Vector & action,
                          double & reward,
                          sejong::Vector & nx_state);
protected:
  double t_switch_;
  sejong::Vector vel_des_;
};

#endif
