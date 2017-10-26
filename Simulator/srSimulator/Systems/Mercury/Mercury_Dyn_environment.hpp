#ifndef		MERCURY_DYN_ENVIRONMENT
#define		MERCURY_DYN_ENVIRONMENT

#include "Mercury.hpp"
#include "srDyn/srSystem.h"
#include "srDyn/srCollision.h"
#include "Ground.h"
#include <vector>
#include "Configuration.h"

class Mercury_Dyn_environment
{
public:
  Mercury_Dyn_environment();
  ~Mercury_Dyn_environment();

  static void ContolFunction(void* _data);

  void Rendering_Fnc();

  Mercury*	m_Mercury;
  srSpace*	m_Space;
  Ground*	m_ground;

};

#endif
