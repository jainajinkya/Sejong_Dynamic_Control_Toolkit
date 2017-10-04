#ifndef  DYN_ENVIRONMENT_Valkyrie
#define  DYN_ENVIRONMENT_Valkyrie

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>

//TEST JUNHYEOK
#include "new_valkyrie.h"

//TEST
////////////////////////////////////////////////
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

//#ifdef linux
#ifdef __linux__
#include <GL/glut.h>
#endif
////////////////////////////////////////////////


class ObstacleBuilder;
class Obstacle;
class Interface;
class srSpace;
class Ground;
class SR_Valkyrie;
class srBlockTerrain;
class sr3DImportSystem;


struct state
{
  std::vector<double> conf;
  std::vector<double> jvel;
  std::vector<double> torque   ;
  // std::vector<double> euler_ang;
  double ori_mtx[9];
  std::vector<double> ang_vel;
};

class Valkyrie_Dyn_environment
{
public:
  Valkyrie_Dyn_environment();
  ~Valkyrie_Dyn_environment();

  static void ContolFunction(void* _data);
  static void ContolFunction1(void* _data);
  static void _ObstacleControl(void* _data);

  void Rendering_Fnc();

  void SetCurrentState_All();
  void saveLandingLocation();
public:
  Interface* interface_;
  New_Valkyrie* new_robot_;
  //
  srBlockTerrain* terrain_;
  srBlockTerrain* room_;
  std::vector<sr3DImportSystem*> srObstacle_dyn_list_;
  std::vector<srBlockTerrain*> srObstacle_stat_list_;
  std::vector<Obstacle> obs_dyn_list_;
  ObstacleBuilder* obstacle_builder;

  srSpace*	m_Space;
  Ground*	m_ground;

  std::vector<double> curr_conf_;
  std::vector<double> curr_jvel_;
  std::vector<double> torque_   ;

  double ori_mtx_[9];
  std::vector<double> ang_vel_  ;

  std::vector<state> history_state_;
  std::vector<sejong::Vect3> contact_pt_list_;
  std::vector<sejong::Vect3> contact_force_list_;

  std::vector<sejong::Vect3> indicated_contact_pt_list_;
  std::vector<sejong::Vect3> commanded_contact_force_list_;

protected:
  double slope_;
  double loc_x_;
  void _SaveStanceFoot();
  void _ExternalDisturbance(int count);

  void _Save_Orientation_Matrix();
  void _Get_Orientation(sejong::Quaternion & rot);

  void _Copy_Array(double * , double *, int);

  void _ListReactionForce();
  void _ListCommandedReactionForce(const sejong::Vector & Fr);
  void _Draw_Contact_Point();
  void _Draw_Contact_Force();
  void _Draw_StatEqConvexHull_Point();
  void _Draw_CoM3DAcc_Point();
  void _Draw_CoM();
  void _Draw_CoMAcc();
  void _Draw_Commanded_Force();
  void _Draw_Path();
  void _DrawHollowCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);
  void _CheckFootContact();
  void _Draw_FootPlacement();
};

#endif
