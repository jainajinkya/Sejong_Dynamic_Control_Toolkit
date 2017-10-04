#ifndef DYN_ENVIRONMENT_NAO
#define DYN_ENVIRONMENT_NAO

#include "nao.h"
#include "srDyn/srSystem.h"
#include "srDyn/srCollision.h"
#include "Ground.h"
#include "LieGroup/LieGroup.h"
#include <vector>
#include "utils/utilities.h"

////////////////////////////////////////////////
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

#ifdef __linux__
#include <GL/glut.h>
#endif
////////////////////////////////////////////////

class Interface;
class srSpace;
class Ground;
class srNao;

struct state
{
    std::vector<double> conf;
    std::vector<double> jvel;
    std::vector<double> torque   ;
    // std::vector<double> euler_ang;
    double ori_mtx[9];
    std::vector<double> ang_vel;
};

class NAO_Dyn_environment
{
    public:
        NAO_Dyn_environment();
        ~NAO_Dyn_environment();

        static void ContolFunction(void* _data);

        void Rendering_Fnc();

        void SetCurrentState_All();
        void saveLandingLocation();
    public:
        Interface* interface_;
        srNao* robot_;

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
        void _Get_Orientation(sejong::Quaternion & rot);

        void _Copy_Array(double * , double *, int);

        void _ListCommandedReactionForce(const sejong::Vector & Fr);
        void _Draw_Commanded_Force();
        void _Draw_Contact_Point();
        void _Draw_StatEqConvexHull_Point();
        void _Draw_CoM3DAcc_Point();
        void _Draw_CoM();
        void _Draw_CoMAcc();

};

#endif
