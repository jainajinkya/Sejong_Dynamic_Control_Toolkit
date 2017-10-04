#ifndef VALKYRIE_DYN_MODEL
#define VALKYRIE_DYN_MODEL

#include <Utils/wrap_eigen.hpp>
#include <rbdl/rbdl.h>

using namespace sejong;

class Valkyrie_Dyn_Model{
public:

    Valkyrie_Dyn_Model(RigidBodyDynamics::Model* model);
    ~Valkyrie_Dyn_Model(void);

    bool getMassInertia(Matrix & a);
    bool getInverseMassInertia(Matrix & ainv);
    bool getGravity(Vector &  grav);
    bool getCoriolis(Vector & coriolis);

    void UpdateDynamics(const sejong::Vector & q, const sejong::Vector & qdot);

protected:
    Matrix A_;
    Matrix Ainv_;
    Vector grav_;
    Vector coriolis_;

    RigidBodyDynamics::Model* model_;
};

#endif
