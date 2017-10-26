#ifndef MERCURY_MODEL
#define MERCURY_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Mercury_Dyn_Model;
class Mercury_Kin_Model;

using namespace sejong;

class MercuryModel{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static MercuryModel* GetMercuryModel();
    virtual ~MercuryModel(void);

    bool getMassInertia(sejong::Matrix & A);

    virtual bool getInverseMassInertia(sejong::Matrix & Ainv) ;
    virtual bool getGravity(Vector & grav) ;
    virtual bool getCoriolis(Vector & coriolis) ;

    virtual void getCentroidJacobian(sejong::Matrix & Jcent);
    virtual void getCentroidInertia(sejong::Matrix & Icent);
    virtual void getPosition(const Vector & q,
                             int link_id, Vect3 & pos) ;
    virtual void getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const ;
    virtual void getCoMPosition(const Vector & q, Vect3 & com_pos, bool update= false);
    virtual void getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel);

    void getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & J) const ;
    void getVelocity(const Vector & q, const Vector &qdot,
                     int link_id, Vect3 & vel) ;

    void getCentroidVelocity(sejong::Vector & centroid_vel);
    void getCoMJacobian(const Vector & q, sejong::Matrix & J);
    void UpdateKinematics(const Vector & q, const Vector &qdot);
    void UpdateModel(const sejong::Vector & q, const sejong::Vector & qdot);

protected:
    Mercury_Dyn_Model* dyn_model_;
    Mercury_Kin_Model* kin_model_;

    RigidBodyDynamics::Model* model_;
private:
    MercuryModel();
};

#endif
