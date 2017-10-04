#ifndef FOOT_FORCE_CALCULATOR
#define FOOT_FORCE_CALCULATOR

#include "utils/Sejong_Thread.h"
#include <utils/wrap_eigen.hpp>
#include <Configuration.h>

using namespace sejong;

class HumeModel;

class Foot_Force_Calculator: public Sejong_Thread{
public:
    Foot_Force_Calculator();
    virtual ~Foot_Force_Calculator(void);

    virtual void run(void);

    static void Cal_FootForce(SJLinkID _link_id, const Vector& torque, Vect3 & _force);
protected:
    sejong::Matrix U_;
    void Prepare_FootForce_Computation();

    HumeModel* model_;
};

#endif
