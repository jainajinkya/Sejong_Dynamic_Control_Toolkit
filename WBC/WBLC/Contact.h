#include <Utils/utilities.h>

#ifndef CONTACT_H
#define CONTACT_H

class Polyhedron;

class ContactProperty
{
public:
    ContactProperty (double, double);
    virtual ~ContactProperty ();

    double static_coefficient;
    double dynamic_coefficient;

};

class ContactGeometry
{
    public:
        ContactGeometry (sejong::Matrix, sejong::Vector);
        virtual ~ContactGeometry ();

        sejong::Matrix htrans;
        sejong::Matrix rot;
        sejong::Vector lwh;
        sejong::Vect3 cent_pos;
};

class Contact
{
public:
    Contact(ContactProperty*, ContactGeometry*, int);
    virtual ~Contact();

    /*
     *++, +-, --, -+ in global coordinate
     */
    std::vector<sejong::Vect3> vertices;

    /*
     *Return grasp matrix which convert a local wrench(glob) to another wrench(glob) at point P
     *Usage : Wr_p = Grsp * Wr_local
     */
    sejong::Matrix GetGraspMatrix(sejong::Vect3);

    /*
     *H-representation of force friction cone in global coordinate
     *Matrix (4, 3)
     *Usage : H-rep * force < 0
     */
    sejong::Matrix GetForceFace();

    //V-representation of force friction cone in global coordinate
    std::vector<sejong::Vect3> GetForceRays();

    /*
     *Matrix (3, 4) : [f1, f2, f3, f4]
     *Usage : [f1, f2, f3, f4] * [a; b; c; d]
     */
    sejong::Matrix GetForceSpan();

    /*
     *H-representation of wrench friction cone taken at contact point (represented in global coordinates)
     *Matrix (16, 6)
     *Usage : H-rep * wrench < 0
     */
    sejong::Matrix GetWrenchFace();

    //V-representation of wrench friction cone in global coordinates taken at center of contact
    std::vector<sejong::Matrix> GetWrenchRays();

    /*
     *Matrix (6, 16)
     *Usage : w_P = S * lamda
     */
    sejong::Matrix GetWrenchSpan(); //span taken at contact point
    sejong::Matrix GetWrenchSpan(sejong::Vect3 pos); //span taken at specific position

    sejong::Matrix GetHrepDuality(sejong::Vect3);

    int GetLinkID();
    double GetStatFricCoef();


private:
    ContactGeometry* ContactGeometry_;
    ContactProperty* ContactProperty_;

    void _SetVertices();

    int link_id_;
    double aprx_rate;
    double aprx_mu;
};

class ContactHandler
{
public:
    ContactHandler (std::vector<Contact*> contact_list, sejong::Vect3 pos);
    virtual ~ContactHandler ();

    sejong::Matrix GetVrep();
    Eigen::MatrixXd GetHrep();
    std::vector<sejong::Vect2> GetStaticEquillibrium();
    std::vector<sejong::Vect2> Get3DCoMAccCone(sejong::Vect3 CoM_pos, double z_ddot);

    std::vector<Contact*> contact_list;
private:
    Polyhedron * poly;

    void _VrepAggregatedWrench(); //aggregated span taken at specific position
    void _HrepAggregatedWrench(); //aggregated face taken at specific position
    void _PreProcessforStaticEquillibrium(); //calculate B,c
    void _PreProcessfor3DCoMAccCone(sejong::Vect3 CoM_pos); //calculate B,c
    sejong::Vect2 _GetInterSection(double a, double b, double c, double d); // get intersection of ax+by=1, cx+dy=1

    int num_contact_;
    sejong::Vect3 aggregated_point_;

    sejong::Matrix v_rep_;
    Eigen::MatrixXd h_rep_;
    sejong::Matrix B_stat_;
    sejong::Vector c_stat_;
    sejong::Vect3 chebyshev_center_stat_;
    sejong::Matrix B_polar_stat_;

    sejong::Matrix B_comacc_;
    sejong::Vector c_comacc_;
    sejong::Vect3 chebyshev_center_comacc_;
    sejong::Matrix B_polar_comacc_;

    double g;
};

namespace WrenchConeProjection
{
    sejong::Matrix GetHrepDuality(sejong::Matrix hrep, sejong::Vect3 from, sejong::Vect3 to);
}
#endif /* CONTACT_H */
