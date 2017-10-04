#include "Contact.h"
#include <Polyhedron/Polyhedron.h>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>

/*
 *CONTACT PROPERTY
 */

ContactProperty::ContactProperty(double stat_coef, double dyn_coef){
    static_coefficient = stat_coef;
    dynamic_coefficient = dyn_coef;
}

ContactProperty::~ContactProperty(){
}


/*
 *CONTACT GEOMETRY
 */

ContactGeometry::ContactGeometry(sejong::Matrix TF, sejong::Vector _lwh){
    lwh = _lwh;
    cent_pos << TF(0,3), TF(1,3), TF(2,3);
    rot = TF.block(0, 0, 3, 3);
    htrans = TF;
}

ContactGeometry::~ContactGeometry(){
}


/*
 *CONTACT
 */

Contact::Contact(ContactProperty* Prop, ContactGeometry* Geom, int link_num):aprx_rate(1.414){
    ContactGeometry_ = Geom;
    ContactProperty_ = Prop;
    link_id_ = link_num;
    _SetVertices();
}

Contact::~Contact(){
    delete ContactGeometry_;
    delete ContactProperty_;
}

void Contact::_SetVertices() {
    sejong::Vector local_v1(4);
    sejong::Vector local_v2(4);
    sejong::Vector local_v3(4);
    sejong::Vector local_v4(4);

    local_v1 << ContactGeometry_->lwh[0], ContactGeometry_->lwh[1], -ContactGeometry_->lwh[2], 1;
    local_v2 << ContactGeometry_->lwh[0], -ContactGeometry_->lwh[1], -ContactGeometry_->lwh[2], 1;
    local_v3 << -ContactGeometry_->lwh[0], -ContactGeometry_->lwh[1], -ContactGeometry_->lwh[2], 1;
    local_v4 << -ContactGeometry_->lwh[0], ContactGeometry_->lwh[1], -ContactGeometry_->lwh[2], 1;

    vertices.resize(4);
    vertices[0] = (ContactGeometry_->htrans * local_v1).head(3);
    vertices[1] = (ContactGeometry_->htrans * local_v2).head(3);
    vertices[2] = (ContactGeometry_->htrans * local_v3).head(3);
    vertices[3] = (ContactGeometry_->htrans * local_v4).head(3);
}

sejong::Matrix Contact::GetGraspMatrix(sejong::Vect3 to){
    sejong::Vect3 p;
    p = ContactGeometry_->cent_pos - to;
    sejong::Matrix ret(6, 6);
    ret << 1, 0, 0  ,  0       , -p[2]  ,  p[1],
           0, 1, 0  ,  p[2]    ,  0     , -p[0],
           0, 0, 1  , -p[1]    ,  p[0]  ,  0   ,
           0, 0, 0  ,  1       ,  0     ,  0   ,
           0, 0, 0  ,  0       ,  1     ,  0   ,
           0, 0, 0  ,  0       ,  0     ,  1   ;

    return ret;
}

sejong::Matrix Contact::GetForceFace() {
    aprx_mu = ContactProperty_->static_coefficient / aprx_rate;
    sejong::Matrix face_local(4,3);
    face_local << -1,  0, -aprx_mu,
                   1,  0, -aprx_mu,
                   0, -1, -aprx_mu,
                   0,  1, -aprx_mu;
    return face_local * ContactGeometry_->rot.transpose();
}

std::vector<sejong::Vect3> Contact::GetForceRays() {
    aprx_mu = ContactProperty_->static_coefficient / aprx_rate;
    std::vector<sejong::Vect3> ret;
    ret.resize(4);
    sejong::Vect3 f_1;
    sejong::Vect3 f_2;
    sejong::Vect3 f_3;
    sejong::Vect3 f_4;

    f_1 << aprx_mu, aprx_mu, 1;
    f_2 << aprx_mu, -aprx_mu, 1;
    f_3 << -aprx_mu, aprx_mu, 1;
    f_4 << -aprx_mu, -aprx_mu, 1;

    ret[0] = ContactGeometry_->rot * f_1;
    ret[1] = ContactGeometry_->rot * f_2;
    ret[2] = ContactGeometry_->rot * f_3;
    ret[3] = ContactGeometry_->rot * f_4;

    return ret;
}

sejong::Matrix Contact::GetForceSpan(){
    sejong::Matrix ret(3, 4);
    std::vector<sejong::Vect3> force_rays;
    force_rays = GetForceRays();

    for (int i = 0; i < 4; ++i) {
        ret.block(0, i, 3, 1) = force_rays[i];
    }

    return ret;
}

sejong::Matrix Contact::GetWrenchFace() {
    double x(ContactGeometry_->lwh[0]);
    double y(ContactGeometry_->lwh[1]);
    aprx_mu = ContactProperty_->static_coefficient / aprx_rate;
    sejong::Matrix face_local(16, 6);
    sejong::Matrix aug_rot = sejong::Matrix::Zero(6, 6);

    face_local <<  0             , 0             , 0       ,-1        , 0        , -aprx_mu          ,
                   0             , 0             , 0       , 1        , 0        , -aprx_mu          ,
                   0             , 0             , 0       , 0        ,-1        , -aprx_mu          ,
                   0             , 0             , 0       , 0        , 1        , -aprx_mu          ,
                  -1             , 0             , 0       , 0        , 0        , -y                ,
                   1             , 0             , 0       , 0        , 0        , -y                ,
                   0             ,-1             , 0       , 0        , 0        , -x                ,
                   0             , 1             , 0       , 0        , 0        , -x                ,
                   aprx_mu       , aprx_mu       ,-1       ,-y        ,-x        ,-(x + y) * aprx_mu ,
                   aprx_mu       ,-aprx_mu       ,-1       ,-y        , x        ,-(x + y) * aprx_mu ,
                  -aprx_mu       , aprx_mu       ,-1       , y        ,-x        ,-(x + y) * aprx_mu ,
                  -aprx_mu       ,-aprx_mu       ,-1       , y        , x        ,-(x + y) * aprx_mu ,
                   aprx_mu       , aprx_mu       ,1        , y        , x        ,-(x + y) * aprx_mu ,
                   aprx_mu       ,-aprx_mu       ,1        , y        ,-x        ,-(x + y) * aprx_mu ,
                  -aprx_mu       , aprx_mu       ,1        ,-y        , x        ,-(x + y) * aprx_mu ,
                  -aprx_mu       ,-aprx_mu       ,1        ,-y        ,-x        ,-(x + y) * aprx_mu ;

    aug_rot.block(0, 0, 3, 3) = ContactGeometry_->rot.transpose();
    aug_rot.block(3, 3, 3, 3) = ContactGeometry_->rot.transpose();

    return face_local * aug_rot;
}

std::vector<sejong::Matrix> Contact::GetWrenchRays() {
    std::vector<sejong::Vect3> force_rays;
    force_rays = GetForceRays();
    std::vector<sejong::Matrix> ret;
    ret.resize(16);
    sejong::Matrix torque_force(3, 2);

    for (int i = 0; i < 4; ++i) {
        torque_force.setZero();
        for (int j = 0; j < 4; ++j) {
            torque_force.block(0, 0, 3, 1) = (vertices[i] - ContactGeometry_->cent_pos).cross( force_rays[j] );
            torque_force.block(0, 1, 3, 1) = force_rays[j];
            ret[4*i + j] = torque_force;
        }
    }

    return ret;
}

sejong::Matrix Contact::GetWrenchSpan() {
    sejong::Matrix force_span(3,4);
    sejong::Matrix muliplier(6,3);
    force_span = GetForceSpan();
    sejong::Matrix ret(6, 16);

    muliplier.block(3, 0, 3, 3) = sejong::Matrix::Identity(3,3);
    for (int i = 0; i < 4; ++i) {
        muliplier.block(0, 0, 3, 3) = sejong::crossmat(vertices[i] - ContactGeometry_->cent_pos);
        ret.block(0, 4*i, 6, 4) = muliplier * force_span;
    }

    return ret;
}

sejong::Matrix Contact::GetWrenchSpan(sejong::Vect3 pos) {
    sejong::Matrix force_span(3,4);
    sejong::Matrix muliplier(6,3);
    force_span = GetForceSpan();
    sejong::Matrix ret(6, 16);

    muliplier.block(3, 0, 3, 3) = sejong::Matrix::Identity(3,3);
    for (int i = 0; i < 4; ++i) {
        muliplier.block(0, 0, 3, 3) = sejong::crossmat(vertices[i] - pos);
        ret.block(0, 4*i, 6, 4) = muliplier * force_span;
    }

    return ret;
}

sejong::Matrix Contact::GetHrepDuality(sejong::Vect3 to) {
    sejong::Matrix ret = GetWrenchFace();
    sejong::Matrix h_at_contact_surf = GetWrenchFace();
    sejong::Vect3 go = ContactGeometry_->cent_pos - to;

    sejong::Vect3 a1, a2; //a1 : torque, a2 : force
    for (int i = 0; i < h_at_contact_surf.rows(); ++i) {
        a1.transpose() = h_at_contact_surf.block(i, 0, 1, 3);
        a2.transpose() = h_at_contact_surf.block(i, 3, 1, 3);
        ret.block(i, 3, 1, 3) = (a2 + go.cross(a1)).transpose();
    }

    return ret;
}

int Contact::GetLinkID() {
    return link_id_;
}

double Contact::GetStatFricCoef() {
    return ContactProperty_->static_coefficient;
}


/*
 *CONTACT HANDLER
 */
ContactHandler::ContactHandler(std::vector<Contact*> ct_list, sejong::Vect3 pos):g(9.81){
    num_contact_ = ct_list.size();
    std::cout << "[Contact Handler for " << num_contact_ <<" Points] Generated" << std::endl;
    poly = new Polyhedron();
    v_rep_ = sejong::Matrix::Zero(6, 16*num_contact_);
    contact_list = ct_list;
    aggregated_point_ = pos;
    _VrepAggregatedWrench();
    _HrepAggregatedWrench();
}

ContactHandler::~ContactHandler(){
}

void ContactHandler::_VrepAggregatedWrench() {
   for (int i = 0; i < num_contact_; ++i) {
       v_rep_.block(0, 16*i, 6, 16) = contact_list[i]->GetWrenchSpan(aggregated_point_);
   }
}

void ContactHandler::_HrepAggregatedWrench() {
    sejong::Vector vb_rep(16*num_contact_);
    vb_rep.setZero();
    Eigen::VectorXd hb_rep;

    auto hrep = poly->hrep(v_rep_.transpose(), vb_rep);
    hb_rep = hrep.second;
    h_rep_ = hrep.first;
}

sejong::Matrix ContactHandler::GetVrep() {
    return v_rep_;
}

Eigen::MatrixXd ContactHandler::GetHrep() {
    return h_rep_;
}

std::vector<sejong::Vect2> ContactHandler::GetStaticEquillibrium() {
    std::vector<sejong::Vect2> ret;

    _PreProcessforStaticEquillibrium();

    B_polar_stat_ = sejong::Matrix::Zero(h_rep_.rows(), 2);
    for (int i = 0; i < h_rep_.rows(); ++i) {
        B_polar_stat_(i, 0) = B_stat_(i, 0) / c_stat_[i];
        B_polar_stat_(i, 1) = B_stat_(i, 1) / c_stat_[i];
    }


    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;
    typedef boost::geometry::model::multi_point<point> multi_point;

    multi_point m_points;
    multi_point hull_points;

    for (int i = 0; i < B_polar_stat_.rows(); ++i) {
        m_points.push_back(point(B_polar_stat_(i, 0), B_polar_stat_(i, 1)));
    }
    m_points.push_back(point(0,0));

    boost::geometry::convex_hull(m_points, hull_points);

    std::vector<point>::iterator iter;
    for (iter = hull_points.begin() ; iter < hull_points.end()-1; ++iter) {
       ret.push_back(_GetInterSection(iter->get<0>(), iter->get<1>(), (iter+1)->get<0>(), (iter+1)->get<1>()));
    }

    for (int i = 0; i < ret.size(); ++i) {
        ret[i] += chebyshev_center_stat_.head(2);
    }

    return ret;
}

sejong::Vect2 ContactHandler::_GetInterSection(double a, double b, double c, double d) {
    sejong::Vect2 ret;
    if (std::abs(a) < 0.00001) {
        ret[1] = 1/b;
        ret[0] = ( 1 - d*ret[1] ) / c;
    } else {
        ret[1] = ( 1 - (c/a) ) / ( d - (b*c/a) );
        ret[0] = ( 1 - b*ret[1] ) / a;
    }
    return ret;
}

void ContactHandler::_PreProcessforStaticEquillibrium() {
    B_stat_ = sejong::Matrix::Zero(h_rep_.rows(), 2);
    c_stat_ = sejong::Vector::Zero(h_rep_.rows());

    for (int i = 0; i < h_rep_.rows(); ++i) {
        B_stat_(i, 0) = -h_rep_(i, 1);
        B_stat_(i, 1) = h_rep_(i, 0);
        c_stat_[i] = -h_rep_(i, 5);
    }

    chebyshev_center_stat_.setZero();
    for (int i = 0; i < c_stat_.size(); ++i) {
        if (c_stat_[i] < 0) {
            chebyshev_center_stat_ = poly->GetChebyshevCenter(B_stat_, c_stat_);
            c_stat_ = c_stat_ - B_stat_ * chebyshev_center_stat_.head(2);
            break;
        }
    }
}

//TODO : Verify
std::vector<sejong::Vect2> ContactHandler::Get3DCoMAccCone(sejong::Vect3 CoM_pos, double z_ddot) {
     std::vector<sejong::Vect2> ret_tilda;
     std::vector<sejong::Vect2> ret;

    _PreProcessfor3DCoMAccCone(CoM_pos);

    B_polar_comacc_ = sejong::Matrix::Zero(h_rep_.rows(), 2);
    for (int i = 0; i < h_rep_.rows(); ++i) {
        B_polar_comacc_(i, 0) = B_comacc_(i, 0) / c_comacc_[i];
        B_polar_comacc_(i, 1) = B_comacc_(i, 1) / c_comacc_[i];
    }


    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;
    typedef boost::geometry::model::multi_point<point> multi_point;

    multi_point m_points;
    multi_point hull_points;

    for (int i = 0; i < B_polar_comacc_.rows(); ++i) {
        m_points.push_back(point(B_polar_comacc_(i, 0), B_polar_comacc_(i, 1)));
    }
    m_points.push_back(point(0,0));

    boost::geometry::convex_hull(m_points, hull_points);

    std::vector<point>::iterator iter;
    for (iter = hull_points.begin() ; iter < hull_points.end()-1; ++iter) {
       ret_tilda.push_back(_GetInterSection(iter->get<0>(), iter->get<1>(), (iter+1)->get<0>(), (iter+1)->get<1>()));
    }

    ret.resize(ret_tilda.size());
    for (int i = 0; i < ret_tilda.size(); ++i) {
        ret[i] = (z_ddot + g) * ret_tilda[i];
        ret[i] += chebyshev_center_comacc_.head(2);
    }

    return ret;
}

void ContactHandler::_PreProcessfor3DCoMAccCone(sejong::Vect3 CoM_pos) {
    B_comacc_ = sejong::Matrix::Zero(h_rep_.rows(), 2);
    c_comacc_ = sejong::Vector::Zero(h_rep_.rows());
    sejong::Vect3 a0_plus_a_cross_pg;
    sejong::Vect3 trq;
    sejong::Vect3 frc;

    for (int i = 0; i < h_rep_.rows(); ++i) {
        trq.transpose() = h_rep_.block(i, 0, 1, 3);
        frc.transpose() = h_rep_.block(i, 3, 1, 3);
        a0_plus_a_cross_pg = trq.cross(CoM_pos) + frc;
        B_comacc_(i, 0) = a0_plus_a_cross_pg[0];
        B_comacc_(i, 1) = a0_plus_a_cross_pg[1];
        c_comacc_[i] = -a0_plus_a_cross_pg[2];
    }

    chebyshev_center_comacc_.setZero();
    for (int i = 0; i < c_comacc_.size(); ++i) {
        if (c_comacc_[i] < 0) {
            chebyshev_center_comacc_ = poly->GetChebyshevCenter(B_comacc_, c_comacc_);
            c_comacc_ = c_comacc_ - B_comacc_ * chebyshev_center_comacc_.head(2);
            break;
        }
    }
}

namespace WrenchConeProjection{

    sejong::Matrix GetHrepDuality(sejong::Matrix hrep, sejong::Vect3 from, sejong::Vect3 to) {
        sejong::Matrix ret = hrep;
        sejong::Vect3 go = from - to;

        sejong::Vect3 a1, a2; //a1 : torque, a2 : force
        for (int i = 0; i < hrep.rows(); ++i) {
            a1.transpose() = hrep.block(i, 0, 1, 3);
            a2.transpose()= hrep.block(i, 3, 1, 3);
            ret.block(i, 3, 1, 3) = (a2 + go.cross(a1)).transpose();
        }

        return ret;
    }

}
