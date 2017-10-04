#ifndef CONSTRAINT_LIBRARY_H
#define CONSTRAINT_LIBRARY_H

#include <WBOSC/Constraint.hpp>

namespace sejong {
    class Hume_Fixed_Constraint: public WBC_Constraint{
    public:
        Hume_Fixed_Constraint():WBC_Constraint(){}
        virtual ~Hume_Fixed_Constraint(){}
        virtual bool updateJcU(const Vector & conf, WBOSC_Model * _model);

    };

    class Hume_RThigh_Fixed_Constraint: public WBC_Constraint{
    public:
        Hume_RThigh_Fixed_Constraint():WBC_Constraint(){}
        virtual ~Hume_RThigh_Fixed_Constraint(){}
        virtual bool updateJcU(const Vector & conf, WBOSC_Model * _model);
    };
    
    class Hume_No_Constraint : public WBC_Constraint{
    public:
        Hume_No_Constraint():WBC_Constraint(){}
        virtual ~Hume_No_Constraint(){}
        virtual bool updateJcU(const Vector & conf, WBOSC_Model * _model);
    };
    
    class Hume_Contact_Both : public WBC_Constraint
    {
    public:
        Hume_Contact_Both();
        virtual ~Hume_Contact_Both(){}
        virtual bool updateJcU(const Vector & conf, WBOSC_Model * _model);
        //////////////////////////////////////////////////////////////////////////
        void Update_Internal_Matrix(const Vector & q, WBOSC_Model* model);
        virtual void set_constraint_weight(const Vector & weight);
        virtual void getDesInternalForce(Vector & des_int_force);
    };
    class Hume_Point_Contact : public WBC_Constraint{
    public:
        Hume_Point_Contact(SJLinkID _foot);
        virtual ~Hume_Point_Contact(){}
        virtual bool updateJcU(const Vector & conf, WBOSC_Model* _model);
    public:
        SJLinkID foot_;
    };
    

    class Hume_Point_Contact_Left : public Hume_Point_Contact{
    public:
        Hume_Point_Contact_Left();
        virtual ~Hume_Point_Contact_Left(){}
    };

    class Hume_Point_Contact_Right : public Hume_Point_Contact{
    public:
        Hume_Point_Contact_Right();
        virtual ~Hume_Point_Contact_Right(){}
    };

}

#endif
