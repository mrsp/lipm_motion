#include <lipm_motion/dcmDynamics.h>

dcmDynamics::dcmDynamics()
{
    //State is com, dcm, vrp
    x.setZero();
    A.setZero();
    B.setZero();
    I.setIdentity();
}

void dcmDynamics::init()
{
    A(0,0) = -omega;
    A(0,1) = omega;
    A(1,1) = omega;
    A(1,2) = -omega;
    A *= dt;
    A.noalias() += I;
    B(2) = 1.000;
    B *= dt;
}

void dcmDynamics::setParams(double omega_, double dt_)
{
    omega = omega_;
    dt = dt_;
}


void dcmDynamics::setState(Vector3d x_)
{
    x = x_;
}



void dcmDynamics::integrate(double u_)
{
    x = A*x;
    x.noalias() +=  B*u_;
    com = x(0);
    dcm = x(1);
    vrp = x(2);
}

Vector3d dcmDynamics::getState()
{
    return x;
}