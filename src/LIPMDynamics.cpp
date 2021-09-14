#include <lipm_motion/LIPMDynamics.h>

LIPMDynamics::LIPMDynamics()
{
    //State is com, vcom, acom, zmp, input u is com jerk
    x.setZero();
    A.setZero();
    B.setZero();
    C.setZero();
    I.setIdentity();

    Ad.setZero();
    Bd.setZero();
    Cd.setZero();
}

void LIPMDynamics::init()
{
    A(0,1) = 1;
    A(1,2) = 1; 
    A(3,1) = 1;
    Ad = A;
    Ad *= dt;
    Ad.noalias() += I;
    B(2) = 1.000;
    B(3) = -1/(omega*omega);
    Bd = B;
    Bd *= dt;
    C(3) = 1.00;
    Cd = C;
}

void LIPMDynamics::setParams(double omega_, double dt_)
{
    omega = omega_;
    dt = dt_;
}


void LIPMDynamics::setState(Vector4d x_)
{
    x = x_;
}



void LIPMDynamics::integrate(double u_)
{
    x = Ad*x;
    x.noalias() +=  Bd*u_;
    com = x(0);
    vcom = x(1);
    acom = x(2);
    zmp = x(3);
}

Vector4d LIPMDynamics::getState()
{
    return x;
}