#include <lipm_motion/dcmDynamics.h>

dcmDynamics::dcmDynamics(RobotParameters &robot_):robot(robot_)
{
    //State is com, dcm, vrp
    x.setZero();
    A.setZero();
    B.setZero();
    I.setIdentity();
    A(0,0) = -robot.getWalkParameter(omega);
    A(0,1) = robot.getWalkParameter(omega);
    A(1,1) = robot.getWalkParameter(omega);
    A(1,2) = -robot.getWalkParameter(omega);
    A *= robot.getWalkParameter(Ts);
    A.noalias() += I;
    B.setZero();
    B(2) = 1.000;
    B *= robot.getWalkParameter(Ts);
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