#include <lipm_motion/delayedObserverDCM.h>

delayedObserverDCM::delayedObserverDCM(RobotParameters &robot_):robot(robot_)
{

    
    //State is com, dcm, vrp, and ZMP offset
    x.setZero();
    A.setZero();
    B.setZero();
    I.setIdentity();
 
    
    Ccom.setZero();
    Czmp.setZero();

    Ccom(0,0) = 1.000;
	Czmp(0,2) = 1.000;
	Czmp(0,3) = 1.000;
    



    A(0,0) = -robot.getWalkParameter(omega);
    A(0,1) = robot.getWalkParameter(omega);
    A(1,1) = robot.getWalkParameter(omega);
    A(1,2) = -robot.getWalkParameter(omega);
    A *= robot.getWalkParameter(Ts);
    A.noalias() += I;
 
    B.setZero();
    B(2) = 1.000;
    B *= robot.getWalkParameter(Ts);


    L.setZero();
    L(0,0) =0.13494;
    L(0,1) =0.003825;
    L(1,0) =0.003129;
    L(1,1) =0.038694;
    L(2,0) =0.033215;
    L(2,1) =0.34279;
    L(3,0) =-0.052321;
    L(3,1) =0.26414;

    L=L/20.0;
    
   Lcom.setZero();
   Lcom  = L.block<4,1>(0,0);


    cout<<"ZMP Delayed Observer Initialized Successfully"<<endl;
    firstrun = true;

}

void delayedObserverDCM::setState(Vector4f x_)
{
    x = x_;
    updateVars();
    firstrun = false;
}



void delayedObserverDCM::update(float u_, float zmp_, float com_)
{
    
    if(xbuffer.size() > (int) ZMPDELAY - 1)
    {
        com_ -= Ccom*x;
        zmp_ -= Czmp*xbuffer.front();
        x = A*x;
        x.noalias() +=  B*u_;
        //x.noalias() +=  L * Vector2f(com_,zmp_);
        xbuffer.pop();        
    }
    else
    {
        com_ -= Ccom*x;
        x = A*x;
        x.noalias() +=  B*u_;
        //x.noalias() +=  Lcom * com_;
    }



    xbuffer.push(x);

    updateVars();
}



void delayedObserverDCM::updateVars()
{
    com = x(0);
    dcm = x(1);
    vrp = x(2);
    dist = x(3);
}

Vector4f delayedObserverDCM::getState()
{
    return x;
}