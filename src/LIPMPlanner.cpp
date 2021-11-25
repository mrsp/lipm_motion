#include <lipm_motion/LIPMPlanner.h>

LIPMPlanner::LIPMPlanner(int bsize):  CoMBuffer(bsize), DCMBuffer(bsize), VRPBuffer(bsize)
{

    A.setZero();
    B.setZero();
    C.setZero();
	//State is com, vcom, acom, ZMP u is com jerk
    x.setZero();
    y.setZero();
    x_.setZero();
    y_.setZero();
    dcmx_d = 0;
    comx_d = 0;
    comdx_d = 0;
    dcmdx_d = 0;
    ZMPx_d = 0;
    dcmy_d = 0;
    comy_d = 0;
    comdy_d = 0;
    dcmdy_d = 0;
    ZMPy_d = 0;
    u_x = 0;
    u_y = 0;
    CoM_d.resize(9);
    DCM_d.resize(6);
    ZMP_d.resize(3);

}

void LIPMPlanner::setParams(double comZ_, double g_, double dt_, double q_, double r_, int Np_)
{
    comZ= comZ_;
    g = g_;
    omega = sqrt(g/comZ);
    dt = dt_;
    Np = Np_; 
    rv = r_; 
    qv = q_;

    LIPMDynamicsX.setParams(omega,dt);
    LIPMDynamicsY.setParams(omega,dt);
}


void LIPMPlanner::init()
{


    //ZMP
    Fv.resize(Np,4);
    Fv.setZero();
    Fvu.resize(Np,Np);
    Fvu.setZero();


    tmpb.resize(1,Np-1);
    temp.resize(4);

    R.resize(Np,Np);
    Qv.resize(Np,Np);
    H.resize(Np,Np);
    H_inv.resize(Np,Np);
    H.setZero();
    I.resize(Np,Np);
    I.setIdentity();
    U_x.resize(Np);
    U_y.resize(Np);
    Gx.resize(Np,4);
    Gp.resize(Np,Np);
    ZMPRefX.resize(Np,1);  
    ZMPRefY.resize(Np,1);  
    ZMPRefX.setZero();
    ZMPRefY.setZero();


    LIPMDynamicsX.init();
    LIPMDynamicsY.init();

    //Embedded Integrator DCM ZMP
    A = LIPMDynamicsX.Ad;
    B = LIPMDynamicsX.Bd;
    C = LIPMDynamicsX.Cd;



    Fv.block(0,0,1,4)=C.transpose()*A;
    Fvu(0,0) = C.transpose()*B;
    temp = B;

    for (unsigned int i = 1; i < Np; i++)
    {
        Fv.block(i,0,1,4) =  Fv.block(i-1,0,1,4) * A;
	    tmpb = Fvu.block(i-1,0,1,Np-1);
        Fvu.block(i,1,1,Np-1) = tmpb;
        temp = A*temp;
        Fvu(i,0) =  C.transpose()*temp;
    }


    R = I * rv; 
    Qv = I * qv;

    //Hessian Matrix
    H = R*Qv.llt().solve(I);
    H.noalias() += Fvu.transpose()*Fvu;

    
    //Make Symmetric
    //H = (H+H.transpose())/2.0;
    //Compute the Gains
    H_inv = H.llt().solve(I);
    Gp = -H_inv* Fvu.transpose();
    Gx = Gp*Fv;

    planAvailable = false;
}


void LIPMPlanner::setState(Vector2d CoM, Vector2d vCoM, Vector2d ZMP)
{

    u_x = 0.0;
    u_y = 0.0;
    LIPMDynamicsX.setState(Vector4d(CoM(0),vCoM(0),0, ZMP(0)));
    LIPMDynamicsY.setState(Vector4d(CoM(1),vCoM(1),0, ZMP(1)));
}



void LIPMPlanner::plan(boost::circular_buffer<VectorXd> & ZMPRef)
{


  while(ZMPRef.size()>0)
  {

    for (unsigned int i = 0; i < Np; i++)
    {
        if (i+1 < ZMPRef.size())
        {
            ZMPRefX(i) = ZMPRef[i+1](0);
            ZMPRefY(i) = ZMPRef[i+1](1);
        }
        else
        {
            ZMPRefX(i) = ZMPRef[ZMPRef.size() - 1](0);
            ZMPRefY(i) = ZMPRef[ZMPRef.size() - 1](1);

        }
     }

    x = LIPMDynamicsX.getState();
    y = LIPMDynamicsY.getState();
	//Optimal MPC Law

    U_x = Gx*x - Gp *ZMPRefX;
    U_y = Gx*y - Gp *ZMPRefY;

    u_x =  U_x(0);
    u_y =  U_y(0);


	//LIPM Dynamics
	x_ = x;
	y_ = y;  
	LIPMDynamicsX.integrate(u_x);
    LIPMDynamicsY.integrate(u_y);
    x =  LIPMDynamicsX.getState();
    y =  LIPMDynamicsY.getState();
    
	//Desired Gait Pattern Reference
	comx_d = x(0);
	comy_d = y(0);
    comdx_d = x(1);
    comdy_d = y(1);
   	ZMPx_d = x(3);
    ZMPy_d = y(3);




	dcmx_d = comx_d + 1/omega * comdx_d;
	dcmy_d = comy_d + 1/omega * comdy_d;

	comddx_d = omega*omega*(comx_d - ZMPx_d);
	comddy_d = omega*omega*(comy_d - ZMPy_d);
	dcmdx_d = comdx_d + 1/omega * comddx_d;
	dcmdy_d = comdy_d + 1/omega * comddy_d;


    CoM_d(0) = comx_d;
    CoM_d(1) = comy_d;
    CoM_d(2) = comZ + ZMPRef.front()(2);
    
    CoM_d(3) = comdx_d;
    CoM_d(4) = comdy_d;
    CoM_d(5) = 0.0;

    CoM_d(6) = comddx_d;
    CoM_d(7) = comddy_d;
    CoM_d(8) = 0.0;

    DCM_d(0) = dcmx_d;
    DCM_d(1) = dcmy_d;
    DCM_d(2) = CoM_d(2);

    DCM_d(3) = dcmdx_d;
    DCM_d(4) = dcmdy_d;
    DCM_d(5) = 0.0;


    ZMP_d(0) = ZMPx_d;
    ZMP_d(1) = ZMPy_d;
    ZMP_d(2) = ZMPRef.front()(2);

    CoMBuffer.push_back(CoM_d);
    DCMBuffer.push_back(DCM_d);
    VRPBuffer.push_back(ZMP_d);

    ZMPRef.pop_front();

  }
    planAvailable = true;
}

void LIPMPlanner::emptyPlan()
{
    if(!planAvailable)
        return;
     while (CoMBuffer.size() > 0)
        CoMBuffer.pop_front();
    
     while (DCMBuffer.size() > 0)
        DCMBuffer.pop_front();
    
     while (VRPBuffer.size() > 0)
        VRPBuffer.pop_front();
}