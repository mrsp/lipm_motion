//  KWalkMath.h
//  Kouretes Walk Engine
//
//  Created by Stelios Piperakis on 8/14/13.
//  Copyright (c) 2013 SP. All rights reserved.
//
/* Mathematics Used in Walking */
/* More MATH will be added in the way */
#ifndef _KWALKMAT_
#define _KWALKMAT_
#include <math.h>
#include <vector>
class KWalkMat
{

public:

    void LinearInterpolation(float *buffer, int &counter, float end, float start, float Ts, float T)
    {
        //std::cout << counter<< "Ts " << Ts << " T " << T << std::endl;
        for (float t = 0; t <= 1; t += Ts / T, counter++)
        {
            buffer[counter] = (1 - t) * start + t * end; //    vec.push_back();

            //	std::cout << counter << " "<< buffer[counter] <<  std::endl;
        }
    }

    void CubicBezierInterpolation(float *buffer, int &counter, float end, float start, float Ts, float Tss)
    {
        /**
        float H=0.020 for max stepping height 0.015m
        **/

        float H = 0.02, P0 = start, P1 = start + H, P2 = end + H, P3 = end;
        for (float t = 0; t <= 1; t = t + Ts / Tss, counter++)
            buffer[counter] = (pow(1 - t, 3) * P0 + 3 * pow(1 - t, 2) * t * P1 + 3 * (1 - t) * pow(t, 2) * P2 + pow(t, 3) * P3);

    }


    void trigIntegInterpolation(float *buffer, int &counter, float end, float start, float Ts, float Tss)
    {
        float delta = (end-start)/Tss;

        for (float t = 0; t <= Tss; t = t + Ts , counter++)
            buffer[counter]  = start+delta*t-delta*sin((t*M_PI*2)/Tss)/((M_PI*2)/Tss);

    }

    float trigIntegInterpolation(float t,float end, float start, float Tss)
    {
        float delta = (end-start)/Tss;

        return  start+delta*t-delta*sin((t*M_PI*2)/Tss)/((M_PI*2)/Tss);

    }
    float LinearInterpolation(float currentstep,float end, float start, float steps)
    {
        currentstep/=steps;
        return  currentstep*end+(1.000-currentstep)*start;

    }
    float CubicSplineInterpolation(float tdot,float p0,float p1,float p2, float p3,float p4,float T)
    {
        float h=T/4.0;
        float res;
        float t;
        KMath::KMat::GenMatrix<float,1,7> C;
        KMath::KMat::GenMatrix<float,7,1> D;
        KMath::KMat::GenMatrix<float,5,1> B;
        KMath::KMat::GenMatrix<float,5,5> A;
        B(0)=p0;
        B(1)=6*p1;
        B(2)=6*p2;
        B(3)=6*p3;
        B(4)=p4;

        B.scalar_mult((1/h)*(1/h)*(1/h));

        A.zero();
        A(0,0)=1;
        A(1,0)=1;
        A(1,1)=4;
        A(1,2)=1;
        A(2,1)=1;
        A(2,2)=4;
        A(2,3)=1;
        A(3,2)=1;
        A(3,3)=4;
        A(3,4)=1;
        A(4,4)=1;

        A.fast_invert();
        B=A*B;

        C.zero();
        C(0,0)=2*B(0)-B(1) ;
        C(0,1)=B(0);
        C(0,2)=B(1);
        C(0,3)=B(2);
        C(0,4)=B(3);
        C(0,5)=B(4);
        C(0,6)=2*B(4)-B(3);

        int i=0;
        for(int j=-1; j<6; j++)
        {

            t=tdot-j*h;

            if (t<=-2*h)
            {
                D(i)=0;
            }
            else if ((t>-2*h) &&(t<=-h))
            {
                D(i)=(1.0/6.0)*(2*h+t)*(2*h+t)*(2*h+t);
            }
            else if((t>-h) && (t<=0))
            {
                D(i)=(2.0/3.0)*h*h*h-(1.0/2.0)*t*t*(2*h+t);
            }
            else if((t>0)&&(t<=h))
            {
                D(i)=(2.0/3.0)*h*h*h-(1.0/2.0)*t*t*(2*h-t);

            }
            else  if((t>h)&&(t<=2*h))
            {
                D(i)=(1.0/6.0)*(2*h-t)*(2*h-t)*(2*h-t);
            }
            else
            {
                D(i)=0;
            }

            i++;
        }


        return res=C*D;


    }

    void CubicSplineInterpolation(float *buffer,int &counter,float p0,float p1,float p2, float p3,float p4,float Ts,float T)
    {
        float h=T/4;
        float t;
        KMath::KMat::GenMatrix<float,1,7> C;
        KMath::KMat::GenMatrix<float,7,1> D;
        KMath::KMat::GenMatrix<float,5,1> B;
        KMath::KMat::GenMatrix<float,5,5> A;
        B(0)=p0;
        B(1)=6*p1;
        B(2)=6*p2;
        B(3)=6*p3;
        B(4)=p4;

        B.scalar_mult((1/h)*(1/h)*(1/h));

        A.zero();
        A(0,0)=1;
        A(1,0)=1;
        A(1,1)=4;
        A(1,2)=1;
        A(2,1)=1;
        A(2,2)=4;
        A(2,3)=1;
        A(3,2)=1;
        A(3,3)=4;
        A(3,4)=1;
        A(4,4)=1;

        A.fast_invert();
        B=A*B;

        C.zero();
        C(0,0)=2*B(0)-B(1) ;
        C(0,1)=B(0);
        C(0,2)=B(1);
        C(0,3)=B(2);
        C(0,4)=B(3);
        C(0,5)=B(4);
        C(0,6)=2*B(4)-B(3);

        for(float tdot=0; tdot<=T; tdot=tdot+Ts,counter++)
        {
            int i=0;
            for(int j=-1; j<6; j++)
            {

                t=tdot-j*h;

                if (t<=-2*h)
                {
                    D(i)=0;
                }
                else if ((t>-2*h) &&(t<=-h))
                {
                    D(i)=(1.0/6.0)*(2*h+t)*(2*h+t)*(2*h+t);
                }
                else if((t>-h) && (t<=0))
                {
                    D(i)=(2.0/3.0)*h*h*h-(1.0/2.0)*t*t*(2*h+t);
                }
                else if((t>0)&&(t<=h))
                {
                    D(i)=(2.0/3.0)*h*h*h-(1.0/2.0)*t*t*(2*h-t);

                }
                else  if((t>h)&&(t<=2*h))
                {
                    D(i)=(1.0/6.0)*(2*h-t)*(2*h-t)*(2*h-t);
                }
                else
                {
                    D(i)=0;
                }

                i++;
            }

            buffer[counter]=C*D;
        }
    }



    void LinearInterpolation(std::vector<float> &vec, float end, float start, float Ts, float T)
    {

        for (float t = 0; t <= 1; t += Ts / T)
        {
            vec.push_back((1 - t) * start + t * end);
        }

    }


    void CubicBezierInterpolation(std::vector<float> &vec, float end, float start, float Ts, float Tss)
    {
        /**
        float H=0.0207 for max stepping height 0.020m
        **/

        float H = 0.027, P0 = start, P1 = start + H, P2 = end + H, P3 = end;
        for (float t = 0; t <= 1; t = t + Ts / Tss)
        {
            vec.push_back(pow(1 - t, 3) * P0 + 3 * pow(1 - t, 2) * t * P1 + 3 * (1 - t) * pow(t, 2) * P2 + pow(t, 3) * P3);
        }

    }



    float Bezier(float p[],int end, int start, float t)
    {
        if(start==end)
            return p[start];
        else
            return (1.0-t)*Bezier(p,end-1,start,t)+t*Bezier(p,end,start+1,t);

    }

    float BezierZ(float t, float Height, float steps)
    {   
        float p[] = {0.0,Height/3.0,2.225*Height,Height/3,0.0};
        return Bezier(p,3,0,t/steps);
    }

    float BezierLinearInterpolation(float t,float end, float start, float Tss)
    {
        float p[]= {0, 0.01 , 0.99 , 1.0};
        return  Bezier(p,3,0,t/Tss)*(end-start)+start;

    }

    float QuadSplineInterpolation(float s,KMath::KMat::GenMatrix<float, 5, 1> alpha)
    {
        float sum=0;

        for(int i=0; i<5; i++)
        {
            sum+=alpha(i)*pow(s,(float)i);


        }


        return sum;
    }


    float LinearSplineInterpolation(float s,KMath::KMat::GenMatrix<float, 2, 1> alpha)
    {
        float sum=0;

        for(int i=0; i<2; i++)
        {
            sum+=alpha(i)*pow(s,(float) i);


        }



        return sum;
    }


    KMath::KMat::GenMatrix<float, 6, 1> quinticHermiteCoeff(float s)
    {
        float s3 = s*s*s;
        float s4 = s*s*s*s;
        float s5 = s*s*s*s*s;
        KMath::KMat::GenMatrix<float, 6, 1>  res;
        res(0) = 1 - 10*s3  + 15*s4 - 6*s5;
        res(1) = s - 6*s3 + 8*s4 - 3*s5;
        res(2) = 1/2 * s*s - 3/2 * s3 + 3/2 * s4 - 1/2*s5;
        res(3) = 1/2 * s3 - s4 + 1/2 * s5;
        res(4) = -4*s3 + 7 * s4 - 3*s5;
        res(5) = 10*s3 - 15*s4 + 6*s5;
        return res;
    }

    float evalQuinticHermiteSpline(KMath::KMat::GenMatrix<float, 6, 1>H, KMath::KMat::GenMatrix<float, 3, 1> xs, KMath::KMat::GenMatrix<float, 3, 1> xf)
    {
        return  H(0) *  xs(0) + H(1) * xs(1) + H(2)* xs(2) + H(3)* xf(2) + H(4)*xf(1) + H(5) * xf(0);
    }


    float quinticHermiteInterp(float t, KMath::KMat::GenMatrix<float, 3, 1> xs, KMath::KMat::GenMatrix<float, 3, 1> xf, float dt, float T)
    {
        float s = t/T*dt;
        KMath::KMat::GenMatrix<float, 6, 1>  H = quinticHermiteCoeff(s);

        return evalQuinticHermiteSpline(H,xs,xf);
    }

    float planFeetTrajectoryXY_(float currentstep, float p_max, float p_min, float N)
 {
            KMath::KMat::GenMatrix<float, 3, 1> xs;
        KMath::KMat::GenMatrix<float, 3, 1> xf;
        xs.zero();
        xf.zero();
        xs(0)=p_min;
        xf(0)=p_max;
        return quinticHermiteInterp(currentstep, xs, xf, 1.0,N);
}


    float planFeetTrajectoryXY(float currentstep, float p_max, float p_min, float N)
    {

        float s,res;
        float v=0.500*(p_max-p_min);
        float p0=0.33;
        float p1=0.66;
        float s_p =currentstep/N;

        //Acceleration phase
        if (currentstep <= p0*N)
        {
            KMath::KMat::GenMatrix<float, 5, 1> alpha;
            alpha(0)=p_min;
            alpha(1)=0.000;
            alpha(2)=0.000;
            alpha(3)=v;
            alpha(4)=-v/2.000;

            s=s_p/p0;

            res=QuadSplineInterpolation(s,alpha);

        }
        //Constant Speed Phase
        else if(currentstep<=p1*N)
        {

            KMath::KMat::GenMatrix<float, 2, 1> alpha;
            alpha(0)=p_min+v/2.00;
            alpha(1)=v;

            s=(s_p-p0)/(p1-p0);


            res=LinearSplineInterpolation(s,alpha);


        }
        //Decceleration Phase
        else
        {
            KMath::KMat::GenMatrix<float, 5, 1> alpha;
            alpha(0)=p_min+1.500*v;
            alpha(1)=v;
            alpha(2)=0.000;
            alpha(3)=-v;
            alpha(4)=v/2.000;

            s=(s_p-p1)/(1.000-p1);

            res=QuadSplineInterpolation(s,alpha);



        }


        return res;


    }
    float  planFeetTrajectoryZ(float currentstep, float p_max, float p_min, float steps)
    {

        float acc = -0.065;
        acc=0.00;
        KMath::KMat::GenMatrix<float, 3, 1> xs,xf,xf_;

        xs.zero();
        xf.zero();
        xf_.zero();
        //Starting point on the ground
        xs(0)=p_min;
        //Mid Point in the air
        xf(0)=p_max;
        xf(2)=acc;
        //Final Point on the ground
        xf_(0) = p_min;

        float t1 = floor(steps/2.0);
        float t2 = steps;
        if(currentstep<t1)
            return quinticHermiteInterp(currentstep,xs,xf,1.0,t1-1.0);
        else
            return quinticHermiteInterp((currentstep-t1),xf,xf_,1.0,(t2-t1)-1.0);
    }





    

};
#endif
