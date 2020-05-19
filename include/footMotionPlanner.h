#ifndef  __FOOTMOTIONPLANNER_H__
#define  __FOOTMOTIONPLANNER_H__
#include "KMat.hpp"
#include "KWalkMat.h"
#include "RobotParameters.h"



class footMotionPlanner
{

private:

    RobotParameters &robot;
    KWalkMat interp;


public:
    KMath::KMat::GenMatrix<float, 3, 1> startL, planL, startR, planR;

    //Current Desired FootHold position xytheta
    KMath::KMat::GenMatrix<float, 3, 1> FootL, FootR;
    float startLz,startRz, planRz, planLz, FootLz, FootRz, StepZ_;

    footMotionPlanner(RobotParameters &robot_);
    void reset(); 
    void default_params();
    void setFootStartXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight);
    void setFootDestXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight);
    void setFootXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight);
    void setFootStartZ(float z, bool isRight);
    void setFootDestZ(float z, bool isRight);
    void setFootZ(float z, bool isRight);
    KVecFloat3 getFootDestXYTheta(bool isRight);
    float getFootDestTheta(bool isRight);
    float getFootDestZ(bool isRight);
    KVecFloat3 getFootXYTheta(bool isRight);
    float getFootTheta(bool isRight);
    float getFootZ(bool isRight);
    void MotionPlan(KVecFloat3 target, unsigned step, unsigned totalsteps, bool right_support, bool double_support);
};
#endif
