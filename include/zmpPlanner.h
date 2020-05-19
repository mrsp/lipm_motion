#ifndef  __ZMPPLANER_H__
#define  __ZMPPLANER_H__

#include "KMat.hpp"
#include "KWalkMat.h"
#include "motionDefines.h"
#include "RobotParameters.h"
#include <queue>          // std::queue
#include <boost/circular_buffer.hpp>
#include <iostream>
//#include "StepAdjustment.h"

using namespace std;
class zmpPlanner
{
    
private:
    RobotParameters &NaoRobot;
    WalkInstruction planned, zmpi;
    KWalkMat interp;
    float maxX,maxY,minX,minY;
public:
    /** Walking Instruction to be executed **/
    
    std::queue<WalkInstruction> walkInstructionbuffer;
    KVecFloat3 target, start, startL, startR, targetR, targetL, plantargetL, plantargetR, plantarget, planstartL, planstartR;
    /** ZMP Buffer for the Prediction Horizon **/
    boost::circular_buffer<KVecFloat3> ZMPbuffer;
    /** Queues needed for planning **/
    std::queue<WalkInstruction> stepAnkleQ, zmpQ;
    bool planAvailable, firstrun;

    //Methods
    void planZMPTrajectory();
    void computeSwingLegAndZMP(KVecFloat3 &tl, KVecFloat3 &tr, KVecFloat3 &t, KVecFloat3 sl,  KVecFloat3 sr, WalkInstruction i);
    void generatePlan(KVecFloat2 DCM_, KVecFloat2 COP_,bool UseStepAdjustment);
    void emptyPlan();
    void plan(KVecFloat2 DCM_,KVecFloat2 COP_,bool UseStepAdjustment);
    zmpPlanner(RobotParameters &robot);
    //Step Adjustment
    //StepAdjustment sa;
    KVecFloat2 dx, tempV;
    KMath::KMat::GenMatrix<float,2,2>  SFoot_rot;
    float SFoot_angle;
    
    void computeFeetAnkleFromFeetCenter(KVecFloat3& al, KVecFloat3& ar, KVecFloat3 cl, KVecFloat3 cr);
    void computeFeetCenterFromFeetAnkle(KVecFloat3& cl, KVecFloat3& cr, KVecFloat3 tl, KVecFloat3 tr);
    void setFeet(KVecFloat3 sl, KVecFloat3 sr);
    /** @fn float anglemean(float l, float r) const
     *  @brief Computation of mean angle
    **/
   
    float anglemean(float l, float r) const
    {
        return (float) ((double) l + anglediff2( (double) r, (double) l) / 2.0);
    }
    float cropStep(float f_, float max_, float min_)
    {
        return fmax(min_, fmin(f_, max_));
    }

    inline static double wrapToPi(double angle) {
		while (angle > M_PI)
			angle -= 2.0 * M_PI;

		while (angle < -M_PI)
			angle += 2.0 * M_PI;

		return angle;
	}

	inline static double wrapTo2Pi(double angle) {
		while (angle > 2.0 * M_PI)
			angle -= 2.0 * M_PI;

		while (angle < -2.0 * M_PI)
			angle += 2.0 * M_PI;

		return angle;
	}

	inline static double wrapTo0_2Pi(double angle) {
		while (angle > 2.0 * M_PI)
			angle -= 2.0 * M_PI;

		while (angle < 0)
			angle += 2.0 * M_PI;

		return angle;
	}

	inline static double anglediff2(double a1, double a2) {
		return wrapToPi(wrapToPi(a1 + M_PI - a2) - M_PI);
	}

	inline static double anglediff(double a1, double a2) {
		return fabs(wrapTo0_2Pi(a1 + M_PI - a2) - M_PI);
	}
};
#endif
