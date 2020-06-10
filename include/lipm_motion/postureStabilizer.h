#ifndef __POSTURESTABILIZER_H__
#define __POSTURESTABILIZER_H__
#include "RobotParameters.h"
#include "KMat.hpp"
#include <eigen3/Eigen/Dense>
//#include "ZMPDistributor.h"

using namespace Eigen;

class postureStabilizer
{
private:
    RobotParameters robot;

public:
    /** Ankle PD Control **/
    float Kc_, Tc_, Ka_, Ta_, Kn_, Tn_, dt;
    float dbase_Roll, dbase_Pitch, dL_Roll, dL_Pitch, dR_Roll, dR_Pitch, dLz, dRz, dz;

    postureStabilizer(RobotParameters &robot_);

    void resetFootTorqueStabilizer();
    void footTorqueStabilizer(Vector3f tauld, Vector3f taurd, Vector3f taul, Vector3f taur);
    void resetFootForceStabilizer();
    void footForceStabilizer(float flz, float frz, float flz_d, float frz_d);
    void resetBaseOrientationStabilizer();
    void baseOrientationStabilizer(float base_Roll, float base_Pitch, float base_Roll_d, float base_Pitch_d);
    Vector3f getBaseOrientation();
    Vector3f getLeftFootOrientation();
    Vector3f getRightFootOrientation();
    double getRightFootVerticalPosition();
    double getLeftFootVerticalPosition();
};
#endif
