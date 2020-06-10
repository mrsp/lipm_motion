#include <lipm_motion/postureStabilizer.h>

postureStabilizer::postureStabilizer(RobotParameters &robot_) : robot(robot_)
{

    resetFootTorqueStabilizer();
    resetFootForceStabilizer();
    resetBaseOrientationStabilizer();
    dt = robot.getWalkParameter(Ts);
    Tc_ = robot.getWalkParameter(Tc);
    Kc_ = robot.getWalkParameter(Kc);
    Ta_ = robot.getWalkParameter(Ta);
    Ka_ = robot.getWalkParameter(Ka);
    Tn_ = robot.getWalkParameter(Tn);
    Kn_ = robot.getWalkParameter(Kn);
    std::cout << "Real-time Posture Stabilizer Initialized Successfully" << std::endl;
}

void postureStabilizer::resetFootTorqueStabilizer()
{
    dL_Roll = 0.0;
    dL_Pitch = 0.0;
    dR_Roll = 0.0;
    dR_Pitch = 0.0;
}

void postureStabilizer::footTorqueStabilizer(Vector3f tauld, Vector3f taurd, Vector3f taul, Vector3f taur)
{
    Ta_ = robot.getWalkParameter(Ta);
    Ka_ = robot.getWalkParameter(Ka);
    dt = robot.getWalkParameter(Ts);

    dL_Roll = Ka_ * dt * (tauld(0) - taul(0)) + (1.0 - dt / Ta_) * dL_Roll;
    dL_Pitch = Ka_ * dt * (tauld(1) - taul(1)) + (1.0 - dt / Ta_) * dL_Pitch;

    dR_Roll = Ka_ * dt * (taurd(0) - taur(0)) + (1.0 - dt / Ta_) * dR_Roll;
    dR_Pitch = Ka_ * dt * (taurd(1) - taur(1)) + (1.0 - dt / Ta_) * dR_Pitch;
}

void postureStabilizer::resetFootForceStabilizer()
{
    dz = 0.0;
}

void postureStabilizer::footForceStabilizer(float flz, float frz, float flz_d, float frz_d)
{
    Tn_ = robot.getWalkParameter(Tn);
    Kn_ = robot.getWalkParameter(Kn);
    dt = robot.getWalkParameter(Ts);
    float deltaF = flz - frz;
    float deltaF_d = flz_d - frz_d;

    dz = Kn_ * dt * (deltaF_d - deltaF) + (1.0 - dt / Tn_) * dz;

    dLz = -0.5 * dz;
    dRz = 0.5 * dz;
}

void postureStabilizer::resetBaseOrientationStabilizer()
{
    dbase_Roll = 0.0;
    dbase_Pitch = 0.0;
}

void postureStabilizer::baseOrientationStabilizer(float base_Roll, float base_Pitch, float base_Roll_d, float base_Pitch_d)
{
    Vector3f dbase;
    dbase.setZero();
    Tc_ = robot.getWalkParameter(Tc);
    Kc_ = robot.getWalkParameter(Kc);
    dt = robot.getWalkParameter(Ts);

    dbase_Roll = Kc_ * dt * (base_Roll_d - base_Roll) + (1.0 - dt / Tc_) * dbase_Roll;
    dbase_Pitch = Kc_ * dt * (base_Pitch_d - base_Pitch) + (1.0 - dt / Tc_) * dbase_Pitch;
}

Vector3f postureStabilizer::getBaseOrientation()
{

    return Vector3f(dbase_Roll, dbase_Pitch, 0);
}

Vector3f postureStabilizer::getLeftFootOrientation()
{

    return Vector3f(dL_Roll, dL_Pitch, 0);
}

Vector3f postureStabilizer::getRightFootOrientation()
{
    return Vector3f(dR_Roll, dR_Pitch, 0);
}

double postureStabilizer::getRightFootVerticalPosition()
{
    return dRz;
}

double postureStabilizer::getLeftFootVerticalPosition()
{
    return dLz;
}
