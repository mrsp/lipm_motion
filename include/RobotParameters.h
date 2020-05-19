#ifndef ROBOTPARAMETERS_H_
#define ROBOTPARAMETERS_H_
#include <string>
#include <iostream>
#include <cmath>
#include <stdio.h>

#define MAX_TRAJECTORY_LENGTH 250

enum
{
    StepX=0, StepY, StepZ, H0, HX, HY, ComZ, Tstep,Tinit, Tds, Tss, Ts, Td, Init_instructions, SS_instructions, DS_instructions, MaxStepX, MaxStepY, MaxStepTheta, MinStepX, MinStepY, MinStepTheta, g,
    mass,omega,Early_Contact_threshold, Ground_Contact_threshold,LegUpThres,LegLowThres, CoM_Stiffness, CoM_Damping, PreviewWindow,
    StepPlanSize, com_q, comd_q, fd_q, com_r, comdd_r, I_xx, I_yy, I_zz, bias_fx, bias_fy, bias_fz, transitionSI_instructions,
    CoM_state_uncertainty, DCM_state_uncertainty, VRP_state_uncertainty, External_wrench_uncertainty,  CoM_noise, COP_Noise, StepXF, StepXH, StepYL, StepYR, 
    Tc, Ta, Tn, Kc, Ka, Kn, Tss_min, Tss_max, StepPlanAdjustment, velocityControl, alpha_ZMPFilter, ParametersSize
};

static const char * robotParameterNames[] =
{
    "StepX", "StepY", "StepZ", "H0", "HX", "HY", "ComZ", "Tstep","Tinit", "Tds", "Tss", "Ts", "Td", "Init_instructions", "SS_instructions", "DS_instructions", "MaxStepX", "MaxStepY", "MaxStepTheta", "MinStepX", "MinStepY", "MinStepTheta", "g", "mass","Early_Contact_threshold", 
    "Ground_Contact_threshold","LegUpThres","LegLowThres", "CoM_Stiffness", "CoM_Damping", "PreviewWindow",
    "StepPlanSize" "com_q" "comd_q" "fd_q" "com_r" "comdd_r" "I_xx" "I_yy" "I_zz" "bias_fx" "bias_fy" "bias_fz"  "transitionSI_instructions"
    "CoM_state_uncertainty" "DCM_state_uncertainty" "VRP_state_uncertainty" "External_wrench_uncertainty"  "CoM_noise" "COP_Noise" "StepXF" "StepXH" "StepYL" "StepYR"
    "Tc" "Ta" "Tn" "Kc" "Ka" "Kn" "Tss_min" "Tss_max" "StepPlanAdjustment" "velocityControl" "alpha_ZMPFilter" "ParametersSize"
};


static const char defaultFilenameForParameters[]="/home/nao/KWalkRobotParameters.ini";


class RobotParameters
{
    
private:
    float WalkParameters[ParametersSize];
    char robotName[256];
    
public:
    RobotParameters()
    {
        snprintf(robotName,256,"default");
        WalkParameters[Ts] = 0.01;
        WalkParameters[Td] = 0.05;
        WalkParameters[StepX] = 0.165;
        WalkParameters[StepY] = 0.085;
        WalkParameters[StepZ] = 0.0175;
        WalkParameters[H0] = 0.095/2.00;
        WalkParameters[HY] = 0.01;   
        WalkParameters[HX] = -0.025; 
        WalkParameters[ComZ] = 0.26;
        WalkParameters[Tstep] = 0.4; //0.28
        WalkParameters[Tinit]= 0.4;
        WalkParameters[Tss] = 0.3; //0.3
        WalkParameters[Tds] = 0.1; //0.05
        WalkParameters[MaxStepX] = 0.025;
        WalkParameters[MaxStepY] = 2.0*WalkParameters[H0] + 0.01;
        WalkParameters[MaxStepTheta] = 0.349065850401537;
        WalkParameters[MinStepX] = -0.010;
        WalkParameters[MinStepY] =  2.0*WalkParameters[H0];
        WalkParameters[MinStepTheta] = -0.050614548300834;
        WalkParameters[g] = 9.80665;
        WalkParameters[mass]= 5.182530 + 0.200; //Weight + LIDAR
        WalkParameters[omega]=  sqrt(WalkParameters[g]/WalkParameters[ComZ]);
        WalkParameters[Init_instructions]=ceil(WalkParameters[Tinit]/WalkParameters[Ts]);
        WalkParameters[SS_instructions]=ceil(WalkParameters[Tss]/WalkParameters[Ts]);
        WalkParameters[DS_instructions]=ceil(WalkParameters[Tds]/WalkParameters[Ts]);
        WalkParameters[Early_Contact_threshold]=80.0f;
        WalkParameters[Ground_Contact_threshold]=12.0f;
        WalkParameters[LegUpThres] = 22.0f;
        WalkParameters[LegLowThres] = 0.1f;
        WalkParameters[CoM_Stiffness] = 22000.0f;
        WalkParameters[CoM_Damping] = 1100.0f;
        WalkParameters[PreviewWindow] = 150;
        WalkParameters[StepPlanSize] = 30;
        WalkParameters[com_q] = 5.0e-02;
        WalkParameters[comd_q] = 1.5;
        WalkParameters[fd_q] = 5.0;
        WalkParameters[com_r] = 5.0e-04;
        WalkParameters[comdd_r] = 0.015;
        WalkParameters[I_xx] = 0.0050623407587;
        WalkParameters[I_yy] = 0.0048801358789;
        WalkParameters[I_zz] = 0.001610300038;
        WalkParameters[bias_fx] = 7.35705;
        WalkParameters[bias_fy] = -4.37976;
        WalkParameters[bias_fz] = 6.28909;
        WalkParameters[CoM_state_uncertainty] = 1.0e-2;
        WalkParameters[DCM_state_uncertainty] = 1.0e-1;
        WalkParameters[VRP_state_uncertainty] = 1.0e-1;
        WalkParameters[External_wrench_uncertainty] = 5.0e-1;
        WalkParameters[CoM_noise] = 2.0e-3;
        WalkParameters[COP_Noise] = 1.0e-1;        
        WalkParameters[StepXF] = 0.07;
        WalkParameters[StepXH] = -0.03;
        WalkParameters[StepYL] = 0.02;
        WalkParameters[StepYR] = -0.02;
        WalkParameters[transitionSI_instructions] = 250;
        WalkParameters[Tc] = 0.1; 
        WalkParameters[Ta] = 0.1;  //0.1
        WalkParameters[Tn] = 0.1; //0.1
        WalkParameters[Kc] = 2.5;   //5.0
        WalkParameters[Ka] = 0.05; //0.05
        WalkParameters[Kn] = 0.005; //0.005
        WalkParameters[Tss_min] = 0.28;
        WalkParameters[Tss_max] = 0.35;
        WalkParameters[StepPlanAdjustment] = false;
        WalkParameters[velocityControl] = false;
        WalkParameters[alpha_ZMPFilter] = 0.35;
    }
    
    float getWalkParameter(int);
    void  setWalkParameter(int p, float s);
    int writeWalkParametersFromFile(const char * filename);
    int readWalkParametersFromFile(const char * filename);
    void printWalkParameters();
};
#endif
