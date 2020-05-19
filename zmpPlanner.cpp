#include "zmpPlanner.h"
zmpPlanner::zmpPlanner(RobotParameters &robot):NaoRobot(robot), ZMPbuffer(10*(int) NaoRobot.getWalkParameter(PreviewWindow)){//sa(robot){
    planAvailable = false;
    start.zero();
    target.zero();
    startL.zero();
    targetL.zero();
    startR.zero();
    targetR.zero();
    plantarget.zero();
    plantargetR.zero();
    plantargetL.zero();
    planstartL.zero();
    planstartR.zero();
    planned.targetZMP= SUPPORT_LEG_NONE;
    planned.targetSupport= SUPPORT_LEG_NONE;
    planned.step_id = -1;
    zmpi = planned;
    firstrun = true;
}

void zmpPlanner::setFeet(KVecFloat3 sl, KVecFloat3 sr){
    startL=sl;
    startR=sr;
    /** Double Support Phase **/
    KMath::KMat::GenMatrix<float,2,2> rot;
    float meanangle=anglemean(sl(2),sr(2));
    start=sl;
    /** planL,planR are the ankle positions, transforming them to Reference ZMP **/
    KMath::KMat::transformations::makeRotation(rot,(float) meanangle);
    KVecFloat2 rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),-NaoRobot.getWalkParameter(HY));
    start(0) += rr(0);
    start(1) += rr(1);
    start+=sr;
    rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),NaoRobot.getWalkParameter(HY));
    start(0) += rr(0);
    start(1) += rr(1);
    start.scalar_mult(0.5);
    start(2)=meanangle;
}



void zmpPlanner::generatePlan(KVecFloat2 DCM_, KVecFloat2 COP_, bool UseStepAdjustment){




    if(stepAnkleQ.size()==0){
        std::cout<<"Steps needed for motion planning"<<std::endl;
        return;
    }





    unsigned int j = 0;
    while (stepAnkleQ.size()>0){
        
        
        /** Robot is Ready to begin the walking Cycle **/
        WalkInstruction i = stepAnkleQ.front();
        /** Add a double support phase in the i instruction **/
        if(planned.step_id != i.step_id)
        {
            i.steps = NaoRobot.getWalkParameter(DS_instructions);
            i.phase = double_support;
        }
        else{ /**  Pop walking command i **/
            if(i.targetZMP ==  SUPPORT_LEG_BOTH)
                i.phase = double_support;
            else
                i.phase = single_support;
            
            stepAnkleQ.pop();
        }
        //Determine which Leg is swinging and compute the desired
        //2-D ZMP point
        //First two instructions DS + SS = 1 step
        if(j<2){

            //Only if StepAdjustment is true
            if(i.phase == single_support)
            {

                
                if(i.targetSupport ==  SUPPORT_LEG_RIGHT)
                {
          
                    //Check for Kinematic Bounds on steps
                    //Crop X axis
                    SFoot_rot.zero();
                    SFoot_angle = startR(2);
                    KMath::KMat::transformations::makeRotation(SFoot_rot, SFoot_angle);
                    SFoot_rot.prettyPrint();
                    dx =   KVecFloat2(i.target(0)-startR(0),i.target(1)-startR(1));
                    tempV = SFoot_rot.transp() * dx;
                    tempV(0) = cropStep(tempV(0),NaoRobot.getWalkParameter(MaxStepX),NaoRobot.getWalkParameter(MinStepX));
                    tempV(1) = cropStep(tempV(1),NaoRobot.getWalkParameter(MaxStepY),NaoRobot.getWalkParameter(MinStepY));
                    tempV = SFoot_rot * tempV;
                    i.target(0)= startR(0) + tempV(0);
                    i.target(1)= startR(1) + tempV(1); 
                    // if(UseStepAdjustment)
                    // {
                    //     //sa.solve((double) DCM_(0), (double) DCM_(1), (double) startR(0),
                    //     // (double) startR(1), (double) i.target(0), (double) i.target(1), (double) SFoot_angle, 1);
                    //     sa.solve((double) DCM_(0), (double) DCM_(1), (double) COP_(0),  (double) COP_(1), (double) startR(0), (double) startR(1),
                    //     (double) i.target(0), (double) i.target(1), (double) SFoot_angle, 1);
                    //     i.target(0)= sa.step_locationx;
                    //     i.target(1)= sa.step_locationy;
                    //     i.steps = (unsigned int) sa.step_instructions;
                    //     // dx =  KVecFloat2(i.target(0)-startR(0),i.target(1)-startR(1));
                    // }


                }
                else if(i.targetSupport== SUPPORT_LEG_LEFT)
                {
                   
                    // Check for Kinematic Bounds on steps
                    //Crop X axis
                    SFoot_rot.zero();
                    SFoot_angle = startL(2);
                    KMath::KMat::transformations::makeRotation(SFoot_rot, SFoot_angle);
                    SFoot_rot.prettyPrint();
                    dx =  KVecFloat2(i.target(0)-startL(0),i.target(1)-startL(1));
                    tempV = SFoot_rot.transp() * dx;
                    tempV(0) = cropStep(tempV(0),NaoRobot.getWalkParameter(MaxStepX),NaoRobot.getWalkParameter(MinStepX));
                    tempV(1) = cropStep(tempV(1),-NaoRobot.getWalkParameter(MinStepY),-NaoRobot.getWalkParameter(MaxStepY));
                    tempV = SFoot_rot * tempV;
                    i.target(0)= startL(0) + tempV(0);
                    i.target(1)= startL(1) + tempV(1); 
                    // if(UseStepAdjustment)
                    // {


                    //     //sa.solve((double) DCM_(0), (double) DCM_(1), 
                    //     //(double) startL(0), (double) startL(1), (double) i.target(0), (double) i.target(1), (double) SFoot_angle, 0);
                    //     sa.solve((double) DCM_(0), (double) DCM_(1), 
                    //         (double) COP_(0),  (double) COP_(1), (double) startL(0), (double) startL(1), (double) i.target(0), (double) i.target(1), (double) SFoot_angle, 0);
                        
                    //     i.target(0)= sa.step_locationx;
                    //     i.target(1)= sa.step_locationy;
                    //     i.steps = (unsigned int) sa.step_instructions;
                    //     //dx =  KVecFloat2(i.target(0)-startL(0),i.target(1)-startL(1));
                    // }

           
                }
            }


            computeSwingLegAndZMP(targetL,targetR, target, startL, startR, i);
            zmpi=i;
            zmpi.target = target;
            zmpQ.push(zmpi);
            //To execute
            walkInstructionbuffer.push(i);
            startL = targetL;
            startR = targetR;
            //For next plan
            planstartL = targetL;
            planstartR = targetR;
        }
        else{
            computeSwingLegAndZMP(plantargetL, plantargetR, plantarget, planstartL, planstartR, i);
            zmpi=i;
            zmpi.target = plantarget;
            zmpQ.push(zmpi);
            planstartL = plantargetL;
            planstartR = plantargetR;
        }
        planned = i;
        j++;
    }

}




void zmpPlanner::plan(KVecFloat2 DCM_,KVecFloat2 COP_,bool UseStepAdjustment)
{
    generatePlan(DCM_,COP_,UseStepAdjustment);
    planZMPTrajectory();
    planAvailable = true;
}



void zmpPlanner::planZMPTrajectory()
{
    
    KVecFloat3 ZMPref0,ZMPrefT, ZMPref;
    unsigned int j = 0, p;
    
    while(zmpQ.size()>0){
        WalkInstruction i = zmpQ.front();
        p=0;
        if(j<2){
            target = i.target;
            while(p<i.steps)
            {
                /** Angle between the ending and starting foot orientation **/
                float adiff = anglediff2(target(2),start(2));
                /** ZMP Trajectory Generation **/
                ZMPref(0)=interp.planFeetTrajectoryXY((float) p, target(0), start(0), i.steps-1.0);
                ZMPref(1)=interp.planFeetTrajectoryXY((float) p, target(1), start(1), i.steps-1.0);
                ZMPref(2)=start(2)+interp.LinearInterpolation( (float) p, adiff, 0.000,i.steps-1.0);
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                p++;
            }
            start = target;
            ZMPref0 = start;
        }
        else
        {
            ZMPrefT = i.target;
            while(p<i.steps)
            {
                /** Angle between the ending and starting foot orientation **/
                float adiff = anglediff2(ZMPrefT(2),ZMPref0(2));
                /** ZMP Trajectory Generation **/
                ZMPref(0)=interp.planFeetTrajectoryXY((float) p, ZMPrefT(0), ZMPref0(0), i.steps-1.0);
                ZMPref(1)=interp.planFeetTrajectoryXY((float) p, ZMPrefT(1), ZMPref0(1), i.steps-1.0);
                ZMPref(2)=ZMPref0(2)+interp.LinearInterpolation( (float) p, adiff, 0.000,i.steps-1.0);
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                p++;
            }
            ZMPref0=ZMPrefT;
        }
        zmpQ.pop();
        j++;
    }
}

void zmpPlanner::computeFeetCenterFromFeetAnkle(KVecFloat3& cl, KVecFloat3& cr, KVecFloat3 tl, KVecFloat3 tr)
{
        KMath::KMat::GenMatrix<float,2,2> rot;
        float rel_feet_angle =anglemean(tl(2),tr(2));
        KMath::KMat::transformations::makeRotation(rot,rel_feet_angle);
        KVecFloat2 rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),NaoRobot.getWalkParameter(HY));
        
        cr(0)  = tr(0) + rr(0);
        cr(1)  = tr(1) + rr(1);
        cr(2)  = tr(2);

        rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),-NaoRobot.getWalkParameter(HY));
        cl(0) = tl(0) + rr(0);
        cl(1) = tl(1) + rr(1);
        cl(2) = tl(2);
}

void zmpPlanner::computeFeetAnkleFromFeetCenter(KVecFloat3& al, KVecFloat3& ar, KVecFloat3 cl, KVecFloat3 cr)
{
        KMath::KMat::GenMatrix<float,2,2> rot;
        float rel_feet_angle =anglemean(cl(2),cr(2));
        KMath::KMat::transformations::makeRotation(rot,rel_feet_angle);
        KVecFloat2 rr = rot * KVecFloat2(NaoRobot.getWalkParameter(HX),-NaoRobot.getWalkParameter(HY));
        
        ar(0)  = cr(0) + rr(0);
        ar(1)  = cr(1) + rr(1);
        ar(2)  = cr(2);

        rr = rot * KVecFloat2(NaoRobot.getWalkParameter(HX),NaoRobot.getWalkParameter(HY));
        al(0) = cl(0) + rr(0);
        al(1) = cl(1) + rr(1);
        al(2) = cl(2);
}




void  zmpPlanner::computeSwingLegAndZMP(KVecFloat3 &tl, KVecFloat3 &tr, KVecFloat3 &t, KVecFloat3 sl,  KVecFloat3 sr, WalkInstruction i)
{
    
    if (i.targetZMP ==  SUPPORT_LEG_RIGHT){
        tl = i.target;
        tr = sr;
    }
    else if (i.targetZMP ==  SUPPORT_LEG_LEFT){
        tr = i.target;
        tl = sl;
    }
    else{
        //No Swing
        tl = sl;
        tr = sr;
    }
    
    
    /** Computing the  Reference ZMP point **/
    KMath::KMat::GenMatrix<float,2,2> rot;
    
    if(i.targetZMP== SUPPORT_LEG_RIGHT)
    {
        /** Right Support Phase **/
        t=tr;
        t(2)=anglemean(tl(2),tr(2));
        KMath::KMat::transformations::makeRotation(rot,(float)t(2));
        KVecFloat2 rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),NaoRobot.getWalkParameter(HY));
        
        t(0) += rr(0);
        t(1) += rr(1);
        
    }
    else if(i.targetZMP== SUPPORT_LEG_LEFT)
    {
        /** Left Support Phase **/
        t=tl;
        t(2)=anglemean(tl(2),tr(2));
        KMath::KMat::transformations::makeRotation(rot,(float)t(2));
        KVecFloat2 rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),-NaoRobot.getWalkParameter(HY));
        t(0) += rr(0);
        t(1) += rr(1);
    }
    else
    {
        /** Double Support Phase **/
        float meanangle=anglemean(tl(2),tr(2));
        t=tl;
        
        /** planL,planR are the ankle positions, transforming them to Reference ZMP **/
        KMath::KMat::transformations::makeRotation(rot,(float) meanangle);
        KVecFloat2 rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),-NaoRobot.getWalkParameter(HY));
        
        t(0) += rr(0);
        t(1) += rr(1);
        
        t+=tr;
        rr = rot * KVecFloat2(-NaoRobot.getWalkParameter(HX),NaoRobot.getWalkParameter(HY));
        
        t(0) += rr(0);
        t(1) += rr(1);
        t.scalar_mult(0.5);
        t(2)=meanangle;
    }
}


void zmpPlanner::emptyPlan(){
    

    while(stepAnkleQ.size()>0)
        stepAnkleQ.pop();


    while (zmpQ.size()>0)
        zmpQ.pop();
    
    while(ZMPbuffer.size()>0)
        ZMPbuffer.pop_front();
    
    planned.targetZMP= SUPPORT_LEG_NONE;
    planned.targetSupport= SUPPORT_LEG_NONE;
    planAvailable = false;
    planned.step_id = -1;
}


