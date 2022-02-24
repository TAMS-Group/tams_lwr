//  ---------------------- Doxygen info ----------------------
//! \file move_to_candle.cpp
//!
//! \brief
//! Implementation file for performing a joint space motion to
//! the candle position of the robot
//!
//! \details
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
/** move_to_candle.cpp - move the KuKA lwr to the upright (candle)
 *  position using ROS-FRI.
 *
 *  fnh: 2017.01.09 - update location of init file, reduce vel/acc
 *  fnh: 2016.06.30 - modified to use Reflexxes-TypeII API.
 */

//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date October 2013
//!
//! \version 1.0.1
//!
//!  \author Torsten Kroeger, tkr@stanford.edu
//!
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#include <Console.h>
#include <errno.h>
#include <string.h>
#include <LinuxAbstraction.h>
#include <FastResearchInterface.h>
#include <FastResearchInterfaceTest.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A) ((A)*PI / 180.0)
#endif

#ifndef DEG
#define DEG(A) ((A)*180.0 / PI)
#endif

#define NUMBER_OF_JOINTS 7

// ****************************************************************
// MoveToCandle()
//
int main(int argc, char* argv[])
{
    FastResearchInterface* FRI;
    unsigned int ControlScheme = FastResearchInterface::JOINT_POSITION_CONTROL;
    FRI = new FastResearchInterface("/opt/FRILibrary/etc/980039-FRI-Driver.init");
    FRI->StartRobot(ControlScheme);

    float JointStiffnessValues[LBR_MNJ], JointDampingValues[LBR_MNJ], CartStiffnessValues[FRI_CART_VEC],
        CartDampingValues[FRI_CART_VEC];

    for (int i = 0; i < LBR_MNJ; i++)
    {
        JointStiffnessValues[i] = (float)10.0;
        JointDampingValues[i] = (float)0.7;
    }

    for (int i = 0; i < FRI_CART_VEC; i++)
    {
        CartStiffnessValues[i] = (float)10.0;
        CartDampingValues[i] = (float)0.7;
    }

    FRI->SetCommandedCartDamping(CartStiffnessValues);
    FRI->SetCommandedCartStiffness(CartDampingValues);
    FRI->SetCommandedJointDamping(JointDampingValues);
    FRI->SetCommandedJointStiffness(JointStiffnessValues);

    unsigned int i = 0;
    int ResultValue = 0;
    float JointValuesInRad[NUMBER_OF_JOINTS];
    double CycleTime = 0.01;  // fnh: 100 Hz, was 0.002

    ReflexxesAPI* RML = NULL;
    RMLPositionInputParameters* IP = NULL;
    RMLPositionOutputParameters* OP = NULL;
    RMLPositionFlags RML_PosFlags;

    RML = new ReflexxesAPI(NUMBER_OF_JOINTS, CycleTime);
    IP = new RMLPositionInputParameters(NUMBER_OF_JOINTS);
    OP = new RMLPositionOutputParameters(NUMBER_OF_JOINTS);
    RML_PosFlags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

    memset(JointValuesInRad, 0x0, NUMBER_OF_JOINTS * sizeof(float));

    if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
    {
        printf("Program is going to stop the robot.\n");
        FRI->StopRobot();

        FRI->GetMeasuredJointPositions(JointValuesInRad);
        FRI->SetCommandedJointPositions(JointValuesInRad);

        printf("Restarting the joint position control scheme.\n");
        ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);

        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            printf("An error occurred during starting up the robot...\n");
            delete RML;
            delete IP;
            delete OP;
            return 0;
        }
    }

    printf("Moving to the candle position..\n");

    FRI->GetMeasuredJointPositions(JointValuesInRad);

    for (i = 0; i < NUMBER_OF_JOINTS; i++)
    {
        IP->CurrentPositionVector->VecData[i] = JointValuesInRad[i];
        IP->CurrentVelocityVector->VecData[i] = 0.0;      // fnh: assuming LWR is at rest!!
        IP->CurrentAccelerationVector->VecData[i] = 0.0;  // fnh: assuming LWR is at rest!!

        IP->MaxVelocityVector->VecData[i] = 0.5;
        IP->MaxAccelerationVector->VecData[i] = 0.1;
        IP->MaxJerkVector->VecData[i] = 1.0;

        IP->TargetPositionVector->VecData[i] = 0.02;  // fnh: was 0.2, candle is 0.0... ?
        IP->TargetVelocityVector->VecData[i] = 0.0;   // fnh: robot should stop at target
        IP->SelectionVector->VecData[i] = true;
    }

    ResultValue = ReflexxesAPI::RML_WORKING;

    while ((FRI->IsMachineOK()) && (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED))
    {
        FRI->WaitForKRCTick();

        // was: ResultValue  =  RML->GetNextMotionState_Position( *IP, OP );
        ResultValue = RML->RMLPosition(*IP, OP, RML_PosFlags);

        // if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
        if (ResultValue < 0)
        {
            printf("MoveToCandle(): ERROR during trajectory generation (%d).", ResultValue);
        }

        for (i = 0; i < NUMBER_OF_JOINTS; i++)
        {
            JointValuesInRad[i] = (OP->NewPositionVector->VecData[i]);
        }

        printf("moving to %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf\n", JointValuesInRad[0], JointValuesInRad[1],
               JointValuesInRad[2], JointValuesInRad[3], JointValuesInRad[4], JointValuesInRad[5], JointValuesInRad[6]);

        FRI->SetCommandedJointPositions(JointValuesInRad);

        *(IP->CurrentPositionVector) = *(OP->NewPositionVector);
        *(IP->CurrentVelocityVector) = *(OP->NewVelocityVector);
    }

    if (!FRI->IsMachineOK())
    {
        printf("MoveToCandle(): ERROR, machine is not ready.");

        delete RML;
        delete IP;
        delete OP;

        return 0;
    }

    printf("Stopping the robot.\n");
    FRI->StopRobot();

    if (ResultValue != EOK)
    {
        printf("An error occurred during stopping the robot...\n");
    }
    delete RML;
    delete IP;
    delete OP;

    printf("Exiting.\n");
    return 0;
}
