/*
 * Aris_ControlData.h
 *
 *  Created on: Nov 13, 2014
 *      Author: leo
 */

#ifndef ARIS_CONTROLDATA_H_
#define ARIS_CONTROLDATA_H_
#include "Aris_Core.h"

#define FORCE_SENSOR_NUMBER  6
#define AXIS_NUMBER 18

extern char ServoStateName[7][20];

class CSysBase;

namespace Aris
{
    namespace RT_CONTROL
    {
        class ACTUATION;

        enum EServoState
        {
            EMSTAT_NONE=0,
            EMSTAT_POWEREDOFF=1,
            EMSTAT_STOPPED=2,
            EMSTAT_ENABLED=3,
            EMSTAT_RUNNING=4,
            EMSTAT_HOMING=5,
            EMSTAT_FAULT,
            EMSTAT_INVALID,
            EMSTAT_EMERGE,
        };
        const int N_MOTOR_STATE=9;

        enum EOperationMode
        {
            OM_INVALID     = -1,
            //OM_PROFILEPOS = 1,
            OM_CYCLICPOS  = 8,
            OM_CYCLICVEL  = 9,
            OM_CYCLICTORQ = 10,
            //  OM_OTHER      = 18
        };

        enum EOperationModeDisplay
        {
            OMD_INVALID     = -1,
            OMD_PROFILEPOS = 1,
            OMD_HOMING =6,
            OMD_CYCLICPOS  = 8,
            OMD_CYCLICVEL  = 9,
            OMD_CYCLICTORQ = 10,
            OMD_OTHER      = 18
        };

        enum EServoCommand
        {
            EMCMD_NONE=0,
            EMCMD_POWEROFF=1,
            EMCMD_STOP=2,
            EMCMD_ENABLE=3,
            EMCMD_RUNNING=4,
            EMCMD_GOHOME=5,
        };

        const int N_MOTOR_CMD=6;

        enum EControl_Msg
        {
            CM_PUSH_CMD_TO_MOTORS=100,
            CM_GO_TRAJ,
            CM_CUS_MESSAGE,
            CM_NONE,
        };

        enum EMachineState
        {
            CS_UNINITED,
            CS_INITED,
            CS_COMM_INITED,
            CS_RTTASK_STARTED,
            CS_STOPPED,
        };

        /*
         * CSysInitParameters should be used by SetSysInitializer
         */
        class CSysInitParameters
        {

            public:
                CSysInitParameters();
                ~CSysInitParameters();
                //	CSysInitParameters& operator=(const CSysInitParameters& other);
                int motorNum;
                int homeMode;
                int homeAccel;
                int homeLowSpeed;
                int homeHighSpeed;
                int p2pMaxSpeed;
                int p2pSpeed;
                int nsPerCyclePeriod;
                int homeTorqueLimit;
                int* homeOffsets;
                int* driverIDs;
        };

        /*
         * CForceData struct contains the 6-D force data (Fx Fy Fz Mx Fy Mz), used by CMachineData
         */
        class CForceData
        {
            public:
                CForceData& operator=(const CForceData& other);
                int forceValues[6];
                int isZeroingRequest;
        };

        class CIMUData
        {
            public:
                float EulerAngle[3];
                float AngularVel[3];
                float LinearAcc[3];

                CIMUData()
                {
                    for (int i = 0; i < 3; ++i) {
                        EulerAngle[i] = 0;
                        AngularVel[i] = 0;
                        LinearAcc[i]  = 0;
                    }
                }
        };
        /*
         * CMotorData struct contain one single motor's data, used by CMachineData
         */
        class CMotorData
        {
            public:
                CMotorData& operator=(const CMotorData& other);
                int Position;
                int Velocity;
                int Torque;
        };
        /*
         * CWorkData struct should be used by TrajectoryGenerator and Logger as Input Data
         *
         * may include sensor data, and other data
         *
         */

#define RT_MSG_BUFFER_SIZE 8192
#define RT_MSG_HEADER_LENGTH MSG_HEADER_LENGTH
#define PRINT_INFO_BUFFER_SIZE 200

        class CMachineData
        {
            const static int CONTROL_DATA_SIZE = 128;
            public:
                int motorNum;
                EMachineState machinestate;
                long long int time;
                //Motor Data
                CMachineData& operator=(const CMachineData& other);
                bool isMotorHomed[AXIS_NUMBER];
                EServoState motorsStates[AXIS_NUMBER];
                EServoCommand motorsCommands[AXIS_NUMBER];
                EOperationMode motorsModes[AXIS_NUMBER];
                EOperationModeDisplay motorsModesDisplay[AXIS_NUMBER];
                CMotorData feedbackData[AXIS_NUMBER];//currentFeedback,collected after read()
                CMotorData commandData[AXIS_NUMBER];//lastCommand,collected before write()
                //sensor data
                CForceData forceData[FORCE_SENSOR_NUMBER];
                CIMUData   imuData; // only one can be used at CURRENT STAGE
                // Controller Data
                char controlData[CONTROL_DATA_SIZE]; // We offer a space to store possible controller data
        };

    }

}

/*
 * Function pointer with peculiar parameters
 */
typedef int (*FuncPtrWork)(Aris::RT_CONTROL::CMachineData&, Aris::Core::RT_MSG&,Aris::Core::RT_MSG&);
typedef int (*FuncPtrInit)(Aris::RT_CONTROL::CSysInitParameters&);
typedef int (*FuncPtrState)(void*);
typedef int (*FuncPtrCustom)(Aris::Core::MSG&);


#endif /* ARIS_CONTROLDATA_H_ */
