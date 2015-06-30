/*
 * Aris_ControlData.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: leo
 */

#include "Aris_ControlData.h"
#include "GlobalConfiguration.h"
#include "stdio.h"
#include <string.h>


char ServoStateName[7][20]=
{
    "EMSTAT_NONE",
    "EMSTAT_POWEREDOFF",
    "EMSTAT_STOPPED",
    "EMSTAT_ENABLED",
    "EMSTAT_RUNNING",
    "EMSTAT_HOMING",
    "EMSTAT_FAULT",
};

namespace Aris
{
    namespace RT_CONTROL
    {
        CSysInitParameters::CSysInitParameters()
        {

            //    m_motorGeneralSettings.homeMode = HOMING_MODE;
            //    m_motorGeneralSettings.homeAccel = HOMING_ACC;
            //    m_motorGeneralSettings.homeLowSpeed = HOMING_LO_SPEED;
            //    m_motorGeneralSettings.homeHighSpeed = HOMING_HI_SPEED;
            //    m_motorGeneralSettings.p2pMaxSpeed = PTP_MAX_SPEED;
            //    m_motorGeneralSettings.p2pSpeed = PTP_SPEED;
            //    m_motorGeneralSettings.nsPerCyclePeriod = PERIOD_NS_CORE;

            motorNum 			 = ACTUAL_MOTOR_NUMBER;
            homeMode 			 = HOMING_MODE;
            homeAccel 			= HOMING_ACC;
            homeLowSpeed	= HOMING_LO_SPEED;
            homeHighSpeed= HOMING_HI_SPEED;
            p2pMaxSpeed 	= PTP_MAX_SPEED;
            p2pSpeed 			 = PTP_SPEED;
            nsPerCyclePeriod   	= PERIOD_NS_CORE;
            homeTorqueLimit     = HOMING_TORQUE_LIMIT;
            homeOffsets=NULL;
            driverIDs=NULL;
            /*  for(int i=0;i<ACTUAL_MOTOR_NUMBER;i++)
                {
                homeOffsets[i]=HEXBOT_HOME_OFFSETS_RESOLVER[i];
                driverIDs[i]=i;
                }
                */
        };
        CSysInitParameters::~CSysInitParameters()
        {

        };

        //CSysInitParameters& CSysInitParameters::operator=(const CSysInitParameters& other)
        //{
        //	return *this;
        //};
        CForceData& CForceData::operator=(const CForceData& other)
        {
            memcpy(this,&other,sizeof(*this));
            return *this;
        }

        CMotorData& CMotorData::operator=(const CMotorData& other)
        {
            memcpy(this,&other,sizeof(*this));
            return *this;
        };

        CMachineData& CMachineData::operator=(const CMachineData& other)
        {
            memcpy(this,&other,sizeof(*this));
            return *this;
        };
    }
}
