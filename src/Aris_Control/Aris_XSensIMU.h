#ifndef XSENS_IMU_H
#define XSENS_IMU_H

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>
#include <xcommunication/mtwsdidata.h>

#include "deviceclass.h"
#include "Aris_ControlData.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace Aris{
    namespace RT_CONTROL{

        class CIMUDevice
        {
            public:
                CIMUDevice();
                ~CIMUDevice();
                int Initialize();
                int UpdateData(CIMUData& data);
                int Sleep(int milliseconds);
            private:
                DeviceClass m_device;
                XsByteArray m_byteData;
                XsMessageArray m_msgs;
                XsPortInfo m_mtPort;
        };
    }
}


#endif
