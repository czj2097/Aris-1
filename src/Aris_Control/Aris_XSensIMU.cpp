#include "Aris_XSensIMU.h"

using namespace Aris::RT_CONTROL;

CIMUDevice::CIMUDevice()
{
    m_isDeviceOpened = false;
}

CIMUDevice::~CIMUDevice()
{
    if (m_isDeviceOpened)
    {
        m_device.close();
        m_isDeviceOpened = false;
    }
}

int CIMUDevice::Initialize()
{
    try
    {
        // Scan for connected USB devices
        XsPortInfoArray portInfoArray;
        xsEnumerateUsbDevices(portInfoArray);
        if (!portInfoArray.size())
        {
            std::string portNumber="/dev/ttyUSB0";
            int baudRate=921600;

            XsPortInfo portInfo(portNumber, XsBaud::numericToRate(baudRate));
            portInfoArray.push_back(portInfo);
        }

        // Use the first detected device
        m_mtPort = portInfoArray.at(0);

        // Open the port with the detected device
        if (!m_device.openPort(m_mtPort))
            throw std::runtime_error("Could not open port. Aborting.");

        // Put the device in configuration mode
        if (!m_device.gotoConfig()) // Put the device into configuration mode before configuring the device
        {
            throw std::runtime_error("Could not put device into configuration mode. Aborting.");
        }

        // Request the device Id to check the device type
        m_mtPort.setDeviceId(m_device.getDeviceId());

        // Check if we have an MTi / MTx / MTmk4 device
        if (!m_mtPort.deviceId().isMt9c() && !m_mtPort.deviceId().isMtMk4())
        {
            throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
        }
        std::cout << "Found a device with id: " << m_mtPort.deviceId().toString().toStdString() << " @ port: " << m_mtPort.portName().toStdString() << ", baudrate: " << m_mtPort.baudrate() << std::endl;

        try
        {
            if (m_mtPort.deviceId().isMt9c())
            {
                XsOutputMode outputMode = XOM_Orientation; // output orientation data
                XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

                // set the device configuration
                if (!m_device.setDeviceMode(outputMode, outputSettings))
                {
                    throw std::runtime_error("Could not configure MT device. Aborting.");
                }
            }
            else if (m_mtPort.deviceId().isMtMk4())
            {
                XsOutputConfiguration quat_Q(XDI_Quaternion, 100);
                XsOutputConfiguration quat_DQ(XDI_DeltaQ, 100);
                XsOutputConfiguration quat_DV(XDI_DeltaV, 100);
                XsOutputConfiguration quat_ACC(XDI_Acceleration, 100);
                XsOutputConfigurationArray configArray;
                configArray.push_back(quat_Q);
                configArray.push_back(quat_DQ);
                configArray.push_back(quat_DV);
                configArray.push_back(quat_ACC);
                if (!m_device.setOutputConfiguration(configArray))
                {
                    throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
                }
            }
            else
            {
                throw std::runtime_error("Unknown device while configuring. Aborting.");
            }

            // Put the device in measurement mode
            std::cout << "Putting device into measurement mode..." << std::endl;
            if (!m_device.gotoMeasurement())
            {
                throw std::runtime_error("Could not put device into measurement mode. Aborting.");
            }
        }
        catch (std::runtime_error const & error)
        {
            std::cout << error.what() << std::endl;
            m_device.close();
            return -2;
        }

    }
    catch (std::runtime_error const & error)
    {
        std::cout << error.what() << std::endl;
        return -1;
    }
    m_msgs.clear();

    std::cout << "IMU Initialized" << std::endl;

    m_isDeviceOpened = true;

    return 0;
}

int CIMUDevice::UpdateData(CIMUData& imuData)
{
    if (!m_isDeviceOpened)
        return -1;

    m_device.readDataToBuffer(m_byteData);
    m_device.processBufferedData(m_byteData, m_msgs);
    
    for (XsMessageArray::iterator it = m_msgs.begin(); it != m_msgs.end(); ++it)
    {
        // Retrieve a packet
        if ((*it).getMessageId() == XMID_MtData2) 
        {
            XsDataPacket packet;
            packet.setMessage((*it));
            packet.setDeviceId(m_mtPort.deviceId());

            // Get the quaternion data
            XsQuaternion quaternion = packet.orientationQuaternion();
            // Get deltaQ
            XsSdiData deltaParam=packet.sdiData();
            XsQuaternion deltaQ=deltaParam.orientationIncrement();
            XsVector deltaV=deltaParam.velocityIncrement();
            // Convert packet to euler
            XsEuler euler = packet.orientationEuler();
            // Get acceleration
            XsVector acceleration=packet.calibratedAcceleration();
            for (int i = 0; i < 3; ++i) 
            {
                if (packet.containsOrientation())
                    imuData.EulerAngle[i] = euler[i] / 180.0 * 3.14159265359;

                if (packet.containsSdiData())
                    imuData.AngularVel[i] = deltaQ[i+1] * 100 * 2.0;

                if (packet.containsCalibratedAcceleration())
                    imuData.LinearAcc[i] = acceleration[i];
            }
        }
    }
    m_msgs.clear();
    return 0;
}

int CIMUDevice::Sleep(int milliseconds)
{
    XsTime::msleep(milliseconds);
    return 0;
}


int CIMUDevice::Close()
{
    if (m_isDeviceOpened)
    {
        m_device.close();
    }
    return 0;
}
