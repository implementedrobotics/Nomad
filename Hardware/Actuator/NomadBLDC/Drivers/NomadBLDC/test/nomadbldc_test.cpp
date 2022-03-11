
#include <CAN/PCANDevice.h>
#include <NomadBLDC/Registers.h>
#include <NomadBLDC/RealTimeTask.hpp>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <NomadBLDC/NomadBLDC.h>

#define DEVICE "/dev/pcanusbfd32"
//#define DEVICE "/dev/pcanpcifd1"
PCANDevice can;

int main(int argc, char *argv[])
{
    CANDevice::Config_t config;
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 5e6; //5mbps
    config.sample_point = 0.80; //80.0% 
    config.d_sample_point = 0.625; //62.5%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    config.mode_fd = 1; // FD Mode

    if(!can.Open(DEVICE, config, true))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return -1;
    }

    // Setup Filters
    can.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can.AddFilter(1, 2); // Only Listen to messages on id 0x01.  

    NomadBLDC servo(2, 0x10, &can);
    if(!servo.Connect())
    {
        std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
        return -1;
    }

    std::cout << "Nomad Servo: " << servo.GetServoId() << " Connected!" << std::endl;

    servo.SetControlMode(10);

    usleep(10000);

    servo.ClosedLoopTorqueCommand(5.0f,0.0f, 0.0f,0.0f,0.0f);

getchar();

    servo.SetControlMode(1);
    return 0;
}
