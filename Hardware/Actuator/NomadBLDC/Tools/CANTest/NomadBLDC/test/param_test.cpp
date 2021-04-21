
#include <CAN/PCANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <NomadBLDC/NomadBLDC.h>

#define DEVICE "/dev/pcanusbfd32"

PCANDevice can;

int main(int argc, char *argv[])
{
    CANDevice::Config_t config;
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 5e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
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

    NomadBLDC servo(1, 0x11, &can);
   if(!servo.Connect())
   {
       std::cout << "[ERROR]: Unable to connect to Nomad Servo!" << std::endl;
       return -1;
   }

    std::cout << "Nomad Servo: " << servo.GetServoId() << " Connected!" << std::endl;
   // servo.SetMaxCurrent(35.0f);


  //  usleep(100000);


 //  servo.SaveConfig();

  //  std::cout << "SET!" << std::endl;
    float torque_max = servo.GetMaxCurrent();

   std::cout << "MAX A: " << torque_max << std::endl;
   // servo.SetControlMode(10);

    //float torque_max2 = servo.GetMaxTorque();

   // usleep(10000);

   // servo.ClosedLoopTorqueCommand(1.0f,0.0f, 0.0f,0.0f,0.0f);

getchar();

  //  servo.SetControlMode(1);
    return 0;
}
