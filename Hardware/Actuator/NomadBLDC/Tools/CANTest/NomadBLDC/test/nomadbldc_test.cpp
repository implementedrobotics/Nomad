
#include <CAN/PCANDevice.h>
#include <CAN/Registers.h>
#include <CAN/RealTimeTask.hpp>
#include <string.h>
#include <iostream>

#define DEVICE "/dev/pcanusbfd32"

PCANDevice can;
uint32_t can_tx_id = 0x10;
uint32_t can_tx_id2 = 0x110;

float pos1 = 0.0f;
float pos2 = 0.0f;
float vel1 = 0.0f;
float vel2 = 0.0f;

float kp = .050f;
float kd = .010f;
float tau1 = 0.0f;
float tau2 = 0.0f;


int main(int argc, char *argv[])
{
    return 0;
}
