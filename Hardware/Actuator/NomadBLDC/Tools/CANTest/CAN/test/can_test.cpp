#include <CAN/CANDevice.h>
#include <iostream>

int main()
{
    CANDevice can;
    if(!can.Open("can0"))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return -1;
    }

    CANDevice::CAN_msg_t msg;
    msg.id = 0x123;
    msg.length = 3;
    msg.data[0] = 0x01;
    msg.data[1] = 0x0f;
    msg.data[2] = 0x00;
    can.Send(msg);

    



    bool b_close = can.Close();
}