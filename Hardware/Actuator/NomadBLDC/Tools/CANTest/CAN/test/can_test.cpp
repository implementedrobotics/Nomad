#include <CAN/CANDevice.h>
#include <CAN/Registers.h>
#include <string.h>
#include <iostream>
#include <unistd.h>

int main()
{
    CANDevice can;
    if(!can.Open("can0", 1))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        return -1;
    }

    register_command_t test;
    test.header.rwx = 0;
    test.header.address = ControllerStateRegisters_e::FETTemp;
    test.header.data_type = 1;
    test.header.sender_id = 0x001;
    test.header.length = 0;

    //test.header.reserved = 1;
    //uint32_t new_val = 64001;


    //memcpy(&test.cmd_data, (uint32_t *)&new_val, sizeof(uint32_t));

   // memcpy(&test.cmd_data, (uint8_t *)&test_me, sizeof(Test_Struct));
   
    std::cout << "Command Size: " << sizeof(request_header_t) << std::endl;

    for (int j = 0; j < 100; j++)
    {
        CANDevice::CAN_msg_t msg;
        msg.id = 0x123;
        msg.length = sizeof(request_header_t);
        memcpy(msg.data, &test, msg.length);

        can.Send(msg);

        int i = 0;
        while (!can.Receive(msg))
        {
            if (i++ > 10000)
                break;

            std::cout << "WAITING: " << std::endl;
        }

        register_reply_t *reponse = (register_reply_t *)msg.data;
        std::cout << "Receive Message: " << *(float *)reponse->cmd_data << std::endl;

        usleep(10000);
    }
    bool b_close = can.Close();
}