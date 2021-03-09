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

    register_command_t enable;
    enable.header.rwx = 1;
    enable.header.address = ControllerStateRegisters_e::ControlMode;
    enable.header.data_type = 1;
    enable.header.sender_id = 0x001;
    enable.header.length = 4;


    register_command_t test;
    test.header.rwx = 1;
    test.header.address = ControllerStateRegisters_e::TorqueControlModeRegister;
    test.header.data_type = 1;
    test.header.sender_id = 0x001;
    test.header.length = 4;
    uint32_t new_mode = 10;
    //test.header.reserved = 1;
    //uint32_t new_val = 64001;

    TorqueControlModeRegister_t tcmr;

    tcmr.K_d = 0.05f;
    tcmr.K_p = 20.0f;
    tcmr.Pos_ref = 0.0f;
    tcmr.Vel_ref = 0.0f;
    tcmr.T_ff = 0.0f;

    
    memcpy(&enable.cmd_data, &new_mode, sizeof(uint32_t));
    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));

   // memcpy(&test.cmd_data, (uint8_t *)&test_me, sizeof(Test_Struct));
   
    std::cout << "Command Size: " << sizeof(request_header_t) << std::endl;

  //  for (int j = 0; j < 1; j++)
   // {
        CANDevice::CAN_msg_t msg;
         msg.id = 0x123;

        msg.length = sizeof(request_header_t) + sizeof(uint32_t);
        memcpy(msg.data, &enable, msg.length);

        can.Send(msg);

usleep(1000000);

for (int j = 0; j < 2500; j++)
{
    tcmr.Pos_ref += 0.05f;
    memcpy(&test.cmd_data, (uint8_t *)&tcmr, sizeof(TorqueControlModeRegister_t));
    msg.length = sizeof(request_header_t) + sizeof(TorqueControlModeRegister_t);
    memcpy(msg.data, &test, msg.length);

    can.Send(msg);

    usleep(100000);
}

        // int i = 0;
        // while (!can.Receive(msg))
        // {
        //     if (i++ > 10000)
        //         break;

        //     std::cout << "WAITING: " << std::endl;
        // }

//        register_reply_t *reponse = (register_reply_t *)msg.data;
  //      std::cout << "Receive Message: " << *(uint16_t *)reponse->cmd_data << std::endl;

      //  usleep(10000);
  //  }
    std::cout << "OUT!" << std::endl;
    bool b_close = can.Close();
}