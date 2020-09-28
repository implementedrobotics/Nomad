/*
 * DRV8323.cpp
 *
 *  Created on: September 23, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

// Primary Include
#include "DRV8323.h"

// C System Files

// C++ System Files

// Project Includes
#include <Utilities/utils.h>

DRV8323::DRV8323(SPIDevice *spi, GPIO_t enable_pin, GPIO_t nFault_pin) : spi_(spi), enable_(enable_pin), nFault_(nFault_pin)
{

}

bool DRV8323::Init()
{
    if(!DriverEnabled())

        return false;
    
    // Reset All Registers
    WriteRegister(RegisterAddress_e::DriverControl, 0x0);
    WriteRegister(RegisterAddress_e::GateDriveHS, RegisterLockMode_e::REG_UNLOCK); // Keep Unlocked
    WriteRegister(RegisterAddress_e::GateDriveLS, 0x0);
    WriteRegister(RegisterAddress_e::OCPControl, 0x0);
    WriteRegister(RegisterAddress_e::CSAControl, 0x0);

    
    /* Driver Control Register Nit */
    //EnableOTWReport();
    SetPWMMode(PwmMode_e::PWM_MODE_3X); // 3X PWM Mode
    ClearLatchedFaults(); // Clear Any Latched Faults

    /* Gate Drive HS Register */
    //UnlockRegisters();
    SetIDriveP_HS(GateDriveSource_e::IDRIVEP_260_mA);
    SetIDriveN_HS(GateDriveSink_e::IDRIVEN_520_mA);
    
    /* Gate Drive LS Register */
    EnableCBC();
    SetTDrive(GateDriveTime_e::TDRIVE_TIME_1000ns);
    SetIDriveP_LS(GateDriveSource_e::IDRIVEP_260_mA);
    SetIDriveN_LS(GateDriveSink_e::IDRIVEN_520_mA);

    /* OCP Conteol Register */
    EnableTRETRY_4ms(); // Default
    SetDeadTime(DeadTime_e::DEADTIME_100ns);
    SetOCPMode(OverCurrentMode_e::OC_MODE_LATCHSHUTDOWN); // Latch Overcurrent Faults
    SetOCPDeglitch(OverCurrentDeglitch_e::OC_DEGLITCH_4us); // Default
    SetVDSLevel(OverCurrentVdsLevel_e::VDSLEVEL_0p20_V);

    /* CSA Control Register */
    EnableVREFDiv(); // Bidirectional Mode
    SetCSAGain(CSA_Gain_e::CSA_GAIN_40VpV);
    SetOCPSenseLevel(SenseOCLevel_e::SENSE_OC_LEVEL_1p0_V);
    

    return true;
}

void DRV8323::EnableDriver()
{
    // Enable DRV
    LL_GPIO_SetOutputPin(enable_.port, enable_.pin);
}

void DRV8323::DisableDriver()
{
    // Enable DRV
    //LL_GPIO_ResetOutputPin(enable_.port, enable_.pin);
}

void DRV8323::FaultReset()
{
    // Pulse Reset
    LL_GPIO_ResetOutputPin(enable_.port, enable_.pin);
    delay_us(20);
    LL_GPIO_SetOutputPin(enable_.port, enable_.pin);
}

bool DRV8323::InFault()
{
    // TODO: Check for Fault Mode.  Could be hardware GPIO, or SPI.  For now we have nFault wired
    return !LL_GPIO_IsOutputPinSet(nFault_.port, nFault_.pin); // Pulled low in fault condition
}

void DRV8323::PrintFaults()
{
    
}

void DRV8323::Calibrate()
{
    uint16_t cal_mask = CSA_CTRL_CSA_CAL_A_MASK | CSA_CTRL_CSA_CAL_B_MASK | CSA_CTRL_CSA_CAL_C_MASK;
    // Start Calibration Mode
    UpdateRegister(RegisterAddress_e::CSAControl, cal_mask, cal_mask);

    // Datasheet says 100us to complete.  We add some margin here
    delay_us(300); 

    // End Calibration Mode
    UpdateRegister(RegisterAddress_e::CSAControl, cal_mask, 0);

}

bool DRV8323::DriverEnabled()
{
    return LL_GPIO_IsOutputPinSet(enable_.port, enable_.pin);
}

void DRV8323::EnableUVLO()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_DIS_CPUV_MASK, 0x0);
}

void DRV8323::DisableUVLO()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_DIS_CPUV_MASK, DRIVER_CTRL_DIS_CPUV_MASK);
}

void DRV8323::EnableGDF()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_DIS_GDF_MASK, 0x0);
}

void DRV8323::DisableGDF()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_DIS_GDF_MASK, DRIVER_CTRL_DIS_GDF_MASK);
}

void DRV8323::EnableOTWReport()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_OTW_REP_MASK, DRIVER_CTRL_OTW_REP_MASK);
}

void DRV8323::DisableOTWReport()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_OTW_REP_MASK, 0x0);
}

void DRV8323::SetPWMMode(PwmMode_e pwm_mode)
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_PWM_MODE_MASK, pwm_mode);
}

void DRV8323::EnableCoast()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_COAST_MASK, DRIVER_CTRL_COAST_MASK);
}

void DRV8323::DisableCoast()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_COAST_MASK, 0x0);
}

void DRV8323::ClearLatchedFaults()
{
    UpdateRegister(RegisterAddress_e::DriverControl, DRIVER_CTRL_CLR_FLT_MASK, DRIVER_CTRL_CLR_FLT_MASK);
}

void DRV8323::LockRegisters()
{
    UpdateRegister(RegisterAddress_e::GateDriveHS, GATE_DRIVE_HS_LOCK_MASK, RegisterLockMode_e::REG_LOCK);
}

void DRV8323::UnlockRegisters()
{
    UpdateRegister(RegisterAddress_e::GateDriveHS, GATE_DRIVE_HS_LOCK_MASK, RegisterLockMode_e::REG_UNLOCK);
}

void DRV8323::SetIDriveP_HS(GateDriveSource_e I_source)
{
    UpdateRegister(RegisterAddress_e::GateDriveHS, GATE_DRIVE_HS_IDRIVEP_HS_MASK, I_source);
}
void DRV8323::SetIDriveN_HS(GateDriveSink_e I_sink)
{
    UpdateRegister(RegisterAddress_e::GateDriveHS, GATE_DRIVE_HS_IDRIVEN_HS_MASK, I_sink);
}

void DRV8323::EnableCBC()
{
    UpdateRegister(RegisterAddress_e::GateDriveLS, GATE_DRIVE_LS_CBC_MASK, GATE_DRIVE_LS_CBC_MASK);
}

void DRV8323::DisableCBC()
{
    UpdateRegister(RegisterAddress_e::GateDriveLS, GATE_DRIVE_LS_CBC_MASK, 0x0);
}

void DRV8323::SetTDrive(GateDriveTime_e time)
{
    UpdateRegister(RegisterAddress_e::GateDriveLS, GATE_DRIVE_LS_TDRIVE_MASK, time);
}

void DRV8323::SetIDriveP_LS(GateDriveSource_e I_source)
{
    UpdateRegister(RegisterAddress_e::GateDriveLS, GATE_DRIVE_LS_IDRIVEP_LS_MASK, I_source);
}

void DRV8323::SetIDriveN_LS(GateDriveSink_e I_sink)
{
    UpdateRegister(RegisterAddress_e::GateDriveLS, GATE_DRIVE_LS_IDRIVEN_LS_MASK, I_sink);
}

void DRV8323::EnableTRETRY_4ms()
{
    UpdateRegister(RegisterAddress_e::OCPControl, OCP_CTRL_TRETRY_MASK, 0x0);
}

void DRV8323::EnableTRETRY_50us()
{
    UpdateRegister(RegisterAddress_e::OCPControl, OCP_CTRL_TRETRY_MASK, OCP_CTRL_TRETRY_MASK);
}

void DRV8323::SetDeadTime(DeadTime_e dead_time)
{
    UpdateRegister(RegisterAddress_e::OCPControl, OCP_CTRL_DEAD_TIME_MASK, dead_time);
}

void DRV8323::SetOCPMode(OverCurrentMode_e OCP_mode)
{
    UpdateRegister(RegisterAddress_e::OCPControl, OCP_CTRL_OCP_MODE_MASK, OCP_mode);
}

void DRV8323::SetOCPDeglitch(OverCurrentDeglitch_e OCP_deg)
{
    UpdateRegister(RegisterAddress_e::OCPControl, OCP_CTRL_OCP_DEG_MASK, OCP_deg);
}

void DRV8323::SetVDSLevel(OverCurrentVdsLevel_e VDS_lvl)
{
    UpdateRegister(RegisterAddress_e::OCPControl, OCP_CTRL_VDS_LVL_MASK, VDS_lvl);
}

void DRV8323::SetCSAPosInput_SPx()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_FET_MASK, 0x0);
}

void DRV8323::SetCSAPosInput_SHx()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_FET_MASK, CSA_CTRL_CSA_FET_MASK);
}

void DRV8323::EnableVREFDiv()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_VREF_DIV_MASK, CSA_CTRL_VREF_DIV_MASK);
}
void DRV8323::DisableVREFDiv()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_VREF_DIV_MASK, 0x0);
}

void DRV8323::EnableLSRef()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_LS_REF_MASK, CSA_CTRL_LS_REF_MASK);
}
void DRV8323::DisableLSRef()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_LS_REF_MASK, 0x0);
}

void DRV8323::SetCSAGain(CSA_Gain_e gain)
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_GAIN_MASK, gain);
}

void DRV8323::EnableOCSenseFault()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_DIS_SEN_MASK, 0x0);
}

void DRV8323::DisableOCSenseFault()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_DIS_SEN_MASK, CSA_CTRL_DIS_SEN_MASK);
}

void DRV8323::EnableCSA_CAL_A()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_CAL_A_MASK, CSA_CTRL_CSA_CAL_A_MASK);
}

void DRV8323::DisableCSA_CAL_A()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_CAL_A_MASK, 0x0);
}

void DRV8323::EnableCSA_CAL_B()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_CAL_B_MASK, CSA_CTRL_CSA_CAL_B_MASK);
}

void DRV8323::DisableCSA_CAL_B()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_CAL_B_MASK, 0x0);
}

void DRV8323::EnableCSA_CAL_C()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_CAL_C_MASK, CSA_CTRL_CSA_CAL_C_MASK);
}

void DRV8323::DisableCSA_CAL_C()
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_CSA_CAL_C_MASK, 0x0);
}

void DRV8323::SetOCPSenseLevel(SenseOCLevel_e lvl)
{
    UpdateRegister(RegisterAddress_e::CSAControl, CSA_CTRL_SEN_LVL_MASK, lvl);
}

uint16_t DRV8323::GetFaultStatus1()
{
    return ReadRegister(RegisterAddress_e::FaultStatus1);
}

uint16_t DRV8323::GetFaultStatus2()
{
    return ReadRegister(RegisterAddress_e::FaultStatus2);
}


void DRV8323::WriteRegister(uint16_t address, uint16_t value)
{
    uint16_t tx_data = REG_WRITE_MASK | (address << 11) | (value & DATA_MASK);

    spi_->Select();
    spi_->TransmitReceive16(tx_data);
    spi_->Deselect();
}

uint16_t DRV8323::ReadRegister(uint16_t address)
{
    uint16_t tx_data = REG_READ_MASK | (address << 11);
    uint16_t rx_data = 0;

    spi_->Select();
    rx_data = spi_->TransmitReceive16(tx_data);
    spi_->Deselect();

    return rx_data & DATA_MASK;
}

void DRV8323::UpdateRegister(uint16_t address, uint16_t value_mask, uint16_t value)
{
    uint16_t data = ReadRegister(address);

    // Clear bits we are updating
    data &= (~value_mask);

    // Set the bits
    data |= value;

    WriteRegister(address, data);
}