/*
 * DRV8323.h
 *
 *  Created on: September 22, 2020
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

#ifndef DRV8323_H_
#define DRV8323_H_

// C System Files

// C++ System Files

// Project Includes
#include <Peripherals/spi.h>
#include "main.h"

class DRV8323
{
public:
    //     //Defines the address mask
    // //!
    // static constexpr uint16_t ADDR_MASK               (0x7800)

    //Defines the data mask
    //!
    static constexpr uint16_t DATA_MASK = (0x7FF); // (b000001111111111) / First 5 bits are don't care bits

    // //Defines the R/W mask
    // //!
    // static constexpr uint16_t RW_MASK                 (0x8000)

    // //Defines the R/W mask
    // //!
    // static constexpr uint16_t FAULT_TYPE_MASK         (0x07FF)

    static constexpr uint16_t REG_READ_MASK = (1 << 15);

    static constexpr uint16_t REG_WRITE_MASK = (0 << 15);

    /* Faults Status Register 1 Masks */

    // Defines the location of the VDS_LC (VDS Over Current fault on the C low-side MOSFET) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_VDS_LC_MASK = (1 << 0);

    // Defines the location of the VDS_HC (VDS Over Current fault on the C high-side MOSFET) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_VDS_HC_MASK = (1 << 1);

    //Defines the location of the VDS_LB (VDS Over Current fault on the B high-side MOSFET) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_VDS_LB_MASK = (1 << 2);

    //Defines the location of the VDS_HB (VDS Over Current fault on the B high-side MOSFET) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_VDS_HB_MASK = (1 << 3);

    //Defines the location of the VDS_LA (VDS Over Current fault on the A high-side MOSFET) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_VDS_LA_MASK = (1 << 4);

    //Defines the location of the VDS_HA (VDS Over Current fault on the A high-side MOSFET) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_VDS_HA_MASK = (1 << 5);

    //Defines the location of the OTSD (Over Temperature Shut Down) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_OTSD_MASK = (1 << 6);

    //Defines the location of the UVLO (Under Voltage Lockout) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_UVLO_MASK = (1 << 7);

    //Defines the location of the GDF (Gate Drive Fault) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_GDF_MASK = (1 << 8);

    //Defines the location of the VDS_OCP (VDS monitor Over Current) MASK in the Status 1 register
    static constexpr uint16_t STATUS1_VDS_OCP_MASK = (1 << 9);

    //Defines the location of the FAULT MASK in the Status 1 register
    static constexpr uint16_t STATUS1_FAULT_MASK = (1 << 10);



    /* Faults Status Register 2 Masks */

    //Defines the location of the VGS_LC (VGS Gate Drive Fault on the C low-side MOSFET) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_VGS_LC_MASK = (1 << 0);

    //Defines the location of the VGS_HC (VGS Gate Drive Fault on the C high-side MOSFET) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_VGS_HC_MASK = (1 << 1);

    //Defines the location of the VGS_LB (VGS Gate Drive Fault on the B low-side MOSFET) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_VGS_LB_MASK = (1 << 2);

    //Defines the location of the VGS_HB (VGS Gate Drive Fault on the B high-side MOSFET) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_VGS_HB_MASK = (1 << 3);

    //Defines the location of the VGS_LA (VGS Gate Drive Fault on the A low-side MOSFET) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_VGS_LA_MASK = (1 << 4);

    //Defines the location of the VGS_HA (VGS Gate Drive Fault on the A high-side MOSFET) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_VGS_HA_MASK = (1 << 5);

    //Defines the location of the CPUV (Charge Pump Under Voltage Condition) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_CPUV_MASK = (1 << 6);

    //Defines the location of the OTW (Over Temperature Warning) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_OTW_MASK = (1 << 7);

    //Defines the location of the SC_OC (Over Current on phase C sense amplifier (DRV8323xS)) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_SC_OC_MASK = (1 << 8);

    //Defines the location of the SC_OB (Over Current on phase B sense amplifier (DRV8323xS)) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_SB_OC_MASK = (1 << 9);

    //Defines the location of the SC_OA (Over Current on phase A sense amplifier (DRV8323xS)) MASK in the Status 2 register
    static constexpr uint16_t STATUS2_SA_OC_MASK = (1 << 10);




    /* Driver Control Register Masks */

    //Defines the location of the CLR_FLT (Latched Faults) MASK in the Driver Control  register
    static constexpr uint16_t DRIVER_CTRL_CLR_FLT_MASK = (1 << 0);

    //Defines the location of the BRAKE (Low-Side MOSFETs -> 1x PWM) MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_BRAKE_MASK = (1 << 1);

    //Defines the location of the COAST (MOSFETs -> HI-Z) MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_COAST_MASK = (1 << 2);

    //Defines the location of the 1PWM_DIR MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_1PWM_DIR_MASK = (1 << 3);

    //Defines the location of the 1PWM_COM MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_1PWM_COM_MASK = (1 << 4);

    //Defines the location of the PWM_MODE MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_PWM_MODE_MASK = (3 << 5);

    //Defines the location of the OTW_REP MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_OTW_REP_MASK = (1 << 7);

    //Defines the location of the DIS_GDF MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_DIS_GDF_MASK = (1 << 8);

    //Defines the location of the DIS_CPUV MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_DIS_CPUV_MASK = (1 << 9);

    //Defines the location of the RESERVED (Reserved) MASK in the Driver Control register
    static constexpr uint16_t DRIVER_CTRL_RESERVED_MASK = (1 << 10);



    /* Gate Drive HS Register Masks */

    // Defines the location of the IDRIVEN_HS MASK in the Gate Drive HS register
    static constexpr uint16_t GATE_DRIVE_HS_IDRIVEN_HS_MASK = (15 << 0);

    // Defines the location of the IDRIVEP_HS MASK in the Gate Drive HS register
    static constexpr uint16_t GATE_DRIVE_HS_IDRIVEP_HS_MASK = (15 << 4);

    // Defines the location of the LOCK  MASK in the Gate Drive HS register
    static constexpr uint16_t GATE_DRIVE_HS_LOCK_MASK = (7 << 8);



    /* Gate Drive LS Register Masks */

    // Defines the location of the IDRIVEN_LS MASK in the Gate Drive LS register
    static constexpr uint16_t GATE_DRIVE_LS_IDRIVEN_LS_MASK = (15 << 0);

    // Defines the location of the IDRIVEP_LS MASK in the Gate Drive LS register
    static constexpr uint16_t GATE_DRIVE_LS_IDRIVEP_LS_MASK = (15 << 4);

    // Defines the location of the TDRIVE MASK in the Gate Drive LS register
    static constexpr uint16_t GATE_DRIVE_LS_TDRIVE_MASK = (3 << 8);

    // Defines the location of the CBC MASK in the Gate Drive LS register
    // In retry OCP_MODE, for both VDS_OCP and SEN_OCP, the
    // fault is automatically cleared when a PWM input is given
    static constexpr uint16_t GATE_DRIVE_LS_CBC_MASK = (1 << 10);



    /* OCP Register Masks */

    //  Defines the location of the VDS_LVL MASK in the OCP Control register
    static constexpr uint16_t OCP_CTRL_VDS_LVL_MASK = (15 << 0);

    // Defines the location of the OCP_DEG MASK in the OCP Control register
    static constexpr uint16_t OCP_CTRL_OCP_DEG_MASK = (3 << 4);

    // Defines the location of the OCP_MODE MASK in the OCP Control register
    static constexpr uint16_t OCP_CTRL_OCP_MODE_MASK = (3 << 6);

    // Defines the location of the DEAD_TIME MASK in the OCP Control register
    static constexpr uint16_t OCP_CTRL_DEAD_TIME_MASK = (3 << 8);

    // Defines the location of the TRETRY MASK in the OCP Control register
    static constexpr uint16_t OCP_CTRL_TRETRY_MASK = (1 << 10);



    /* CSA Register Masks */

    // Defines the location of the SEN_LVL MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_SEN_LVL_MASK = (3 << 0);

    // Defines the location of the CSA_CAL_C MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_CSA_CAL_C_MASK = (1 << 2);

    // Defines the location of the CSA_CAL_B MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_CSA_CAL_B_MASK = (1 << 3);

    // Defines the location of the CSA_CAL_A MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_CSA_CAL_A_MASK = (1 << 4);

    // Defines the location of the DIS_SEN MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_DIS_SEN_MASK = (1 << 5);

    // Defines the location of the CSA_GAIN MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_CSA_GAIN_MASK = (3 << 6);

    // Defines the location of the LS_REF MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_LS_REF_MASK = (1 << 8);

    // Defines the location of the VREF_DIV MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_VREF_DIV_MASK = (1 << 9);

    // Defines the location of the CSA_FET MASK in the CSA Control register
    static constexpr uint16_t CSA_CTRL_CSA_FET_MASK = (1 << 10);

    typedef enum
    {
        FaultStatus1 = 0x00,
        FaultStatus2 = 0x01,
        DriverControl = 0x02,
        GateDriveHS = 0x03,
        GateDriveLS = 0x04,
        OCPControl = 0x05,
        CSAControl = 0x06
    } RegisterAddress_e;

    typedef enum
    {
        NoFault = (0 << 0), //!< No fault

        /* Fault Status Register 1 */
        FAULT = (1 << 10),  //!< Logic OR of FAULT status registers. Mirrors nFAULT pin
        VDS_OCP = (1 << 9), //!< VDS monitor Over Current Fault
        GDF = (1 << 8),     //!< Under Voltage Lockout Fault
        UVLO = (1 << 7),    //!< Under Voltage Lockout Fault
        OTSD = (1 << 6),    //!< Over Temperature Shutdown Fault
        VDS_HA = (1 << 5),  //!< FET High side, Phase A Over Current fault
        VDS_LA = (1 << 4),  //!< FET Low side, Phase A Over Current fault
        VDS_HB = (1 << 3),  //!< FET High side, Phase B Over Current fault
        VDS_LB = (1 << 2),  //!< FET Low side, Phase B Over Current fault
        VDS_HC = (1 << 1),  //!< FET High side, Phase C Over Current fault
        VDS_LC = (1 << 0),  //!< FET Low side, Phase C Over Current fault

        /* Fault Status Register 2 */
        SA_OC = (1 << 10), //!< Sense Amplifier, Over Current on Phase A Fault
        SB_OC = (1 << 9),  //!< Sense Amplifier, Over Current on Phase B Fault
        SC_OC = (1 << 8),  //!< Sense Amplifier, Over Current on Phase C Fault
        OTW = (1 << 7),    //!< Over Temperature Warning Fault
        CPUV = (1 << 6),   //!< Charge Pump Under voltage Fault
        VGS_HA = (1 << 5), //!< FET High side, Gate Drive fault
        VGS_LA = (1 << 4), //!< FET Low side, Gate Drive fault
        VGS_HB = (1 << 3), //!< FET High side, Gate Drive fault
        VGS_LB = (1 << 2), //!< FET Low side, Gate Drive fault
        VGS_HC = (1 << 1), //!< FET High side, Gate Drive fault
        VGS_LC = (1 << 0)  //!< FET Low side, Gate Drive fault

    } FaultType_e;

    typedef enum
    {
        PWM_MODE_6X = 0 << 5,         //!< 6x PWM Mode
        PWM_MODE_3X = 1 << 5,         //!< 3x PWM Mode
        PWM_MODE_1X = 2 << 5,         //!< 1x PWM Mode
        PWM_MODE_INDEPENDENT = 3 << 5 //!< Independent PWM Mode
    } PwmMode_e;

    typedef enum
    {
        IDRIVEN_20_mA = (0 << 0),    //!< I = 20 mA
        IDRIVEN_60_mA = (1 << 0),    //!< I = 60 mA
        IDRIVEN_120_mA = (2 << 0),   //!< I = 120 mA
        IDRIVEN_160_mA = (3 << 0),   //!< I = 160 mA
        IDRIVEN_240_mA = (4 << 0),   //!< I = 240 mA
        IDRIVEN_280_mA = (5 << 0),   //!< I = 280 mA
        IDRIVEN_340_mA = (6 << 0),   //!< I = 340 mA
        IDRIVEN_380_mA = (7 << 0),   //!< I = 380 mA
        IDRIVEN_520_mA = (8 << 0),   //!< I = 520 mA
        IDRIVEN_660_mA = (9 << 0),   //!< I = 660 mA
        IDRIVEN_740_mA = (10 << 0),  //!< I = 740 mA
        IDRIVEN_880_mA = (11 << 0),  //!< I = 880 mA
        IDRIVEN_1140_mA = (12 << 0), //!< I = 1140 mA
        IDRIVEN_1360_mA = (13 << 0), //!< I = 1360 mA
        IDRIVEN_1640_mA = (14 << 0), //!< I = 1640 mA
        IDRIVEN_2000_mA = (15 << 0)  //!< I = 2000 mA
    } GateDriveSink_e;

    typedef enum
    {
        IDRIVEP_10_mA = (0 << 4),   //!< I = 10 mA
        IDRIVEP_30_mA = (1 << 4),   //!< I = 30 mA
        IDRIVEP_60_mA = (2 << 4),   //!< I = 60 mA
        IDRIVEP_80_mA = (3 << 4),   //!< I = 80 mA
        IDRIVEP_120_mA = (4 << 4),  //!< I = 120 mA
        IDRIVEP_140_mA = (5 << 4),  //!< I = 140 mA
        IDRIVEP_170_mA = (6 << 4),  //!< I = 170 mA
        IDRIVEP_190_mA = (7 << 4),  //!< I = 190 mA
        IDRIVEP_260_mA = (8 << 4),  //!< I = 260 mA
        IDRIVEP_330_mA = (9 << 4),  //!< I = 330 mA
        IDRIVEP_370_mA = (10 << 4), //!< I = 370 mA
        IDRIVEP_440_mA = (11 << 4), //!< I = 440 mA
        IDRIVEP_570_mA = (12 << 4), //!< I = 570 mA
        IDRIVEP_680_mA = (13 << 4), //!< I = 680 mA
        IDRIVEP_820_mA = (14 << 4), //!< I = 820 mA
        IDRIVEP_1000_mA = (15 << 4) //!< I = 1000 mA
    } GateDriveSource_e;

    typedef enum
    {
        TDRIVE_TIME_500ns = (0 << 8),  //!< Drive time of 500-ns
        TDRIVE_TIME_1000ns = (1 << 8), //!< Drive time of 1000-ns
        TDRIVE_TIME_2000ns = (2 << 8), //!< Drive time of 2000-ns
        TDRIVE_TIME_4000ns = (3 << 8)  //!< Drive time of 4000-ns
    } GateDriveTime_e;

    typedef enum
    {
        OC_RETRY_4ms = (0 << 10), //!< VDS_OCP and SEN_OCP retry time is 4 ms
        OC_RETRY_50us = (1 << 10) //!< VDS_OCP and SEN_OCP retry time is 50 us
    } OverCurrentRetryTime_e;

    typedef enum
    {
        DEADTIME_50ns = (0 << 8),  //!< Dead Time of 50-ns
        DEADTIME_100ns = (1 << 8), //!< Dead Time of 100-ns
        DEADTIME_200ns = (2 << 8), //!< Dead Time of 200-ns
        DEADTIME_400ns = (3 << 8)  //!< Dead Time of 400-ns
    } DeadTime_e;

    typedef enum
    {
        OC_MODE_LATCHSHUTDOWN = (0 << 6), //!< Over Current causes a latched fault
        OC_MODE_AUTORETRY = (1 << 6),     //!< Over Current causes an automatic retrying fault
        OC_MODE_REPORTONLY = (2 << 6),    //!< Over Current is report only but no action is taken
        OC_MODE_DISABLED = (3 << 6)       //!< Over Current is not reported and no action is taken
    } OverCurrentMode_e;

    typedef enum
    {
        OC_DEGLITCH_2us = (0 << 4), //!< Over Current deglitch of 2 µs
        OC_DEGLITCH_4us = (1 << 4), //!< Over Current deglitch of 4 µs
        OC_DEGLITCH_6us = (2 << 4), //!< Over Current deglitch of 6 µs
        OC_DEGLITCH_8us = (3 << 4)  //!< Over Current deglitch of 8 µs
    } OverCurrentDeglitch_e;

    typedef enum
    {
        VDSLEVEL_0p060_V = (0 << 0), //!< VDS = 0.060 V
        VDSLEVEL_0p13_V = (1 << 0),  //!< VDS = 0.13 V
        VDSLEVEL_0p20_V = (2 << 0),  //!< VDS = 0.2 V
        VDSLEVEL_0p26_V = (3 << 0),  //!< VDS = 0.26 V
        VDSLEVEL_0p31_V = (4 << 0),  //!< VDS = 0.31 V
        VDSLEVEL_0p45_V = (5 << 0),  //!< VDS = 0.45 V
        VDSLEVEL_0p53_V = (6 << 0),  //!< VDS = 0.53 V
        VDSLEVEL_0p60_V = (7 << 0),  //!< VDS = 0.6 V
        VDSLEVEL_0p68_V = (8 << 0),  //!< VDS = 0.68 V
        VDSLEVEL_0p75_V = (9 << 0),  //!< VDS = 0.75 V
        VDSLEVEL_0p94_V = (10 << 0), //!< VDS = 0.94 V
        VDSLEVEL_1p13_V = (11 << 0), //!< VDS = 1.13 V
        VDSLEVEL_1p30_V = (12 << 0), //!< VDS = 1.3 V
        VDSLEVEL_1p50_V = (13 << 0), //!< VDS = 1.5 V
        VDSLEVEL_1p70_V = (14 << 0), //!< VDS = 1.7 V
        VDSLEVEL_1p88_V = (15 << 0)  //!< VDS = 1.88 V
    } OverCurrentVdsLevel_e;

    typedef enum
    {
        CSA_GAIN_5VpV = (0 << 6),  //!< 5 V per V
        CSA_GAIN_10VpV = (1 << 6), //!< 10 V per V
        CSA_GAIN_20VpV = (2 << 6), //!< 20 V per V
        CSA_GAIN_40VpV = (3 << 6)  //!< 40 V per V
    } CSA_Gain_e;

    typedef enum
    {
        SENSE_OC_LEVEL_0p25_V = (0 << 0), //!< Sense OCP 0.25 V
        SENSE_OC_LEVEL_0p5_V = (1 << 0),  //!< Sense OCP 0.5 V
        SENSE_OC_LEVEL_0p75_V = (2 << 0), //!< Sense OCP 0.75 V
        SENSE_OC_LEVEL_1p0_V = (3 << 0)   //!< OSense OCP 1.0 V
    } SenseOCLevel_e;

    typedef enum
    {
        SENSE_AMP_NUMBER_1 = 1, //!< Shunt amplifier number 1
        SENSE_AMP_NUMBER_2 = 2, //!< Shunt amplifier number 2
        SENSE_AMP_NUMBER_3 = 3  //!< Shunt amplifier number 3
    } SenseAmpNumber_e;

    typedef enum
    {
        REG_LOCK = 6 << 8,   //!< Lock all registers but this one and address 0x02h bits 0:2
        REG_UNLOCK = 3 << 8, //!< Unlock all registers
    } RegisterLockMode_e;

    typedef enum
    {
        FAULT_STATUS_1_MASK = (FaultStatus1 << 11),   //!< Fault Status Register 1
        FAULT_STATUS_2_MASK = (FaultStatus2 << 11),   //!< Fault Status Register 2
        DRIVER_CONTROL_MASK = (DriverControl << 11),  //!< Driver Control Register
        GATE_DRIVE_HS_MASK  = (GateDriveHS << 11),    //!< Gate Drive HS Control Register
        GATE_DRIVE_LS_MASK  = (GateDriveLS << 11),    //!< Gate Drive LS Control Register
        OCP_MASK            = (OCPControl << 11),     //!< OCP Control Register
        CSA_MASK            = (CSAControl << 11),     //!< CSA Register
    } RegisterNameMask_e;

    // Construtor
    DRV8323(SPIDevice *spi, GPIO_t enable_pin, GPIO_t nFault_pin);

    bool Init(); // Init DRV and Setup for Use
    void EnableDriver(); // Enable Driver Power
    void DisableDriver(); // Disable Driver Power
    void FaultReset();    // Reset DRV Faults

    /* Helper Functions */

    // Under Voltage Lockout
    void EnableUVLO();
    void DisableUVLO();

    // Gate Drive Fault
    void EnableGDF();
    void DisableGDF(); 

    // Over temperature warning Fault Report
    void EnableOTWReport();
    void DisableOTWReport(); 

    // Set PWM Mode
    void SetPWMMode(PwmMode_e pwm_mode);

    // HI-Z Coast
    void EnableCoast();
    void DisableCoast();

    // Reset any latched faults
    void ClearLatchedFaults();

    // Register locking to prevent accidental writes on SPI
    void LockRegisters();
    void UnlockRegisters();

    // Gate Drive Values
    void SetIDriveP_HS(GateDriveSource_e I_source);
    void SetIDriveN_HS(GateDriveSink_e I_sink);

    // Cycle by cycle operation
    void EnableCBC();
    void DisableCBC();

    // Gate Drive Time Values
    void SetTDrive(GateDriveTime_e time);

    // Gate Drive Values
    void SetIDriveP_LS(GateDriveSource_e I_source);
    void SetIDriveN_LS(GateDriveSink_e I_sink);

    // Overcurrent Retry time
    void EnableTRETRY_4ms();
    void EnableTRETRY_50us();

    // Set Dead Time
    void SetDeadTime(DeadTime_e dead_time);

    // Set Overcurrent Protection Mode
    void SetOCPMode(OverCurrentMode_e OCP_mode);

    // Set Overcurrent Deglitch Mode
    void SetOCPDeglitch(OverCurrentDeglitch_e OCP_deg);

    // Set Overcurrent VDS Level
    void SetVDSLevel(OverCurrentVdsLevel_e VDS_lvl);

    // Set Current Sense FET
    void SetCSAPosInput_SPx();
    void SetCSAPosInput_SHx();

    // Set Current Sense Amplifier Reference Voltage.  (Enable for Bidirectional, Disable for Unidirectional)
    void EnableVREFDiv();
    void DisableVREFDiv();

    // Set Low side FET measurement.  Enable = SHx to SNx, Disable SHx to SPx (default)
    void EnableLSRef();
    void DisableLSRef();

    // Current Sense Amplifier Gain
    void SetCSAGain(CSA_Gain_e gain);

    // Overcurrent Sense Fault
    void EnableOCSenseFault();
    void DisableOCSenseFault();

    // Current Sense Amplifier Calibration Phase A
    void EnableCSA_CAL_A();
    void DisableCSA_CAL_A();

    // Current Sense Amplifier Calibration Phase B
    void EnableCSA_CAL_B();
    void DisableCSA_CAL_B();

    // Current Sense Amplifier Calibration Phase C
    void EnableCSA_CAL_C();
    void DisableCSA_CAL_C();

    // Overcurrent Protection Sense Level
    void SetOCPSenseLevel(SenseOCLevel_e lvl);

    uint16_t test()
    {
        return ReadRegister(GateDriveLS);
    }
protected:

    uint16_t ReadRegister(uint16_t address);
    void WriteRegister(uint16_t address, uint16_t value);
    void UpdateRegister(uint16_t address, uint16_t mask, uint16_t value);
    bool DriverEnabled();   // Is Driver Currently Enabled?

    SPIDevice *spi_;  // SPI Device
    GPIO_t enable_;   // Enable PIN
    GPIO_t nFault_;   // Fault PIN


};

#endif // DRV8323_H_