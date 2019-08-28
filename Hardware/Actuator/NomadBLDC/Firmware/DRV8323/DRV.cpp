#include "mbed.h"
#include "DRV.h"

DRV832x::DRV832x(SPI *spi, DigitalOut *cs)
{
    _spi = spi;
    _cs = cs;
    _cs->write(1);
    wait_us(10);
    _spi->format(16, 1);
    _spi->frequency(500000);
}

uint16_t DRV832x::spi_write(uint16_t val)
{
    _cs->write(0);
    wait_us(10);
    uint16_t reply = _spi->write(val);
    _cs->write(1);
    return reply;
}

int DRV832x::read_FSR1(void)
{
    uint16_t val = (1 << 15) | FSR1;
    return spi_write(val);
}

int DRV832x::read_FSR2(void)
{
    uint16_t val = (1 << 15) | FSR2;
    return spi_write(val);
}

int DRV832x::read_register(int reg)
{
    return spi_write((1 << 15) | (reg << 11));
}

void DRV832x::write_register(int reg, int val)
{
    spi_write((reg << 11) | val);
}
void DRV832x::write_DCR(int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT)
{
    uint16_t val = (DCR << 11) | (DIS_CPUV << 9) | (DIS_GDF << 8) | (OTW_REP << 7) | (PWM_MODE << 5) | (PWM_COM << 4) | (PWM_DIR << 3) | (COAST << 2) | (BRAKE << 1) | CLR_FLT;
    spi_write(val);
}
void DRV832x::write_HSR(int LOCK, int IDRIVEP_HS, int IDRIVEN_HS)
{
    uint16_t val = (HSR << 11) | (LOCK << 8) | (IDRIVEP_HS << 4) | IDRIVEN_HS;
    spi_write(val);
}
void DRV832x::write_LSR(int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS)
{
    uint16_t val = (LSR << 11) | (CBC << 10) | (TDRIVE << 8) | (IDRIVEP_LS << 4) | IDRIVEN_LS;
    spi_write(val);
}
void DRV832x::write_OCPCR(int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL)
{
    uint16_t val = (OCPCR << 11) | (TRETRY << 10) | (DEAD_TIME << 8) | (OCP_MODE << 6) | (OCP_DEG << 4) | VDS_LVL;
    spi_write(val);
}
void DRV832x::write_CSACR(int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL)
{
    uint16_t val = (CSACR << 11) | (CSA_FET << 10) | (VREF_DIV << 9) | (LS_REF << 8) | (CSA_GAIN << 6) | (DIS_SEN << 5) | (CSA_CAL_A << 4) | (CSA_CAL_B << 3) | (CSA_CAL_C << 2) | SEN_LVL;
    spi_write(val);
}

void DRV832x::print_faults(void)
{
    uint16_t val1 = read_FSR1();
    wait_us(10);
    uint16_t val2 = read_FSR2();
    wait_us(10);
    if (val1 & (1 << 10))
    {
        printf("\n\rFAULT\n\r");
    }

    if (val1 & (1 << 9))
    {
        printf("VDS_OCP\n\r");
    }
    if (val1 & (1 << 8))
    {
        printf("GDF\n\r");
    }
    if (val1 & (1 << 7))
    {
        printf("UVLO\n\r");
    }
    if (val1 & (1 << 6))
    {
        printf("OTSD\n\r");
    }
    if (val1 & (1 << 5))
    {
        printf("VDS_HA\n\r");
    }
    if (val1 & (1 << 4))
    {
        printf("VDS_LA\n\r");
    }
    if (val1 & (1 << 3))
    {
        printf("VDS_HB\n\r");
    }
    if (val1 & (1 << 2))
    {
        printf("VDS_LB\n\r");
    }
    if (val1 & (1 << 1))
    {
        printf("VDS_HC\n\r");
    }
    if (val1 & (1))
    {
        printf("VDS_LC\n\r");
    }

    if (val2 & (1 << 10))
    {
        printf("SA_OC\n\r");
    }
    if (val2 & (1 << 9))
    {
        printf("SB_OC\n\r");
    }
    if (val2 & (1 << 8))
    {
        printf("SC_OC\n\r");
    }
    if (val2 & (1 << 7))
    {
        printf("OTW\n\r");
    }
    if (val2 & (1 << 6))
    {
        printf("CPUV\n\r");
    }
    if (val2 & (1 << 5))
    {
        printf("VGS_HA\n\r");
    }
    if (val2 & (1 << 4))
    {
        printf("VGS_LA\n\r");
    }
    if (val2 & (1 << 3))
    {
        printf("VGS_HB\n\r");
    }
    if (val2 & (1 << 2))
    {
        printf("VGS_LB\n\r");
    }
    if (val2 & (1 << 1))
    {
        printf("VGS_HC\n\r");
    }
    if (val2 & (1))
    {
        printf("VGS_LC\n\r");
    }
}

void DRV832x::enable_gd(void)
{
    uint16_t val = (read_register(DCR)) & (~(0x1 << 2));
    write_register(DCR, val);
}

void DRV832x::disable_gd(void)
{
    uint16_t val = (read_register(DCR)) | (0x1 << 2);
    write_register(DCR, val);
}

void DRV832x::calibrate(void)
{
    // Calibrate one amplifier at a time.  Could we do all at once?

    // CSA Amplifier A
    set_dc_cal_mode(CSA_AMP_ID_A, CSA_CAL_A_NO_LOAD); // Disconnect and Calibrate
    wait_us(500);
    set_dc_cal_mode(CSA_AMP_ID_A, CSA_CAL_A_LOAD); // Reconnect
    wait_us(500);

    // CSA Amplifier B
    set_dc_cal_mode(CSA_AMP_ID_A, CSA_CAL_B_NO_LOAD); // Disconnect and Calibrate
    wait_us(500);
    set_dc_cal_mode(CSA_AMP_ID_A, CSA_CAL_B_LOAD); // Reconnect
    wait_us(500);

    // CSA Amplifier C
    set_dc_cal_mode(CSA_AMP_ID_A, CSA_CAL_C_NO_LOAD); // Disconnect and Calibrate
    wait_us(500);
    set_dc_cal_mode(CSA_AMP_ID_A, CSA_CAL_C_LOAD); // Reconnect
    wait_us(500);
}
void DRV832x::set_dc_cal_mode(uint16_t amp_id, uint16_t mode)
{
    uint16_t data = read_register(CSACR); // Read Data

    if (amp_id == CSA_AMP_ID_A) // Clear the bits
    {
        data &= (~CSA_CAL_A_BITS);
    }
    else if (amp_id == CSA_AMP_ID_B)
    {
        data &= (~CSA_CAL_B_BITS);
    }
    else if (amp_id == CSA_AMP_ID_C)
    {
        data &= (~CSA_CAL_C_BITS);
    }
    data |= mode; // Set the bits

    write_register(CSACR, data); // Write SPI Data


}