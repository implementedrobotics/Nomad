# Errata TODO:

1. Fix USB pad short
2. Send PCBway correct p/n for the CAN connector
3. Add AS5147 Encoder to part list
4. Add 120 ohm CAN termination resistor to part list
5. Change CAN resistor value in schematic to 120 ohm
6. Add AB input from encoder for optional quadruture input+incread encoder noise immunity from hall sensor noise
7. Add 100nf decoupling to NRST pin for increades noise immunity.
8. Change current sense resistor to 0.5mOhm.  Early thermal test indicate we could push higher peaks than 40A.
9. Change bus link capactitors to: C3216X5R1H106K160AB for better derating values
10. Add thermistor for FET Temps
11. Change +5V buck converter from DRV to +3.3V (Update feedback resistor network)
12. Change CAN chip to a 3.3V FD CAN variant (https://www.ti.com/store/ti/en/p/product/?p=TCAN334GDCNT)
13. Remove USB
