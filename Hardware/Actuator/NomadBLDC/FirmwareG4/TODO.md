# Firmware TODO:
1.  Remove all print statements and cleanup old serial
2.  Verify CAN Messaging
3.  Keep alive support
4.  Speed Controller ***
5.  Position limits on output?
6.  Gate driver fault checking and reporting ****
7.  Add Bus current ***
8.  Verify optimization flags -O3 breaks something.  Looks like PWM timers don't start?
9.  Remove all HAL timers
10. Circular Buffer for general use. e.g. HDLC packet receive could be circular.  Right now is linear
11. Customize HDLC Command Callback.  And make consistent with CAN Bus once implemented
12. FET Thermistor/Motor Thermistor Variables in Config
13. Get Rid of math_ops.h
14. Adjustable LUT encoder offset size.  Update position sensor % modulus when this happens
15. Save stats for loop cycle time if in debug mode etc.
16. Error out if missing deadlines etc
17. Make RMS Current period configurable?  For now it is 1/10.  Reasonable memory use for 60.0s window.
19. Add ErrorTransition FSM Function
20. Add ADC Start Poll/Read decouple.  Start ADC do other stuff read later
21. SPI DMA? ADC DMA?
23. Remove "Filtered" Phase Currents/Voltages
