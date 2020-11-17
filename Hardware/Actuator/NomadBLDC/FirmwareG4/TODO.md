# Firmware TODO:
* Keep alive support
* Speed Controller ***
* Position limits on output?
* Gate driver fault checking and reporting ****
* Verify optimization flags -O3 breaks something.  Looks like PWM timers don't start?
* Remove all HAL timers
* Circular Buffer for general use. e.g. HDLC packet receive could be circular.  Right now is linear
* Customize HDLC Command Callback.  And make consistent with CAN Bus once implemented
* FET Thermistor/Motor Thermistor Variables in Config*. Adjustable LUT encoder offset size.  Update position sensor % modulus when this happens
* Save stats for loop cycle time if in debug mode etc.
* Error out if missing deadlines etc
* Make RMS Current period configurable?  For now it is 1/10.  Reasonable memory use for 60.0s window.
* Add ErrorTransition FSM Function
* Add ADC Start Poll/Read decouple.  Start ADC do other stuff read later
* SPI DMA? ADC DMA?
* Add Theta Electrical + Speed Feedword in FOC
* FD CAN Peripheral Abort Request
* FD CAN Error Handler Function
* Peripheral Base Class
* Move Load/Save Config out of Motor Controller
* Double check too big ints in functions.  I.E. Motor config has 32-bit ints that could be uint8
* Profile/Optimize CAN Interrupt Handling Function
* PCAN FD 80mhz bit timings - 
1mbps = Prescaler = 1, tseg1 = 63(prop 47), tseg2 = 16, SJW = 16, SP = 80%
5mbps = Prescaler = 1, tseg1 = 9(prop 0),  tseg2 = 6,  SJW = 6,  SP = 62.5%
