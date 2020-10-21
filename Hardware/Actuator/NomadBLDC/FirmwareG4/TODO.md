# Firmware TODO:
2.  Remove all print statements and cleanup old serial
4.  Verify CAN Messaging
5.  Keep alive support
6.  Speed Controller ***
7.  Position limits on output?
8.  Clean up class access public/privates
10. Refactor Measurement Routines (Tasks)
13. Gate driver fault checking and reporting ****
15. Add Bus current ***
16. Lo Pri - Make RMS Current period configurable?  For now it is 1/10.  Reasonable memory use for 60.0s window.
20. RMS Current Calculator -> Low Priority Task
24. Verify optimization flags -O3 breaks something
26. Remove all HAL timers
27. Circular Buffer for general use. e.g. HDLC packet receive could be circular.  Right now is linear
28. Optimize for 40khz
29. Customize HDLC Command Callback.  And make consistent with CAN Bus once implemented
30. FET Thermistor/Motor Thermistor Variables in Config
31. Get Rid of math_ops.h
32. Adjustable LUT encoder offset size
34. Add #DEFINE / ENUM State definitions for IDS
35. Update Polling ADC to new Devices
