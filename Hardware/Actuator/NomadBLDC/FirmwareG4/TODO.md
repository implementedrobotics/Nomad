# Firmware TODO:
1.  Remove all print statements and cleanup old serial
2.  Verify CAN Messaging
3.  Keep alive support
4.  Speed Controller ***
5.  Position limits on output?
6.  Clean up class access public/privates
7.  Refactor Measurement Routines (Tasks)
8.  Gate driver fault checking and reporting ****
9.  Add Bus current ***
10. Lo Pri - Make RMS Current period configurable?  For now it is 1/10.  Reasonable memory use for 60.0s window.
11. RMS Current Calculator -> Low Priority Task
12. Verify optimization flags -O3 breaks something.  Looks like PWM timers don't start?
13. Remove all HAL timers
14. Circular Buffer for general use. e.g. HDLC packet receive could be circular.  Right now is linear
15. Customize HDLC Command Callback.  And make consistent with CAN Bus once implemented
16. FET Thermistor/Motor Thermistor Variables in Config
17. Get Rid of math_ops.h
18. Adjustable LUT encoder offset size