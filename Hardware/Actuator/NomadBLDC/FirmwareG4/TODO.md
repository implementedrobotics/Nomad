# Firmware TODO:
1.  Controller priorities.  Make sure communications priorities are straight. 
2.  Remove all print statements and cleanup old serial
4.  Verify CAN Messaging
5.  Keep alive support
6.  Speed Controller ***
7.  Position limits on output?
8.  Clean up class access public/privates
10. Refactor Measurement Routines (Tasks)
12. Triple current measurements
13. Gate driver fault checking and reporting ****
15. Add Bus current ***
16. Lo Pri - Make RMS Current period configurable?  For now it is 1/10.  Reasonable memory use for 60.0s window.
19. Make UART to cpp
20. RMS Current Calculator -> Low Priority Task
21. Verify current measurements are accurate
22. HDLC command callback to UART
23. Performance profile Cordic vs arm_math
24. Verify optimization flags -O3 breaks something
25. Thermistor Lookup Table vs log calculate

