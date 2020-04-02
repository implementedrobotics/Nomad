# Firmware TODO:
1.  Controller priorities.  Make sure communications priorities are straight. 
2.  Remove all print statements and cleanup old serial
3.  Move serial port creation back to thread
5.  Verify CAN Messaging
6.  Keep alive support
7.  Add Support for Gear Ratios
8.  Speed Controller
9.  Position limits on output?
10. Clean up class access public/privates
12. Send mutex
14. Refactor Measurement Routines
16. Move voltage ADC timer reading to own timer
18. Should make sure controller is in "IDLE" state before resaving configs.  Also should restart to be sure on save
19. Gate driver fault checking and reporting