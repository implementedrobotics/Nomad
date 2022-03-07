import nomad
import time

DEVICE = "/dev/pcanusbfd32"

config = nomad.can_config()
print(config.mode_fd)

config.bitrate = 1000000
config.d_bitrate = 5000000
config.sample_point=0.8
config.d_sample_point=0.625
config.clock_freq=80000000
config.mode_fd = 1
can = nomad.pcan_device()
can.open(DEVICE, config, True)
can.clear_filters()
can.add_filter(1,2)

servo = nomad.nomad_bldc(2, 0x10, can)
if(servo.connect() == False):
    print("[ERROR]: Unable to connect to Nomad Servo!\n")
else:
    print("Nomad Servo: " + str(servo.get_servo_id()) + " Connected!")
servo.print_state()

print("Zeroing Output")
servo.zero_output()
val = servo.set_control_mode(10)
print("Return")
print(str(val))
i = 1
while i < 10000:
    print(i)
    servo.closed_loop_torque_command(5.0, 0.0, 0.0,0.0,0.0)
    time.sleep(0.001)
    i += 1

servo.set_control_mode(1)