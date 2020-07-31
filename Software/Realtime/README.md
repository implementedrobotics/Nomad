

# REALTIME


## Notes:

TODO:
Jetson Xavier AGX RT Setup Here:

# Disable GUI Boot:
sudo systemctl set-default multi-user.target
# Enable GUI Boot:
sudo systemctl set-default graphical.target

Sort these later:
https://github.com/kozyilmaz/nvidia-jetson-rt/blob/master/docs/README.03-realtime.md
https://devtalk.nvidia.com/default/topic/1050012/jetson-nano/preempt-rt-patches-for-jetson-nano/
https://developer.ridgerun.com/wiki/index.php?title=Xavier/JetPack_4.1/Flashing_Board/Flash_Kernel

Services:
https://www.freedesktop.org/software/systemd/man/systemd.service.html

For Cross Compile:
sudo apt-get install zlib1g:i386


On some systems(L4T/Jetson) you may also be required to set:
```
sudo sysctl -w kernel.sched_rt_runtime_us=-1
```
This is required for instance on setting SCHED_FIFO.

Flash with CPU Isolate:

```
# Isolate last 4 CPUs for our RT task.  Let OS use first 4 CPUs
sudo ./flash.sh -k kernel -k kernel-dtb -C "isolcpus=4,5,6,7" jetson-xavier mmcblk0p1
```

To be able to set realtime priorities as non root user you will need:

Enable Max Performance and Fan:
```
sudo /usr/sbin/nvpmodel -m 0

# As root
echo 255 > /sys/devices/pwm-fan/target_pwm 
```

Debug CPUs Isolated:
```
ps -eo psr,command | tr -s " " | grep "^ [0|1]" # Should be minimal task on these cores, or OUR task ideally
lscpu | grep '^CPU.s' # Show total CPUs
taskset -cp 1 # CPU Affinity for first Kernel Task (Shoud be 0-3/The non-islated CPUS)
```
Test it:


```
git clone https://git.kernel.org/pub/scm/linux/kernel/git/clrkwllms/rt-tests.git
cd rt-tests/
make

sudo ./cyclictest -a -t -n -p99
sudo ./cyclictest --mlockall --smp --priority=99 --distance=0
```

```
sudo setcap cap_sys_nice+ep <EXECUTABLE BINARY>
sudo setcap cap_ipc_lock+ep <EXECUTABLE BINARY>
or
Edit /etc/security/capability.conf and add cap_ipc_lock          nomad
```


When assigning a process to run a specific core you may need the compiler command:
```
-D _GNU_SOURCE
```

or put this macro in the source code of the threading file:
```
#define __USE_GNU
```



# For Grub Based Systems
To isolate certain cores from the OS so that they will not be used be any other task/processed:

To reserve CPUs 2 and 3:
```
vim /etc/default/grub
```

Add the line:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=2,3"
```

Update Grub:
```
sudo update-grub
```


# Resources:

https://bytefreaks.net/programming-2/cc-set-affinity-to-threads-example-code

