

# CONTROLLERS


## Notes:

To be able to set realtime priorities as non root user you will need:

```
sudo setcap cap_sys_nice+ep <EXECUTABLE BINARY>
```

On some systems(L4T/Jetson) you may also be required to set:
```
sudo sysctl -w kernel.sched_rt_runtime_us=-1
```
This is required for instance on setting SCHED_FIFO.

Also to note:

When assigning a process to run a specific core you may need the compiler command:
```
-D _GNU_SOURCE
```

or put this macro in the source code of the threading file:
```
#define __USE_GNU
```

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

