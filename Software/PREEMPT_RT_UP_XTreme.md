# PREEMPT_RT Kernel Instructions for UP XTreme SBC

#### Please make sure to complete steps in Ubuntu OS Installation Wiki first!


## Install dependencies
```
sudo apt install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf git
```

## Download Kernel Sources for UP

```mkdir ~/kernel && cd ~/kernel```

```git clone https://github.com/AaeonCM/ubuntu-bionic-up.git```

```cd ubuntu-bionic-up```

```git checkout hwe-5.4-upboard```

## Apply RT Patch to Kernel Sources
### Display kernel version information
``` 
$ make kernelversion
5.4.65
```

Locate on [kernel.org](http://https://kernel.org/) RT patch closest or equal to this version for best success in patching(less code differences)

For instance with kernel version 5.4.65 use **5.4.66-rt38**

```wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.66-rt38.patch.xz```

Apply patch

```unxz -cd patch-5.4.66-rt38.patch.xz | patch -p1```

## Kernel Configuration
Copy the current kernel config for defaults

```
cp /boot/config-5.4.0-1-generic .config
```

### Option 1 (CLI Based Configuration)
Edit .config with your favorite editor, i.e.,

```xdg-open .config``` - GUI based editor

or

```nano .config``` - CLI Editor

Set the following RT parameters:

```CONFIG_HIGH_RES_TIMERS=y```

```CONFIG_PREEMPT_RT=y```

```CONFIG_HZ_1000=y```

```CONFIG_HZ=1000```

```CONFIG_OF=n```

```CONFIG_AUFS_FS=n```

#### Optional CPU Maximum Performance Configuration 

Disable CPU Idle state

```CONFIG_CPU_IDLE=n```

```CONFIG_INTEL_IDLE=n```

Remove power saving governor states

```CONFIG_CPU_FREQ_DEFAULT_GOV_POWERSAVE=n```

```CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND=n```

```CONFIG_CPU_FREQ_DEFAULT_GOV_CONSERVATIVE=n```

```CONFIG_CPU_FREQ_GOV_POWERSAVE=n```

```CONFIG_CPU_FREQ_GOV_ONDEMAND=n```

```CONFIG_CPU_FREQ_GOV_CONSERVATIVE=n```

Set default governer state => PERFORMANCE

```CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE=y```

```CONFIG_CPU_FREQ_GOV_PERFORMANCE=y```

These flags can also be set with cpufrequtils

### Option 2 (GUI Based Configuration)

```make menuconfig```
Using the search **/** shortcut locate and set all of the parameters from Option 1

TODO: Where in menu config options are + screenshots?

## Build Kernel

Compile kernel sources

```make -j8```

Compile kernel modules and install.

```sudo make INSTALL_MOD_STRIP=1 modules_install -j8```

```sudo make install -j8```

Verify correct kernel is installed

```cd /boot```

```ls```

```vmlinuz-5.4.65-rt38+```

Also verify size of initial ramdisk, can have debug symbols that will create a very large/slow initial ram disk
```
$ ls -lh /boot
49M initrd.ing-5.4.65-rt38+
```

## Update Grub and Restart

```sudo update-grub```

```sudo reboot```

## Verify newly compiled RT Kernel is active

```
$ uname -a
Linux nomad 5.4.65-rt38+ #1 SMP PREEMPT_RT Wed Feb 9 12:32:15 CST 2022 x86_64 x86_64 x86_64 GNU/Linux
```

## Setup User Permissions

Running user applications with real time scheduling priorities (SCHED_FIFO and SCHED_RR) requires increased security permissions.  There are 3 options to achieve this:

1.  Run application as the super user/root (Not recommended)
2.  Change permissions on the executable binary directly using ```setcap```

e.g.
```
sudo setcap cap_sys_nice+ep <EXECUTABLE BINARY>
sudo setcap cap_ipc_lock+ep <EXECUTABLE BINARY>
```

Finally for the last and preferred option
3.  Add a ```realtime``` user group with permissions and add default user to this group

Add realtime group.

```sudo groupadd realtime```

Add current user to realtime group.

```sudo usermod -aG realtime ${USER}```

Add security permissions for realtime group to access low latency options
```
sudo nano /etc/security/limits.d/99-realtime.conf
```

Add the following lines and save:

```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 104857600
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 104857600
```

**Note:** For memlock options this is the size in *Bytes* (100 * 1024 * 1024 = **100MB** in this case) that is allowed to be locked into physical RAM and not paged out to virtual memory.  If you attempt to allocate more than this amount you will get an error which should be handled.  Otherwise you will **NOT** be guaranteed the low latency you are expecting.

**Note:** Also you can pass **-1** as the byte value. Now memlock will impose **NO** allocation limits.  Be careful with this, if you go over the amount of physical RAM of the hardware there will be errors/crash.

Reboot to apply new permissions to the user.

```sudo reboot```

Once system has restarted let's verify correct permissions.

* Check user is in realtime group.

```
$ groups | grep -o realtime
realtime
```

* Check memlock allocation limits.

```
$ ulimit -l
104857600
```






