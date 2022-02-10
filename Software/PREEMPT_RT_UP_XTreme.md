# PREEMPT_RT Kernel Instructions for UP XTreme SBC on Ubuntu 20.04.03

#### Please make sure to complete steps in Ubuntu OS Installation Wiki first!


## Install dependencies

```
sudo apt install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf git
```

## Download kernel sources from UP

```mkdir ~/kernel && cd ~/kernel```

```git clone https://github.com/AaeonCM/ubuntu-bionic-up.git```

```cd ubuntu-bionic-up```

```git checkout hwe-5.4-upboard```

## Apply RT patch to kernel Sources


### Display kernel version information

``` 
$ make kernelversion
5.4.65
```

Locate on [kernel.org](http://https://kernel.org/) RT patch closest or equal to this version for best success in patching(less code differences)

For instance with kernel version **5.4.65** use **5.4.66-rt38**

```wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.66-rt38.patch.xz```

Apply patch

```unxz -cd patch-5.4.66-rt38.patch.xz | patch -p1```

## Kernel configuration
Copy the current kernel config for defaults

```
cp /boot/config-5.4.0-1-generic .config
```

### Option 1 (CLI based configuration)
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

#### Optional CPU maximum performance configuration 

Disable CPU idle state

```CONFIG_CPU_IDLE=n```

```CONFIG_INTEL_IDLE=n```

Remove power saving governor states

```CONFIG_CPU_FREQ_DEFAULT_GOV_POWERSAVE=n```

```CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND=n```

```CONFIG_CPU_FREQ_DEFAULT_GOV_CONSERVATIVE=n```

```CONFIG_CPU_FREQ_GOV_POWERSAVE=n```

```CONFIG_CPU_FREQ_GOV_ONDEMAND=n```

```CONFIG_CPU_FREQ_GOV_CONSERVATIVE=n```

Set default governor state => PERFORMANCE

```CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE=y```

```CONFIG_CPU_FREQ_GOV_PERFORMANCE=y```

These flags can also be set more dynamically.  See ```cpufrequtils``` setup instructions [below](#cpu-performance).

### Option 2 (GUI based configuration)

```make menuconfig```
Using the search **/** shortcut locate and set all of the parameters from Option 1

TODO: Where in the menu config options are + screenshots?

## Build kernel

Compile kernel sources

```make -j8```

Compile kernel modules and install.

```sudo make INSTALL_MOD_STRIP=1 modules_install -j8```

```sudo make install -j8```

Verify correct kernel is installed

```cd /boot```

```ls```

```vmlinuz-5.4.65-rt38+```

Also verify size of initial ramdisk, it is possible debug symbols were included that will create a very large/slow initial ram disk
```
$ ls -lh /boot
49M initrd.ing-5.4.65-rt38+
```

## Update grub and restart

```sudo update-grub```

```sudo reboot```

## Verify newly compiled RT kernel is active

```
$ uname -a
Linux nomad 5.4.65-rt38+ #1 SMP PREEMPT_RT Wed Feb 9 12:32:15 CST 2022 x86_64 x86_64 x86_64 GNU/Linux
```

## Setup user permissions

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

Add a realtime group.

```sudo groupadd realtime```

Add the current user to the realtime group.

```sudo usermod -aG realtime ${USER}```

Add security permissions for realtime group to access low latency options
```
sudo nano /etc/security/limits.d/99-realtime.conf
```

Add the following lines and save:

```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

**Note:** For memlock options this is the size in *KBytes* (100 * 1024 = **100MB** in this case) that is allowed to be locked into physical RAM and not paged out to virtual memory.  If you attempt to allocate more than this amount you will get an error which should be handled.  Otherwise you will **NOT** be guaranteed the low latency you are expecting.

**Note:** Also you can pass ```-1``` as the byte value. Now memlock will impose **NO** allocation limits.  Be careful with this, if you go over the amount of physical RAM of the hardware there will be errors/crashes.

Reboot to apply new permissions to the user.

```sudo reboot```

Once the system has restarted let's verify correct permissions.

* Check if the user is in the realtime group.

```
$ groups | grep -o realtime
realtime
```

* Check memlock allocation limits.

```
$ ulimit -l
104857600
```

## CPU performance

If opted not to include these CPU performance settings in the kernel config, it can be setup as below.  Benefit of this method is the ability to enable/disable these settings without kernel recompilation.

Install ```cpufrequtils``` utility to set CPU governor and scaling parameters:

```sudo apt install cpufrequtils```

Disable load dependent ```ondemand``` CPU governor.

```sudo systemctl disable ondemand```

Enable ```cpufrequtils``` service.

```sudo systemctl enable cpufrequtils```

Set CPU governor to ```performance``` mode.

```sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'```

Reload system services to apply changes.

```sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils```

Set CPU DMA latency permissions.
```sudo nano /etc/udev/rules.d/55-cpu_dma_latency.rules```

Add the following to this file:
```KERNEL=="cpu_dma_latency", NAME="cpu_dma_latency", MODE="0666"```

Now by echoing ```0``` to ```/dev/cpu_dma_latency``` we can on demand adjust processor idle C states for minimal latency

Reboot for udev rules to take effect.

```sudo reboot```

## Test and verify Real-Time latency

### Setup rt-tests 

```sudo apt install rt-tests```

### Run tests

**Note:** If the below commands give errors, make sure user/group realtime has been set up.  Alternatively, run with ```sudo```.

Run the cyclic test to analyze max latency and jitter

```cyclictest -a -t -n -p99```

Expected Non RT System Results:

```
T: 0 ( 1946) P:99 I:1000 C: 21097 Min: 3 Act: 8 Avg: 7 Max: 7756
T: 1 ( 1947) P:99 I:1500 C: 14064 Min: 3 Act: 8 Avg: 7 Max: 1697
T: 2 ( 1948) P:99 I:2000 C: 10548 Min: 3 Act: 7 Avg: 6 Max: 1582
T: 3 ( 1949) P:99 I:2500 C: 8439  Min: 3 Act: 9 Avg: 8 Max: 1556
T: 4 ( 1950) P:99 I:3000 C: 7032  Min: 3 Act: 5 Avg: 6 Max: 3502
T: 5 ( 1951) P:99 I:3500 C: 6027  Min: 3 Act: 6 Avg: 8 Max: 4108
T: 6 ( 1952) P:99 I:4000 C: 5274  Min: 3 Act: 7 Avg: 8 Max: 6850
T: 7 ( 1953) P:99 I:4500 C: 4688  Min: 3 Act: 6 Avg: 7 Max: 5412
```

Expected RT System Results:
  
```
T: 0 ( 1946) P:99 I:1000 C: 21097 Min: 1 Act: 2 Avg: 2 Max: 45
T: 1 ( 1947) P:99 I:1500 C: 14064 Min: 1 Act: 3 Avg: 2 Max: 45
T: 2 ( 1948) P:99 I:2000 C: 10548 Min: 1 Act: 1 Avg: 2 Max: 43
T: 3 ( 1949) P:99 I:2500 C: 8439  Min: 1 Act: 2 Avg: 2 Max: 33
T: 4 ( 1950) P:99 I:3000 C: 7032  Min: 1 Act: 2 Avg: 2 Max: 36
T: 5 ( 1951) P:99 I:3500 C: 6027  Min: 1 Act: 2 Avg: 2 Max: 50
T: 6 ( 1952) P:99 I:4000 C: 5274  Min: 1 Act: 2 Avg: 2 Max: 46
T: 7 ( 1953) P:99 I:4500 C: 4688  Min: 1 Act: 2 Avg: 2 Max: 41
```
  
Ideally you will see Min/Average latency  **<2us** with a worst case maximum latency  **<100us**(best results under 50us)

### Check latency under stress

The previous test performed for "optimal" latency.  The following test will run the same test but will also stress the CPU simultaneously to show more real world results under load.

Change to a preferred directory and checkout stress tests sources

```
git clone https://github.com/QiayuanLiao/Ubuntu-RT-UP-Board.git
```

```cd ~/Ubuntu-RT-UP-Board/test```

**Note:** In rt-test.sh, adjust the number of CPU cores to stress on line 2, ```--cpu``` parameter. Also the number of cores to run RT threads on line 11, ```cores``` parameter.

Run test.
```sudo sh ./rt-test.sh```

Show latency plot.

```
xdg-open plot.png
```

Expected RT System Results:

![plot](https://user-images.githubusercontent.com/39070514/153476110-3db9f1d9-6a76-4565-a7a8-6b46b8519570.png)

## CPU isolation

### Configure
To help further meet our expected RT latency requirements we can isolate specific cores away from the kernel/os.  The UP XTreme used in NOMAD has 8 independent cores available.  Let's isolate the last 4 cores away from the linux scheduler.  Tasks will have to be manually pinned to these CPUs, which we control, via CPU affinity or C library calls.

Set grub parameters to isolate CPUs.  

**Note:** Indexing is 0-based.

To reserve last 4 cores 4,5,6,7 do:

```nano /etc/default/grub```

Update the following line to:

```GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=4,5,6,7"```

Now the linux scheduler will only use the first 4 cores.

Update grub and restart:

```
sudo update-grub
sudo reboot
```

### Verify

Should show the total number of CPU cores available.

```
$ lscpu | grep '^CPU.s'
$ CPU(s): 8
```

Show CPU affinity for the first kernel task.  If setup was successful the affinity list should only include the non-isolated cores.  In this case 0,1,2, and 3.

```
$ taskset -cp 1
$ pid 1's current affinity list: 0-3
```

If you have gotten here you have successfully set up the Real-Time system on the UP board!
