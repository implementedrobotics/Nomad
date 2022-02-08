# PREEMPT_RT Kernel Instructions for UP XTreme SBC

#### Please make sure to complete steps in Ubuntu OS Installation Wiki first!


## Install dependencies
```
sudo apt install build-essential* fakeroot dpkg-dev perl libssl-dev* bc gnupg dirmngr libncurses5-dev* libelf-dev* flex* bison* git*
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

Compile kernel modules and install

```sudo make modules_isntall -j8```

``` sudo make install -j8```

Verify correct kernel is installed

```cd /boot```

```ls```

Should see...

## Update Grub and Restart

```sudo update-grub```

```sudo reboot```


