# PCAN-M.2 Device setup and verification

The PCAN-M.2 is the chosen device to enable real-time CAN-FD communications with the Nomad leg actuators and power distribution systems. It supports upto 4 independent CAN channels for maximum bandwidth in the control loop.

By default it is supported with the mainline kernel driver for netdev/SocketCAN support.  However we wil be using the chardev/proprietary PCAN driver for best performance.  

TODO: Installation Photos Here.  And show CAN Terminations.

## Install PCAN-M.2 Driver for Ubuntu 20.04.04

### Install dependencies

```
sudo apt update
sudo apt install libpopt-dev
```

Create a new PEAK driver directory.

```
mkdir -p ~/drivers/peak
cd ~/drivers/peak
```

Download latest driver. (8.14.0 as of 2/11/2022)

```
wget https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.14.0.tar.gz
```

Extract driver files.

```
tar -xvzf peak-linux-driver-8.14.0.tar.gz
cd peak-linux-driver-8.14.0/
```

Build the driver.

```
make -C driver PCI=PCI_SUPPORT PCIEC=NO_PCIEC_SUPPORT DNG=NO_DONGLE_SUPPORT USB=NO_USB_SUPPORT ISA=NO_ISA_SUPPORT PCC=NO_PCCARD_SUPPORT
```

Build libraries

```
make -C lib
```

Build test utilities (Optional)

```
make -C test
```

Install driver

```
sudo make -C driver install
```

Install libraries

```
sudo make -C lib install
```

Install optional test utilities

```
sudo make -C test install
```

Load driver

```
sudo modprobe pcan
```

## Install PCAN Basic API

Download latest [PCAN Basic API](https://www.peak-system.com/quick/BasicLinux)

Extract files to ```~/drivers/peak```

```
cd ~/drivers/peak/PCAN-Basic_Linux-4.5.4/libpcanbasic/pcanbasic
make clean
make -j4
sudo make install
```

## Verify successful driver installation

```
for f in /sys/class/pcan/pcanpcifd1/*; do [ -f $f ] && echo -n "`basename $f` = " && cat $f; done
```

```
./lspcan -T -t -i
```

## Optimize driver latency

Edit pcan.conf configuration file

```
sudo nano /etc/modprobe.d/pcan.conf
```

Add the following three ```options``` parameters

```
options pcan fdirqtl=1
options pcan fdirqcl=1
options pcan fdusemsi=1
```

The resulting file should now be:

```
# pcan - automatic made entry, begin --------
# if required add options and remove comment
# options pcan type=isa,sp
options pcan fdirqtl=1
options pcan fdirqcl=1
options pcan fdusemsi=1
install pcan modprobe --ignore-install pcan
# pcan - automatic made entry, end ----------
```

Reload pcan driver

```
sudo rmmod pcan
sudo modprobe pcan
```



## TODO: CAN Bus Timings for Nomad Server 1/5mbps
## TODO: Testing


## Uninstall PCAN-M.2 Driver for Ubuntu 20.04.04

In the event of needing to purge the driver for either reinstall, or switch back to netdev/SocketCAN driver:

```
cd ~/drivers/peak/peak-linux-driver-8.14.0/ 
sudo make uninstall
rmmod pcan
sudo reboot
```

