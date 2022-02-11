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

```
cd ~/drivers/peak
wget https://www.peak-system.com/quick/BasicLinux
```

