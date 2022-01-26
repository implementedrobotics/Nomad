# NOMAD BLDC - Firmware

# Installation

The following instructions are for setup of a development environment for building firmware code on Ubuntu 18.04.

## Install GCC ARM Compiler:

```
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi
```

## Install STLink Flash Utility:

```
sudo apt-get install git build-essential libusb-1.0.0-dev cmake
cd $HOME
wget https://github.com/stlink-org/stlink/releases/download/v1.7.0/stlink_1.7.0-1_amd64.deb
```

To use the STLINK flash utility without root you must add a new group for ```stlink```:

```
sudo addgroup stlink
sudo adduser <USERNAME> stlink
```
You will need to logout or reboot for the group modification to take effect.

Now verify that your programmer is found:
```
st-info --probe
```

Should report:
```
Found 1 stlink programmers
```

## Install NOMAD BLDC Firmware Source

Assuming you have not already cloned the repository:

```
cd $HOME
git clone https://github.com/implementedrobotics/Nomad
```

Change to the Firmware directory:

```
cd $HOME/Nomad/Hardware/Actuator/NomadBLDC/Firmware
```

If installation is successful you should now be able to do without error:

```
make
```

## Flash the NOMAD BLDC Binary

Connect your STLINK programmer to the NOMAD BLDC 8-Pin Programming Connector and your PC.

Now flash the build firmware:

```
make flash
```

Alternatively you can manually flash with:

```
cd $HOME/Nomad/Hardware/Actuator/NomadBLDC/Firmware/BUILD
st-flash write NomadBLDC.bin 0x8000000
```

## VS Code Setup:
TODO
