# Ubuntu 20.04 Installation Instructions for UP XTreme SBC

## Choose Desired Ubuntu Image and Download

* [Ubuntu Desktop Edition](https://releases.ubuntu.com/20.04/ubuntu-20.04.3-desktop-amd64.iso) (This is appropriate if you are planning to do development and debugging ON the NOMAD Hardware directly with GUI based tools)

* [Ubuntu Live Server Edition](https://releases.ubuntu.com/20.04/ubuntu-20.04.3-live-server-amd64.iso) (This is appropriate if you are planning to setup the NOMAD Hardware to run optimally on a working/final version)

## Create USB Installation Media
* Burn the desired Ubuntu image onto a USB Drive.  If you are on a windows platform try: [balenaEtcher](https://etcher.io).  This is the software I have used and works very well for creating bootable media.

## Install Operating System

* Place USB installation media into an available USB port on the UP Board.
  * If media does not boot:
    * Verify that you have an acceptable boot order in your BIOS settings (USB has priority to internal storage).
    * Also you can press **F7** at boot to access the boot selection menu.  Select the USB drive that you created in the previous step.  This should start the installation process.

* Select your language and choose ```Install Ubuntu```
* Select ```Minimal installation``` and uncheck ```Download updates while installing Ubuntu```.  Leave ```Install third-party software...``` unchecked.
* If there is a previous installation of Ubuntu on the machine.  Choose ```Erase disk and install Ubuntu``` and click ```Install Now```.
* Select your location and continue.
* Enter your desired ```Name```, ```Computer name```, ```Username``` and ```Password```.  Check ```Log in automatically``` and click ```Continue```.
* Be patient while Ubuntu installs.  If using emmc this should take around 5-10 mins.
* Once completed, remove installation media and press ```Enter``` to restart.

* When Ubuntu loads for the first time on the restart.  Complete last questions to your own preferences.  You may also be asked to update software.  Go ahead and do this when prompted.
* Reboot System

## Optional Configuration
* Disable Auto-Lock and Auto-Logoff:  
  * In ```Ubuntu Settings``` -> ```Privacy``` -> ```Blank Screen Delay=Never```, ```Automatic Screen Lock=Disabled``` and ```Lock Screen on Suspend=Disabled```
  * Additionally  ```Ubuntu Settings``` -> ```Power``` -> ```Suspend & Power Button``` -> ```Automatic Suspend=Off```

## Install Ubuntu kernel 5.4.0 from aaeon/up PPA (Kernel version 5.4.0 is the latest for UP Boards as of 2/7/2022)
Kernel version 5.4.0 is the latest for UP Boards as of 2/7/2022.

Open a command line terminal

Add PPA repository for AaeonCM

```sudo add-apt-repository ppa:aaeon-cm/5.4-upboard```

Update

```sudo apt update```

Remove default installed generic kernels.  When prompted ```Abort Kernel Removal``` enter ```No```

```sudo apt autoremove --purge 'linux-.*generic'```

Install custom upboard kernel from PPA repository. (Note: 20.04 and 18.04 use the same base kernel)

```sudo apt install linux-generic-hwe-18.04-5.4-upboard```

Install updates:

```sudo apt dist-upgrade -y```

Update grub configuration:

```sudo update-grub```

Reboot

```sudo reboot```

Verify correct kernel version after system reboots.  For example:

```
$ uname -a 
Linux nomad 5.4.0-1-generic #0~upboard5-Ubuntu SMP Fri Jan 7 11:53:57 UTC 2022 x86_64 x86_64 x86_64 GNU/Linux
```

## Enable HAT/GPIO Functionality
To use GPIO header pins for GPIO, PWM, SPI, I2C and UART the following must be done to set special user permissions to access the functionality.

**NOTE:** This will add new rules to adjust user permissions and could cause a security risk.  In the case of Nomad we are not concerned about this risk.  Please make sure you are comfortable adjusting these permissions

### Install upboard-extras
```sudo apt install upboard-extras```

This adds the following udev rules to the system:
```
/lib/udev/rules.d/99-gpio.rules
/lib/udev/rules.d/99-i2c.rules
/lib/udev/rules.d/99-leds.rules
/lib/udev/rules.d/99-spi.rules
/lib/udev/rules.d/99-ttyAMA0.rules
```

Also it will add the following groups:
```
gpio
leds
spi
i2c
```

Verify these new permission groups have been added:
For instance to check existence of gpio group:
```
$ getent group | grep gpio
gpio:x:134:
```
## Add user to groups

Add current user to **gpio** group

```
sudo usermod -a -G gpio ${USER}
```

Add current user to **leds** group

```
sudo usermod -a -G leds ${USER}
```

Add current user to **spi** group

```
sudo usermod -a -G spi ${USER}
```

Add current user to **i2c** group

```
sudo usermod -a -G i2c ${USER}
```

Add current user to **dialout** (Serial/UART) group

```
sudo usermod -a -G dialout ${USER}
```

Reboot to apply permissions
```
sudo reboot
```
### Add PWM Support in user space (WIP)

```
wget https://github.com/up-board/up-community/blob/main/files/PWM%20rules_udev/pwm%20udev_v0.3.zip?raw=true
```

* Extract zip files to a folder
* Copy file 99-pwm.rules to /lib/udev/rules.d
* Reboot

Test it
```
./pwm.sh
```









