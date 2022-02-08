# Ubuntu 20.04 Installation Instructions for UP XTreme SBC

## Choose Desired Ubuntu Image and Download

* [Ubuntu Desktop Edition](https://releases.ubuntu.com/20.04/ubuntu-20.04.3-desktop-amd64.iso) (This is appropriate if you are planning to do development and debugging ON the NOMAD Hardware directly with GUI based tools)

* [Ubuntu Live Server Edition](https://releases.ubuntu.com/20.04/ubuntu-20.04.3-live-server-amd64.iso) (This is appropriate if you are planning to setup the NOMAD Hardware to run optimally on a working/final version)

## Create USB Installation Media
* Burn the desired Ubuntu image onto a USB Drive.  If you are on a windows platform try: [balenaEtcher](https://etcher.io).  This is the software I have used and works very well for creating bootable media.

## Install

* Place USB installation media into an available USB port on the UP XTreme.
  * If media does not boot:
    * Verify that you have an acceptable boot order in your BIOS settings (USB has priority to internal storage).
    * Also you can press **F7** at boot to access the boot selection menu.  Select the USB drive that you created in the previous step.  This should start the installation process.

* Select your language and choose ```Install Ubuntu```
* Select ```Minimal installation``` and uncheck ```Download updates while installing Ubuntu```.  Leave ```Install third-party software...``` unchecked.
* If there is a previous isntallation of Ubuntu on the machine.  Choose ```Erase disk and install Ubuntu``` and click ```Install Now```.
* Select your location and continue.
* Enter your desired ```Name```, ```Computer name```, ```Username``` and ```Password```.  Check ```Log in automatically``` and click ```Continue```.
* Be patient while Ubuntu installs.  If using emmc this should take around 5-10 mins.
* Once completed, remove installation media and press ```Enter``` to restart.

* When Ubuntu loads for the first time on the restart.  Complete last questions to your own preferences.  You may also be asked to update software.  Go ahead and do this now.

## Install Ubuntu kernel 5.4.0 from aaeon/up PPA (Kernel version 5.4.0 is the latest for UP Boards as of 2/7/2022)
  * Kernel version 5.4.0 is the latest for UP Boards as of 2/7/2022.
  * 
Disable Auto Logoff/lock

