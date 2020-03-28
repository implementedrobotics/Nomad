# PC/GUI TODO:
1.  Make serial handler class event based.
2.  Add timeouts to serial reads properly
5.  Setup for more event based signalling
6.  Add live plotting support
7.  Add Keep Alive Packet
8.  Read back motor status (Voltage, Phase Currents, etc)
9.  Add support for checking port still open
10. Send mutex/locking
11. Finish motor calibration.  Pass and save all measurements
12. Signals for status feedback to handler


Install Notes
sudo update
upgrade pip - pip3 install --upgrade pip
pip3 install pyqt5
pip3 install pyqtchart

sudo apt-get install python3-pyqt5
sudo apt-get install pyqt5-dev-tools
sudo apt-get install qttools5-dev-tools

pip3 install pyserial

update to 3.7
sudo apt-get install python3.7
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
sudo update-alternatives --config python3 and select 2
vim /usr/bin/gnome-terminal -> change to python3.6
sudo apt-get install python-apt