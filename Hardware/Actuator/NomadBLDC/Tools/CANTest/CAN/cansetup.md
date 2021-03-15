1. Test it

sudo ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on
sudo ifconfig can0 up


ifconfig - should see interface etc

Loop back etc?

How to add a service Ubuntu 20+


Create a Unit file to define a systemd service:

vim socketcan-interface.service

[Unit]
Description=SocketCAN interface can0 [1mbps/2mbps]
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on ; /sbin/ifconfig can0 up
ExecReload=/sbin/ifconfig can0 down ; /sbin/ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on ; /sbin/ifconfig can0 up
ExecStop=/sbin/ifconfig can0 down

[Install]
WantedBy=multi-user.target


sudo cp myservice.service /etc/systemd/system/socketcan-interface.service
sudo chmod 644 /etc/systemd/system/socketcan-interface.service


Test Service
make sure interface is not active
sudo ifconfig can0 down

sudo systemctl start socketcan-interface

Check Status

sudo systemctl status socketcan-interface

<INSERT CORRECT OUTPUT HERE>

// Stop or restart can interface
sudo systemctl stop socketcan-interface
sudo systemctl restart socketcan-interface


Enable on boot:

sudo systemctl enable socketcan-interface

Reboot system and check status

sudo systemctl status socketcan-interface

also:

ifconfig and look for can0


