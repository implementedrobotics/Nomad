# NOMAD
NOMAD is a 12-DOF quadruped walking robot using Linear and Nonlinear Optimization Control techniques.

  - Linear and Nonlinear Model Predictive Control
  - Simultaneous Localization and Mapping
  - Remote Control Teleoperation

# Installation

The following instructions are for setup of a development environment for building executable code for Ubuntu 18.04.

Install [qpOASES 3.2.1](https://projects.coin-or.org/qpOASES/wiki/QpoasesDownload):
```
unzip qpOASES-3.2.1.zip
cd qpOASES-3.2.1
```
There seems to be a compilation/linking bug in 3.2.1. To fix it search for and  delete "-DUSE_LONG_INTEGERS" in make_linux.mk under CPPFLAGS.
```
make
sudo cp bin/libqpOASES.so /usr/local/lib
sudo cp bin/libqpOASES.a /usr/local/lib

sudo cp -r include/* /usr/local/include

```

Install [Eigen 3.3.7](http://eigen.tuxfamily.org/index.php?title=Main_Page)
```
tar xvzf eigen-eigen-323c052e1731.tar.gz
cd eigen-eigen-323c052e1731
sudo cp -r Eigen /usr/local/include
```
 We also use some unsupported features of Eigen as well:
 ```
 sudo cp -r unsupported /usr/local/include
 ```

Clone or Fork Repository:
```
git clone https://github.com/implementedrobotics/Nomad
```

Setup CMake Build Sources
```
cd Nomad/Software
mkdir Build/
cd Build/
cmake ..
```

### Todos

 - Write MORE Tests

License
----

MIT


