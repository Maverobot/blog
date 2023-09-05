---
title:  Setup Kinect on Ubuntu 14.04
date:   2018-06-24T13:53:00+02:00
categories: ["Tutorials"]
---

The following steps are needed for Kinect to work properly in Ubuntu 14.04.

  - Prerequisites
  - OpenNI
  - Kinect Sensor Module
  - Possible user group tuning

## Prerequisites
The necessary packages:
```
$ sudo apt-get install git build-essential python libusb-1.0-0-dev freeglut3-dev openjdk-7-jdk
```
Optional:
```
sudo apt-get install doxygen graphviz mono-complete
```

## OpenNI
* Check out from git:
```
git clone https://github.com/OpenNI/OpenNI.git
```

* Compilation:
```
cd OpenNi
cd Platform/Linux/CreateRedist
chmod +x RedistMaker
./RedistMaker
```
* Installation:
```
cd ../Redist/OpenNI-Bin-Dev-Linux-[xxx]
sudo ./install.sh
```

## Kinect Sensor Module
  * Check out from git:

  ```
  git clone https://github.com/ph4m/SensorKinect
  ```

  * Compilation:

  ```
  cd OpenNi
  cd Platform/Linux/CreateRedist
  chmod +x RedistMaker
  ./RedistMaker
  ```

  * Installation:

  ```
  cd ../Redist/Sensor-Bin-Linux-[xxx]
  sudo ./install.sh
  ```

## User group tuning
Sometimes the sample programs can only run with root. In this case, there are two alternative solutions.

  * Solution 1:
    ```sudo gedit /etc/udev/rules.d/55-primesense-usb.rules ```, and then change the owner or group. (not tested)

  * Solution 2:
    ```sudo adduser $USER root```, then log out and log in.
