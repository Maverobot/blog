---
title:  Setup hotspot on Ubuntu 14.04
date:   2018-06-24T13:57:00+02:00
categories: ["Tutorials"]
---

* Disable WIFI and plug in an internet cable to your laptop so that your Ubuntu is connect to a wired internet and wireless is disabled.
* Go to Network Icon on top panel -> Edit Connections …, then click the "Add" button in the pop-up window.
* Choose Wi-Fi from the drop-down menu when you’re asked to choose a connection type:
* In next window, do:
  * Type in a connection name. (This name will be used later)
  * Type in a SSID
  * Select mode: `Infrastructure`
  * Device MAC address: select your wireless card from drop-down menu.
* Go to Wi-Fi Security tab, select security type `WPA & WPA2 Personal` and set a password.
* Go to IPv4 Settings tab, from Method drop-down box select `Shared to other computers`
* Open the configuration file by ```gksu gedit /etc/NetworkManager/system-connections/wifi-hotspot```, find the line ```mode=infrastructure``` and change it to ```mode=ap```.
