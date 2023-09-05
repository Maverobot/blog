---
title:  "How to configure DHCP server for local network"
date:   2018-07-06T14:30:00+02:00
categories: ["Tutorials"]
---

Here are the steps for configuring a DHCP server in a local network.

Set static IP to the second network card `enp9s3` (this name can be different on a different PC) on the DHCP server PC. Run
```
 sudo vi /etc/network/interfaces
```
Edit the file according to this sample:
```
# interfaces(5) file used by ifup(8) and ifdown(8)
auto lo
iface lo inet loopback

# The other network interface
auto eno123
iface eno123 inet dhcp

# DHCP server interface
auto enp9s3
iface enp9s3 inet static
address 15.0.3.10
netmask 255.255.255.0
gateway 15.0.3.1
dns-nameservers 8.8.4.4 8.8.8.8
```
Restart network service by
```
sudo service networking restart
```
Setup DHCP server
Install dnsmasq package
```
sudo apt-get install dnsmasq
```
Check out what network interfaces exist. Run `ifconfig -a`. The output should be something similar to this.
```
enp9s3    Link encap:Ethernet  HWaddr 91:2b:2e:59:cf:df
          UP BROADCAST MULTICAST  MTU:1500  Metric:1
          RX packets:558 errors:0 dropped:0 overruns:0 frame:0
          TX packets:1616 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:67665 (67.6 KB)  TX bytes:260734 (260.7 KB)
          Interrupt:20 Memory:f7200000-f7220000
eno123    Link encap:Ethernet  HWaddr 27:02:1f:2a:f1:ac
          inet addr:192.168.122.123  Bcast:192.168.122.255  Mask:255.255.255.0
          inet6 addr: fe80::279a:1f01:c7b:78b1/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:396017 errors:0 dropped:0 overruns:0 frame:0
          TX packets:192658 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:361146437 (361.1 MB)  TX bytes:26264534 (26.2 MB)
          Memory:f7100000-f717ffff
lo        Link encap:Local Loopback
          inet addr:127.0.0.1  Mask:255.0.0.0
          inet6 addr: ::1/128 Scope:Host
          UP LOOPBACK RUNNING  MTU:65536  Metric:1
          RX packets:753842 errors:0 dropped:0 overruns:0 frame:0
          TX packets:753842 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:49121050 (49.1 MB)  TX bytes:49121050 (49.1 MB)
```
`eno123` is the other ethernet interface, which has nothing to do with our configuration here.

Now configure DHCP server by editing:
```
sudo vi /etc/dnsmasq.conf
```
Some lines should be uncommented and adjusting such as the sample here:
```
interface=enp9s3
bind-interfaces
dhcp-range=15.0.3.50,15.0.3.53,24h
```
Start dnsmasq daemon.
```
sudo ifconfig enp9s3 up
sudo systemctl start dnsmasq.service
```
