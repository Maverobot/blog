+++
title = "Deprecated linux commands"
author = ["Zheng Qu"]
date = 2020-07-18T00:00:00+02:00
lastmod = 2020-07-18T12:50:05+02:00
tags = ["sed"]
categories = ["cli"]
draft = false
weight = 2001
+++

## What are deprecated? {#what-are-deprecated}

-   `ifconfig`
-   `netstats`
-   `route`


## What are the better options? {#what-are-the-better-options}

-   `ip`
-   `ss`
-   `lsof -iTCP`


## Basic usages {#basic-usages}


### Command `ip` {#command-ip}

Show / manipulate routing, network devices, interfaces and tunnels

-   Show ip adresses: `ip address` or `ip a`
-   Show network devices: `ip link` or `ip l`
-   Show routing table entry: `ip route` or `ip r`
-   Use color output: for example `ip -c a`, `ip -c l`


### Command `ss` {#command-ss}

Another utility to investigate sockets

-   Show processes with listening TCP sockets: `ss -plt`
-   For more infomation: `man ss`


### Command `lsof` {#command-lsof}

[Here](https://www.tecmint.com/10-lsof-command-examples-in-linux/) is a good article about this command.

For finding processes running on port `8080`:

```bash
lsof -iTCP:8080
```
