---
layout: post
title:  "How to install CUDA on a laptop with optimus"
date:   2018-06-24 13:50:00 +0200
categories: Tutorials 
---

I tried it many times and finally installed CUDA 7.5 on my laptop (Nvidia GeForce 750M) with optimus. It was very tricky but these steps worked for me.

**Step 1: Switch to kubuntu**. 

(Some other linux desktop should also work, e.g. lubuntu, mate) According to [this tutorial](https://www.pugetsystems.com/labs/articles/NVIDIA-CUDA-GPU-computing-on-a-modern-laptop-629/), both Ubuntu Unity and Gnome3 desktops were very susceptible to black-screen states and/or corrupted workspaces. KDE didn’t seem to suffer from these problems. Following commands can help you to do the switch:

    sudo apt-get install kubuntu-desktop build-essential emacs synaptic dkms
    sudo apt-get upgrade
    sudo shutdown -r now

In my case after making sure Kubuntu working properly I removed the default ubuntu desktop by using

    sudo apt-get remove ubuntu-desktop

**Step 2: Install the nvidia driver for your graphics card.** 

NVidia 352.79 is the one I used. This step is actually optional. Skipping this step without any nvidia driver installed should also work.

**Step 3: Download and install [cuda toolkit deb local installer file](https://developer.nvidia.com/cuda-downloads)**. 

Because the .run installer did not work for me, I **strongly** recommend using the **deb** file.

    sudo dpkg -i cuda-repo-ubuntu1404-7-5-local_7.5-18_amd64.deb
    sudo apt-get update
    sudo apt-get install cuda

**Step 4: Possible troubleshooting**.

During the installation(`sudo apt-get install cuda`), an error of unmet dependencies happened to me. I tracked down the problem and uninstalled the conflicting package by following this tutorial by @osdf [here](http://askubuntu.com/a/572910/514666).

**Step 5: Load module for CUDA**.

Run this command and you are hopefully free to use CUDA on your machine.

    modprobe nvidia_uvm


