---
title:  "How to boot from grub-rescue shell"
date:   2018-09-23T16:32:00+02:00
categories: ["Tutorials"]
thumbnailImage: https://i.ibb.co/ys52Ftt/fire-man.jpg
coverImage: https://i.ibb.co/ys52Ftt/fire-man.jpg
---

Today I tried to fix an old issue with an old laptop having dual boot - windows / ubuntu. This issue occurred one day out of nowhere (someone from forums said it was caused by a windows update).

While the windows system still boots, the laptop cannot boot into ubuntu and always gets stuck at a grub rescue shell with an error:
```
file /boot/grub/x86_64-efi/normal.mod not found
```

The following steps helped me to boot into linux:

* First, use command `ls` to see what partitions are available. The console will show a list of names such as `(dh0,gpt1)`, `(dh0,gpt2)` etc.
* Then, try to use `ls` command to find your linux partition by
```
ls (dh0,gpt1)/
```
In my case, `ls (dh0,gpt9)` gave me a list of familar folder names, e.g. `boot`, `usr`, `opt` etc.

* Type the following commands and you should be able to boot.
```
set root=(hd0,gpt9)
set prefix=(hd0,gpt9)/boot/grub
insmod linux
linux /vmlinuz root=/dev/sda9 ro
initrd /initrd.img
boot
```

After writing down this little post, I found out this [page](https://www.linux.com/learn/how-rescue-non-booting-grub-2-linux%20%20) containing much better explanation of this process
