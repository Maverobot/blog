---
title:  "How to reassign a static IP address with dnsmasq"
date:   2018-07-06 15:30:00 +0200
categories: Tutorials 
---
This post is a revised version of [link][original post].

The following steps should be followed:

1. Edit the file `/etc/dnsmasq.conf` on your router, and update the mac address associated 
   with the intended ip address.
   ```
   dhcp-host=<mac address>,<ip address>
   ```
   
2. Stop `dnsmasq` service
   ```
   sudo systemctl stop dnsmasq.service 
   ```

3. Next shutdown networking on the new client machine or the command dhclient -v -r might get the job done. Be aware, that this step will make the client lose the connection.

4. On the router, edit the file `/var/lib/misc/dnsmasq.leases`, and delete the pre-existing lease for the client machine.

5. Start dnsmasq service on the router,
   ```
   sudo systemctl start dnsmasq.service 
   ```
6. Start networking on the client machine or `sudo dhclient`.


[original post]: http://trentsonlinedocs.xyz/how_to_reassign_a_static_ip_address_with_dnsmasq/