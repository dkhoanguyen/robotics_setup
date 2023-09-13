from arm32v6/alpine

# Install packages
RUN apk update && apk add hostapd iw dhcp vim iptables

# Configure Hostapd (default will be open)
ADD confs/hostapd_confs/open.conf /etc/hostapd/hostapd.conf
# Configure DHCPD
ADD confs/dhcpd.conf /etc/dhcp/dhcpd.conf
RUN touch /var/lib/dhcp/dhcpd.leases

# Configure networking
ADD confs/interfaces /etc/network/interfaces
ADD confs/iptables.sh /iptables.sh
ADD confs/iptables_off.sh /iptables_off.sh

# Copy and execute init file
ADD confs/start.sh /start.sh
CMD ["/bin/sh", "/start.sh"]
