#!/bin/bash

# Sicherstellen, dass das Skript als Root ausgeführt wird
if [ "$(id -u)" -ne 0 ]; then
  echo "Bitte führen Sie dieses Skript als Root aus (mit sudo)."
  exit 1
fi

# Benutzer nach SSID und Passwort fragen
read -p "Geben Sie die SSID für das WLAN ein: " ssid
read -p "Geben Sie das Passwort für das WLAN ein: " wpa_passphrase

# Installiere notwendige Pakete
apt update
apt install -y hostapd dnsmasq

# Konfiguriere hostapd
cat > /etc/hostapd/hostapd.conf <<EOL
interface=wlan0
driver=nl80211
ssid=$ssid
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
wpa_passphrase=$wpa_passphrase
EOL

# Hostapd konfigurieren
sed -i 's|#DAEMON_CONF=""|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd

# Konfiguriere dnsmasq
mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
cat > /etc/dnsmasq.conf <<EOL
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
EOL

# Statische IP-Adresse für wlan0 konfigurieren
cat >> /etc/dhcpcd.conf <<EOL
interface wlan0
static ip_address=192.168.4.1/24
nohook wpa_supplicant
EOL

# IP-Weiterleitung aktivieren
sed -i 's|#net.ipv4.ip_forward=1|net.ipv4.ip_forward=1|' /etc/sysctl.conf

# NAT (Network Address Translation) einrichten
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sh -c "iptables-save > /etc/iptables.ipv4.nat"

# iptables-Regel beim Booten wiederherstellen
sed -i '/^exit 0/i iptables-restore < /etc/iptables.ipv4.nat' /etc/rc.local

# Hostapd und dnsmasq starten und aktivieren
systemctl unmask hostapd
systemctl enable hostapd
systemctl start hostapd
systemctl start dnsmasq

# Neustart des Raspberry Pi
echo "Konfiguration abgeschlossen. Raspberry Pi wird neu gestartet."
reboot
