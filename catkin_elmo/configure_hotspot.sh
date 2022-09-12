#! /usr/bin/bash

nmcli con add type wifi ifname wlan0 con-name Elmo autoconnect yes ssid Elmo
nmcli con modify Elmo 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
nmcli con modify Elmo wifi-sec.key-mgmt wpa-psk
nmcli con modify Elmo wifi-sec.psk "asdf"

