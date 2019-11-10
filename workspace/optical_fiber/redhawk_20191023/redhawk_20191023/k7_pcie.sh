#!/bin/sh

module="k7_pcie"
major=`awk "\\$2==\"$module\" {print \\$1}" /proc/devices`

rm -f /dev/$module
mknod /dev/$module c $major 0
chmod 644 /dev/$module
