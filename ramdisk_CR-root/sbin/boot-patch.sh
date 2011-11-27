#!/sbin/ext/busybox sh

echo 1 > /sys/class/leds/button-backlight/brightness
/sbin/ext/busybox sh /sbin/boot/busybox.sh
#/sbin/ext/busybox sh /sbin/boot/cwm.sh
/sbin/ext/busybox sh /sbin/boot/install.sh
/sbin/ext/busybox sh /sbin/boot/properties.sh
###/sbin/ext/busybox sh /sbin/boot/app2sd.sh
/sbin/ext/busybox sh /sbin/boot/ext4speed.sh
###/sbin/ext/busybox sh /init.qcom.post_boot.sh
# Tweaks script moved to /system/etc/init.d
#/sbin/ext/busybox sh /sbin/boot/tweaks.sh
/sbin/ext/busybox sh /sbin/boot/scripts.sh
echo 0 > /sys/class/leds/button-backlight/brightness

read sync < /data/sync_fifo
rm /data/sync_fifo

setprop cm.filesystem.ready 1



