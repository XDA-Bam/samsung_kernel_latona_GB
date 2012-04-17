#!/sbin/ext/busybox sh

/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/busybox.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/swap-SD.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/properties.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/check-ver.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/install.sh
# Tweaks script moved to /system/etc/init.d
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/fix_permissions-patch.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/scripts.sh


read sync < /data/sync_fifo
rm /data/sync_fifo
setprop cm.filesystem.ready 1
