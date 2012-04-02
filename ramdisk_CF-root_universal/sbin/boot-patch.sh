#!/sbin/ext/busybox sh

/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/busybox.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/cwmfix.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/properties.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/check-ver.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/install.sh
# Tweaks script moved to /system/etc/init.d
#/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/tweaks.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/scripts.sh

#fix_permissions patch
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/fix_permissions-patch.sh

read sync < /data/sync_fifo
rm /data/sync_fifo
setprop cm.filesystem.ready 1
