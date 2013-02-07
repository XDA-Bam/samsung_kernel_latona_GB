#!/sbin/ext/busybox sh

/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/busybox.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/swap-SD.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/properties.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/fix_wifi.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/install.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/fix_permissions-patch.sh
/system/bin/logwrapper /sbin/ext/busybox sh /sbin/boot/init_d.sh


read sync < /data/sync_fifo
rm /data/sync_fifo
setprop cm.filesystem.ready 1
