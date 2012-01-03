#!/sbin/ext/busybox sh

/sbin/ext/busybox sh /sbin/boot/busybox.sh
#/sbin/ext/busybox sh /sbin/boot/cwm.sh
/sbin/ext/busybox sh /sbin/boot/cwmfix.sh
/sbin/ext/busybox sh /sbin/boot/properties.sh
#/sbin/ext/busybox sh /sbin/boot/check-ver.sh
/sbin/ext/busybox sh /sbin/boot/install.sh
#/sbin/ext/busybox sh /sbin/boot/tweaks.sh # Tweaks script moved to /system/etc/init.d
/sbin/ext/busybox sh /sbin/boot/scripts.sh

read sync < /data/sync_fifo
rm /data/sync_fifo
