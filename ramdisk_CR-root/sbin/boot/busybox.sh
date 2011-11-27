# magic busybox

# mount system and rootfs r/w
/sbin/ext/busybox mount -o remount,rw /system;
/sbin/ext/busybox mount -t rootfs -o remount,rw rootfs;

# make sure we have /system/xbin
/sbin/ext/busybox mkdir -p /system/xbin

# if symlinked busybox in /system/bin or /system/xbin, remove them
LINK=$(/sbin/ext/busybox find /system/bin/busybox -type l);
if /sbin/ext/busybox [ $LINK = "/system/bin/busybox" ]; then
  /sbin/ext/busybox rm -rf /system/bin/busybox;
fi;
LINK=$(/sbin/ext/busybox find /system/xbin/busybox -type l);
if /sbin/ext/busybox [ $LINK = "/system/xbin/busybox" ]; then
  /sbin/ext/busybox rm -rf /system/xbin/busybox;
fi;

# if real busybox in /system/bin, move to /system/xbin
if /sbin/ext/busybox [ -f /system/bin/busybox ]; then
  /sbin/ext/busybox rm -rf /system/xbin/busybox
  /sbin/ext/busybox mv /system/bin/busybox /system/xbin/busybox
fi;

# place wrapper script
/sbin/ext/busybox cp /sbin/ext/busybox-wrapper /sbin/busybox;

# Create property directory (Factory Reset?)
toolbox mkdir /data/prop
toolbox chmod 777 /data/prop

# Do not run Dark Tremor's a2sd on boot
#if [ -f /system/etc/init.d/99complete ]; then
#  toolbox rm /system/etc/init.d/99complete
#  toolbox rm /system/etc/init.d/*apps2sd*
#  toolbox rm /system/etc/init.d/*dtapp*
#fi

# mount system and rootfs r/o
/sbin/ext/busybox mount -t rootfs -o remount,ro rootfs;
/sbin/ext/busybox mount -o remount,ro /system;
