if /sbin/ext/busybox [ -d /system/etc/init.d ]; then
  /sbin/ext/busybox run-parts /system/etc/init.d
fi;

if /sbin/ext/busybox [ -f /system/bin/customboot.sh ]; then
  /sbin/ext/busybox sh /system/bin/customboot.sh;
fi;

if /sbin/ext/busybox [ -f /system/xbin/customboot.sh ]; then
  /sbin/ext/busybox sh /system/xbin/customboot.sh
fi;

if /sbin/ext/busybox [ -f /data/local/customboot.sh ]; then
  /sbin/ext/busybox sh /data/local/customboot.sh
fi;
