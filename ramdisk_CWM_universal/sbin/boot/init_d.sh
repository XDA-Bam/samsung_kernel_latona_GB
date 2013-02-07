if /sbin/ext/busybox [ -d /system/etc/init.d ]; then
  /system/bin/logwrapper /sbin/ext/busybox run-parts /system/etc/init.d
fi;

