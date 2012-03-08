if /sbin/ext/busybox [ ! -f /system/cfroot/release-100-I9003XXKPQ- ]; 
then
# Remount system RW
    /sbin/ext/busybox mount -o remount,rw /system
    /sbin/ext/busybox mount -t rootfs -o remount,rw rootfs

# Free some space from /system as it fully packed in stock odexed roms
    if /sbin/ext/busybox [ -f /system/app/InfoAlarm.apk ]; then
        /sbin/ext/busybox cp /system/app/InfoAlarm.* /tmp/
        toolbox rm /system/app/InfoAlarm.*
    fi;

# ensure /system/xbin exists
    toolbox mkdir /system/xbin
    toolbox chmod 755 /system/xbin

# Custom patches for SGSL
    /sbin/ext/busybox sh /sbin/boot/cfroot-patch.sh

# su
    toolbox rm /system/bin/su
    toolbox rm /system/xbin/su
    toolbox cat /res/misc/su > /system/xbin/su
    toolbox chown 0.0 /system/xbin/su
    toolbox chmod 6755 /system/xbin/su

# Superuser
    toolbox rm /system/app/Superuser.apk
    toolbox rm /data/app/Superuser.apk
    toolbox cat /res/misc/Superuser.apk > /system/app/Superuser.apk
    toolbox chown 0.0 /system/app/Superuser.apk
    toolbox chmod 644 /system/app/Superuser.apk

# CWM Manager
    toolbox rm /system/app/CWMManager.apk
    toolbox rm /data/dalvik-cache/*CWMManager.apk*
    toolbox rm /data/app/eu.chainfire.cfroot.cwmmanager*.apk

    toolbox cat /res/misc/CWMManager.apk > /system/app/CWMManager.apk
    toolbox chown 0.0 /system/app/CWMManager.apk
    toolbox chmod 644 /system/app/CWMManager.apk

# Restore Apps if possible
    /sbin/ext/busybox cp /tmp/InfoAlarm.* /system/app/
    toolbox chown 0.0 /system/app/InfoAlarm.*
    toolbox chmod 644 /system/app/InfoAlarm.*
    toolbox rm /tmp/InfoAlarm.*

# Once be enough
    toolbox mkdir /system/cfroot
    toolbox chmod 755 /system/cfroot
    toolbox rm /data/cfroot/*
    toolbox rmdir /data/cfroot
    toolbox rm /system/cfroot/*
    echo 1 > /system/cfroot/release-100-I9003XXKPQ-

# Remount system RO
    /sbin/ext/busybox mount -t rootfs -o remount,ro rootfs
    /sbin/ext/busybox mount -o remount,ro /system
fi;
