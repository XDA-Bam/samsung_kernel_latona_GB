TOOLBOX=/sbin/ext/busybox
CURRENT_VERSION=$($TOOLBOX cat /proc/version | $TOOLBOX cut -d"#" -f2)

if $TOOLBOX [ ! -f /system/cfroot/release-BCK-I9003 ] || [ "$($TOOLBOX cat /system/cfroot/release-BCK-I9003)" != "$CURRENT_VERSION" ];
then
# Remount system RW
    $TOOLBOX mount -o remount,rw /system
    $TOOLBOX mount -t rootfs -o remount,rw rootfs

# Free some space from /system as it fully packed in stock odexed roms
    if $TOOLBOX [ -f /system/app/InfoAlarm.apk ]; then
        $TOOLBOX mv /system/app/InfoAlarm.* /data/app/
        $TOOLBOX chown 0.0 /system/app/InfoAlarm.*
        $TOOLBOX chmod 644 /system/app/InfoAlarm.*
    fi;

# ensure /system/xbin exists
    $TOOLBOX mkdir /system/xbin
    $TOOLBOX chmod 755 /system/xbin

# Custom patches for SGSL
    $TOOLBOX sh /sbin/boot/cfroot-patch.sh

# su
    $TOOLBOX rm /system/bin/su
    $TOOLBOX rm /system/xbin/su
    $TOOLBOX cat /res/misc/su > /system/xbin/su
    $TOOLBOX chown 0.0 /system/xbin/su
    $TOOLBOX chmod 6755 /system/xbin/su

# Superuser
    $TOOLBOX rm /system/app/Superuser.apk
    $TOOLBOX rm /data/app/Superuser.apk
    $TOOLBOX cat /res/misc/Superuser.apk > /system/app/Superuser.apk
    $TOOLBOX chown 0.0 /system/app/Superuser.apk
    $TOOLBOX chmod 644 /system/app/Superuser.apk

# CWM Manager
    $TOOLBOX rm /system/app/CWMManager.apk
    $TOOLBOX rm /data/dalvik-cache/*CWMManager.apk*
    $TOOLBOX rm /data/app/eu.chainfire.cfroot.cwmmanager*.apk

    $TOOLBOX cat /res/misc/CWMManager.apk > /system/app/CWMManager.apk
    $TOOLBOX chown 0.0 /system/app/CWMManager.apk
    $TOOLBOX chmod 644 /system/app/CWMManager.apk

# Once be enough
    $TOOLBOX mkdir /system/cfroot
    $TOOLBOX chmod 755 /system/cfroot
    $TOOLBOX rm /data/cfroot/*
    $TOOLBOX rm /system/cfroot/*
    $TOOLBOX echo $CURRENT_VERSION > /system/cfroot/release-BCK-I9003

# Remount system RO
    $TOOLBOX mount -t rootfs -o remount,ro rootfs
    $TOOLBOX mount -o remount,ro /system
fi;
