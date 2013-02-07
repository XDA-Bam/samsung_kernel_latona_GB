TOOLBOX=/sbin/ext/busybox
CURRENT_VERSION=$($TOOLBOX cat /proc/version | $TOOLBOX cut -d"#" -f2)

if $TOOLBOX [ ! -f /system/CWM/release-BCK-I9003 ] || [ "$($TOOLBOX cat /system/CWM/release-BCK-I9003)" != "$CURRENT_VERSION" ];
then
# Remount RW
    $TOOLBOX mount -o remount,rw /system
    $TOOLBOX mount -o remount,rw /data
    $TOOLBOX mount -t rootfs -o remount,rw rootfs

# Free some space from /system as it fully packed in stock odexed roms
    if $TOOLBOX [ -f /system/app/InfoAlarm.apk ]; then
        $TOOLBOX mv /system/app/InfoAlarm.* /data/app/
        $TOOLBOX chown 0.0 /system/app/InfoAlarm.*
        $TOOLBOX chmod 644 /system/app/InfoAlarm.*
    fi;

# Bootanimation patch for Samsung ROMs
    $TOOLBOX sh /sbin/boot/bootanim-patch.sh

# ensure /system/xbin exists
    $TOOLBOX mkdir /system/xbin
    $TOOLBOX chmod 755 /system/xbin

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
    $TOOLBOX rm /data/app/eu.chainfire.CWM.cwmmanager*.apk

    $TOOLBOX cat /res/misc/CWMManager.apk > /system/app/CWMManager.apk
    $TOOLBOX chown 0.0 /system/app/CWMManager.apk
    $TOOLBOX chmod 644 /system/app/CWMManager.apk

# Once be enough
    $TOOLBOX mkdir /system/CWM
    $TOOLBOX chmod 755 /system/CWM
    $TOOLBOX rm /data/CWM/*
    $TOOLBOX rm /system/CWM/*
    $TOOLBOX echo $CURRENT_VERSION > /system/CWM/release-BCK-I9003

# Remount system RO
    $TOOLBOX mount -t rootfs -o remount,ro rootfs
    $TOOLBOX mount -o remount,ro /system
fi;
