if /sbin/ext/busybox [ ! -f /system/cfroot/release-100-I9003DDKP2- ]; 
then
# Remount system RW
    /sbin/ext/busybox mount -o remount,rw /system
    /sbin/ext/busybox mount -o remount,rw /data

# Backup files

    /sbin/ext/busybox mkdir -p /data/cf-root/backup
    toolbox cat /system/bin/app_process > /data/cf-root/backup/app_process
    toolbox cat /system/bin/system_server > /data/cf-root/backup/system_server
    toolbox cat /system/lib/libsqlite.so > /data/cf-root/backup/libsqlite.so
    toolbox cat /system/lib/libsqlite_jni.so > /data/cf-root/backup/libsqlite_jni.so
    toolbox cat /system/lib/egl/libGLES_android.so > /data/cf-root/backup/libGLES_android.so

# Applying Fugumod

    toolbox cat /res/misc/bin/app_process > /system/bin/app_process
    toolbox cat /res/misc/bin/system_server > /system/bin/system_server
    toolbox chown 0.0 /system/bin/app_process
    toolbox chown 0.0 /system/bin/system_server
    toolbox chmod 755 /system/bin/app_process
    toolbox chmod 755 /system/bin/system_server

# Helper scripts
    toolbox cat /res/misc/bin/sysro > /system/bin/sysro
    toolbox cat /res/misc/bin/sysrw > /system/bin/sysrw
    toolbox chown 0.0 /system/bin/sysro
    toolbox chown 0.0 /system/bin/sysrw
    toolbox chmod 755 /system/bin/sysro
    toolbox chmod 755 /system/bin/sysrw

# sqlite3
    toolbox rm /system/bin/sqlite3
    toolbox rm /system/xbin/sqlite3
    toolbox cat /res/misc/bin/sqlite3 > /system/xbin/sqlite3
    toolbox chown 0.0 /system/xbin/sqlite3
    toolbox chmod 6755 /system/xbin/sqlite3

# Zipalign
    toolbox rm /system/etc/init.d/align
    toolbox cat /res/misc/bin/zipalign > /system/bin/zipalign
    toolbox chown 0.0 /system/bin/zipalign
    toolbox chmod 755 /system/bin/zipalign

#Applying Patched sqlite lib
    toolbox cat /res/misc/lib/libsqlite.so > /system/lib/libsqlite.so
    toolbox cat /res/misc/lib/libsqlite_jni.so > /system/lib/libsqlite_jni.so
    toolbox chown 0.0 /system/lib/libsqlite.so
    toolbox chown 0.0 /system/lib/libsqlite_jni.so
    toolbox chmod 644 /system/lib/libsqlite.so
    toolbox chmod 644 /system/lib/libsqlite_jni.so

#Applying Graphics Performance Tweak
    
    rm /system/lib/egl/libGLES_android.so

# Free some space from /system as it fully packed in stock odexed roms
    if /sbin/ext/busybox [ -f /system/app/InfoAlarm.apk ]; 
    then
        toolbox rm /system/app/InfoAlarm.apk
    fi;
    if /sbin/ext/busybox [ -f /system/app/InfoAlarm.odex ]; 
    then
        toolbox rm /system/app/InfoAlarm.odex
    fi;

# Remove startup and shutdown sounds, to make some space
    #toolbox rm /system/etc/PowerOn.snd
    #toolbox rm /system/etc/PowerOn.wav

# ensure /system/xbin exists
    toolbox mkdir /system/xbin
    toolbox chmod 755 /system/xbin

# su
    toolbox rm /system/bin/su
    toolbox rm /system/xbin/su
    toolbox cat /res/misc/su > /system/bin/su
    toolbox chown 0.0 /system/bin/su
    toolbox chmod 6755 /system/bin/su
    /sbin/ext/busybox ln -s /system/bin/su /system/xbin/su

# Superuser
    toolbox rm /system/app/Superuser.apk
    toolbox rm /data/app/Superuser.apk
    toolbox cat /res/misc/Superuser.apk > /system/app/Superuser.apk
    toolbox chown 0.0 /system/app/Superuser.apk
    toolbox chmod 644 /system/app/Superuser.apk

# Tweaks script helper
    toolbox rm /data/property/persist.tweak.*

# Tweaks script
    toolbox mkdir /system/etc/init.d
    toolbox chmod 755 /system/etc/init.d
    toolbox rm /system/etc/init.d/00tweaks
    toolbox cat /res/misc/00tweaks > /system/etc/init.d/00tweaks
    toolbox chown 0.0 /system/etc/init.d/00tweaks
    toolbox chmod 755 /system/etc/init.d/00tweaks

# CWM & Tweak Manager
    toolbox rm /system/app/CWMManager.apk
    toolbox rm /data/dalvik-cache/*CWMManager.apk*
    toolbox rm /data/app/eu.chainfire.cfroot.cwmmanager*.apk

    toolbox rm /system/app/TweakManager.apk
    toolbox rm /data/dalvik-cache/*TweakManager.apk*
    toolbox rm /data/app/eu.chainfire.cfroot.tweakmanager*.apk

    toolbox cat /res/misc/CWMManager.apk > /system/app/CWMManager.apk
    toolbox chown 0.0 /system/app/CWMManager.apk
    toolbox chmod 644 /system/app/CWMManager.apk

    toolbox cat /res/misc/TweakManager.apk > /system/app/TweakManager.apk
    toolbox chown 0.0 /system/app/TweakManager.apk
    toolbox chmod 644 /system/app/TweakManager.apk

# Custom Bootanimation
    /sbin/ext/busybox cp /sbin/bootanimation /system/bin/bootanimation
    toolbox chmod 755 /system/bin/bootanimation
    toolbox rm /system/bin/samsungani
    /sbin/ext/busybox ln -s /system/bin/bootanimation /system/bin/samsungani

    if /sbin/ext/busybox [ ! -f /system/media/bootanimation.zip ]; then
       if /sbin/ext/busybox [ -f /system/media/ODEAnim.zip ]; then
            /sbin/ext/busybox rm /system/media/sanim.zip
            /sbin/ext/busybox mv /system/media/ODEAnim.zip /system/media/bootanimation.zip
        else
            /sbin/ext/busybox mv /system/media/sanim.zip /system/media/bootanimation.zip
        fi;
    else
        /sbin/ext/busybox rm /system/media/sanim.zip
    fi;
    /sbin/ext/busybox ln -s /system/media/bootanimation.zip /system/media/sanim.zip

# Once be enough
    toolbox mkdir /system/cfroot
    toolbox chmod 755 /system/cfroot
    toolbox rm /data/cfroot/*
    toolbox rmdir /data/cfroot
    toolbox rm /system/cfroot/*
    echo 1 > /system/cfroot/release-100-I9003DDKP2-

# Remount system RO
    /sbin/ext/busybox mount -o remount,ro /system

fi;
