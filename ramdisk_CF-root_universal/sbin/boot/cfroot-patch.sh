/sbin/ext/busybox mount -o remount,rw /data

TOOLBOX=/sbin/ext/busybox

# Backup files
/sbin/ext/busybox mkdir -p /data/cf-root/backup
$TOOLBOX cat /system/bin/app_process > /data/cf-root/backup/app_process
$TOOLBOX cat /system/bin/system_server > /data/cf-root/backup/system_server
$TOOLBOX cat /system/lib/libsqlite.so > /data/cf-root/backup/libsqlite.so
$TOOLBOX cat /system/lib/libsqlite_jni.so > /data/cf-root/backup/libsqlite_jni.so
#$TOOLBOX cat /system/lib/egl/libGLES_android.so > /data/cf-root/backup/libGLES_android.so

# Applying Fugumod
#$TOOLBOX cat /res/misc/bin/app_process > /system/bin/app_process
#$TOOLBOX cat /res/misc/bin/system_server > /system/bin/system_server
$TOOLBOX cp -f /res/misc/bin/app_process /system/bin/app_process
$TOOLBOX cp -f /res/misc/bin/system_server /system/bin/system_server
$TOOLBOX chown 0.2000 /system/bin/app_process
$TOOLBOX chown 0.2000 /system/bin/system_server
$TOOLBOX chmod 755 /system/bin/app_process
$TOOLBOX chmod 755 /system/bin/system_server

# Helper scripts
$TOOLBOX cat /res/misc/xbin/sysro > /system/xbin/sysro
$TOOLBOX cat /res/misc/xbin/sysrw > /system/xbin/sysrw
$TOOLBOX chown 0.0 /system/xbin/sysro
$TOOLBOX chown 0.0 /system/xbin/sysrw
$TOOLBOX chmod 755 /system/xbin/sysro
$TOOLBOX chmod 755 /system/xbin/sysrw

# sqlite3
$TOOLBOX rm /system/bin/sqlite3
$TOOLBOX rm /system/xbin/sqlite3
$TOOLBOX rm /system/lib/libncurses.so
$TOOLBOX cat /res/misc/xbin/sqlite3 > /system/xbin/sqlite3
$TOOLBOX cat /res/misc/lib/libncurses.so > /system/lib/libncurses.so
$TOOLBOX chown 0.0 /system/xbin/sqlite3
$TOOLBOX chown 0.0 /system/lib/libncurses.so
$TOOLBOX chmod 6755 /system/xbin/sqlite3
$TOOLBOX chmod 644 /system/lib/libncurses.so

#Applying Patched sqlite lib
$TOOLBOX cat /res/misc/lib/libsqlite.so > /system/lib/libsqlite.so
$TOOLBOX cat /res/misc/lib/libsqlite_jni.so > /system/lib/libsqlite_jni.so
$TOOLBOX chown 0.0 /system/lib/libsqlite.so
$TOOLBOX chown 0.0 /system/lib/libsqlite_jni.so
$TOOLBOX chmod 644 /system/lib/libsqlite.so
$TOOLBOX chmod 644 /system/lib/libsqlite_jni.so

# Zipalign
$TOOLBOX rm /system/etc/init.d/align
$TOOLBOX cat /res/misc/xbin/zipalign > /system/xbin/zipalign
$TOOLBOX chown 0.0 /system/xbin/zipalign
$TOOLBOX chmod 755 /system/xbin/zipalign

#Applying Graphics Performance Tweak
#rm /system/lib/egl/libGLES_android.so
echo "0 1 POWERVR_SGX530_125" > /system/lib/egl/egl.cfg

# Tweak Manager
$TOOLBOX rm /system/app/TweakManager.apk
$TOOLBOX rm /data/dalvik-cache/*TweakManager.apk*
$TOOLBOX rm /data/app/eu.chainfire.cfroot.tweakmanager*.apk

$TOOLBOX cat /res/misc/TweakManager.apk > /system/app/TweakManager.apk
$TOOLBOX chown 0.0 /system/app/TweakManager.apk
$TOOLBOX chmod 644 /system/app/TweakManager.apk

# Tweaks script helper
$TOOLBOX rm /data/property/persist.tweak.*

# Tweaks script
$TOOLBOX mkdir /system/etc/init.d
$TOOLBOX chmod 755 /system/etc/init.d
$TOOLBOX rm /system/etc/init.d/00tweaks
$TOOLBOX cat /res/misc/00tweaks > /system/etc/init.d/00tweaks
$TOOLBOX chown 0.0 /system/etc/init.d/00tweaks
$TOOLBOX chmod 755 /system/etc/init.d/00tweaks

# Bootanimation patch for Samsung ROMs
/sbin/ext/busybox sh /sbin/boot/bootanim-patch.sh
