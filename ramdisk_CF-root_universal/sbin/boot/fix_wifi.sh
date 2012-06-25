TOOLBOX=/sbin/ext/busybox

KFILE="/lib/modules/2.6.35.7/kernel/drivers/scsi/scsi_wait_scan.ko"
SORIG=`strings $KFILE |grep vermagic=2.6`
SORIG=`echo $SORIG | cut -d'-' -f2 | cut -d' ' -f1`
SDIFF=`strings /system/etc/wifi/tiwlan_drv.ko |grep vermagic=2.6`
SDIFF=`echo $SDIFF | cut -d'-' -f2 | cut -d' ' -f1`

if [ $SDIFF != $SORIG ]; then
# Remount data & system RW
    $TOOLBOX mount -o remount,rw /system
    $TOOLBOX mount -o remount,rw /data

# Remove the cf-root installed flag
    $TOOLBOX rm -fr /system/cfroot/

# Backup original WiFi modules
if $TOOLBOX [ ! -f /data/cfroot/backup/tiwlan_drv.ko ]; then
    $TOOLBOX cat /system/etc/wifi/tiwlan_drv.ko > /data/cfroot/backup/tiwlan_drv.ko
fi;
if $TOOLBOX [ ! -f /data/cfroot/backup/tiap_drv.ko ]; then
    $TOOLBOX cat /system/etc/wifi/softap/tiap_drv.ko > /data/cfroot/backup/tiap_drv.ko
fi;

# Replace WiFi modules
    $TOOLBOX cat /res/misc/tiwlan_drv.ko > /system/etc/wifi/tiwlan_drv.ko
    $TOOLBOX cat /res/misc/tiap_drv.ko > /system/etc/wifi/softap/tiap_drv.ko
    $TOOLBOX chown 0.0 /system/etc/wifi/tiwlan_drv.ko
    $TOOLBOX chown 0.0 /system/etc/wifi/softap/tiap_drv.ko
    $TOOLBOX chmod 644 /system/etc/wifi/tiwlan_drv.ko
    $TOOLBOX chmod 644 /system/etc/wifi/softap/tiap_drv.ko

# Remount system RO
    $TOOLBOX mount -o remount,ro /system

# Do not remount data RO. The system expects this to be RW.
fi;
