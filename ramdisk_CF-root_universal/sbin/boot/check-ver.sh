KFILE="/lib/modules/2.6.35.7/kernel/drivers/scsi/scsi_wait_scan.ko"
SORIG=`strings $KFILE |grep vermagic=2.6`
SORIG=`expr substr "$SORIG" 19 8`
SDIFF=`strings /system/etc/wifi/tiwlan_drv.ko |grep vermagic=2.6`
SDIFF=`expr substr "$SDIFF" 19 8`

if [ $SDIFF != $SORIG ]
then
# Remount system RW
    /sbin/ext/busybox mount -o remount,rw /system

# Remove the cf-root installed flag
    /sbin/ext/busybox rm -fr /system/cfroot/

# Patch module version for wifi drivers
    /sbin/ext/busybox sh /sbin/boot/patch-ver.sh $KFILE /system/etc/wifi/tiwlan_drv.ko
    /sbin/ext/busybox sh /sbin/boot/patch-ver.sh $KFILE /system/etc/wifi/softap/tiap_drv.ko
fi
