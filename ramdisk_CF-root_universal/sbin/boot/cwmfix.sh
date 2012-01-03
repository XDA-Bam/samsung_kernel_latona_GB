#!/system/bin/sh
#
# Fix for CWM (for sdswap mod) by amit.bagaria
# Version 1.0

busybox mount -o remount rw /
SDVAL=`grep external_sd /system/etc/vold.fstab | cut -d " " -f5 | cut -d "/" -f6` ;
if [ "$SDVAL" == "mmc0" ]; 
then
  ln -s /mnt/sdcard /sd
else
  ln -s /mnt/sdcard/external_sd /sd
fi
busybox mount -o remount ro /
