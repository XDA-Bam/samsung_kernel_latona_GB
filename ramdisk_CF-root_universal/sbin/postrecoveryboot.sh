#!/sbin/ext/busybox sh

rm /sdcard
mkdir /sdcard
busybox mount -t vfat -o noatime,nodiratime /dev/block/mmcblk1p1 /sdcard >> /dev/null 2>&1
busybox mount -t ext4 -o noatime,nodiratime,noauto_da_alloc,barrier=1 /dev/block/mmcblk1p1 /sdcard >> /dev/null 2>&1

#rmdir /sdcard/external_sd
#mkdir /sdcard/external_sd
#busybox mount -t vfat /dev/block/mmcblk0p1 /sdcard/external_sd
busybox mount -t vfat /dev/block/mmcblk0p1 /emmc >> /dev/null 2>&1

if [ ! -d /sdcard/.android_secure ]; then
	mkdir /sdcard/.android_secure
fi

if [ -d /emmc/.android_secure ]; then
#	ASEC=`find /emmc/.android_secure | wc -l`
#	if [ "$ASEC" -gt 1 ]; then
		busybox mount --bind /emmc/.android_secure /sdcard/.android_secure
#	fi
fi

umount /dbdata
