# Load modules if not compiled in kernel
if [ "$(cat /proc/filesystems | grep ext3)" = "" ] &&
  [ -f /system/lib/modules/jbd.ko ] && [ -f /system/lib/modules/ext3.ko ]; then
    toolbox insmod /system/lib/modules/jbd.ko
    toolbox insmod /system/lib/modules/ext3.ko
fi;

if [ "$(cat /proc/filesystems | grep ext4)" = "" ] &&
  [ -f /system/lib/modules/jbd2.ko ] && [ -f /system/lib/modules/ext4.ko ]; then
    toolbox insmod /system/lib/modules/jbd2.ko
    toolbox insmod /system/lib/modules/ext4.ko
fi;

# Mount microSD's second partition for app2sd, try ext4 first
if ! [ "$(cat /proc/partitions | grep mmcblk0p2)" = "" ]; then
    busybox  mount -t ext4 /dev/block/mmcblk1p2 /system/sd
    busybox  mount -t ext3 /dev/block/mmcblk1p2 /system/sd
    busybox  mount -t ext2 /dev/block/mmcblk1p2 /system/sd
fi;

# Make Dark Tremor's app2sd happy
if ! [ "$(busybox mount | grep /system/sd)" = "" ]; then
    setprop a2sd.mountpoint "/dev/block/mmcblk1p2"
fi;
