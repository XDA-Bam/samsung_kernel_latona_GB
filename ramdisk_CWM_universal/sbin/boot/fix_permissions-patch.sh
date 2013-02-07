#!/system/bin/sh
#
# Fix for fix_permissions

MOUNT="busybox mount"
MV="busybox mv"
GREP="busybox grep"
TEST="busybox test"

REMOUNT=0

if $TEST $( $GREP " / " "/proc/mounts" | $GREP "ro" | $GREP -cv "rw" ) -eq 1; then
   /sbin/ext/busybox mount -o remount rw /
   REMOUNT=1
fi

/sbin/ext/busybox mv /sbin/fix_permissions /sbin/fix_permissions_script
/sbin/ext/busybox mv /sbin/fix_permissions_wrapper /sbin/fix_permissions

if $TEST $REMOUNT -eq 1; then
   /sbin/ext/busybox mount -o remount ro /
fi