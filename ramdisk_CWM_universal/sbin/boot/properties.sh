/sbin/ext/busybox mount -t rootfs -o remount,rw rootfs 
mkdir -p /customkernel/property 
# echo true >> /customkernel/property/customkernel.cf-root 
# echo true >> /customkernel/property/customkernel.base.cf-root 
echo CWM-I9003 >> /customkernel/property/customkernel.name 
echo "CWM-I9003 with Bam Custom Kernel" >> /customkernel/property/customkernel.namedisplay 
echo true >> /customkernel/property/customkernel.cwm 
echo 5.0.2.8 >> /customkernel/property/customkernel.cwm.version 
/sbin/ext/busybox mount -t rootfs -o remount,ro rootfs 
