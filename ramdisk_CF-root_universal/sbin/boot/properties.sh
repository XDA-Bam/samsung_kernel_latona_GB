/sbin/ext/busybox mount -t rootfs -o remount,rw rootfs 
mkdir -p /customkernel/property 
echo true >> /customkernel/property/customkernel.cf-root 
echo true >> /customkernel/property/customkernel.base.cf-root 
echo CF-Root-I9003 >> /customkernel/property/customkernel.name 
echo "CF-Root-I9003 by amit.bagaria with Bam Custom Kernel" >> /customkernel/property/customkernel.namedisplay 
#echo 102 >> /customkernel/property/customkernel.version.number 
#echo 5.1 >> /customkernel/property/customkernel.version.name 
#echo true >> /customkernel/property/customkernel.bootani.zip 
#echo true >> /customkernel/property/customkernel.bootani.bin 
echo true >> /customkernel/property/customkernel.cwm 
echo 5.0.2.8 >> /customkernel/property/customkernel.cwm.version 
/sbin/ext/busybox mount -t rootfs -o remount,ro rootfs 
