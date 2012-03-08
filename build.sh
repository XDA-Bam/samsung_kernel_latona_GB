# Build kernel and modules
echo "Press enter to build the kernel"
read ANS
cd Kernel/
./LATONA.sh
cd ..
cp Kernel/arch/arm/boot/zImage zImage


# Copy to ramdisk
echo "Press enter to copy the modules"
read ANS

## Step 1: Clean ramdisk
cp --remove-destination Kernel/crypto/pcbc.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/crypto/pcbc.ko
###Storage
cp --remove-destination Kernel/drivers/bluetooth/btwilink.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/drivers/bluetooth/btwilink.ko
cp --remove-destination Kernel/drivers/misc/ti-st/st_drv.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/drivers/misc/ti-st/st_drv.ko
cp --remove-destination Kernel/drivers/scsi/scsi_wait_scan.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/drivers/scsi/scsi_wait_scan.ko
cp --remove-destination Kernel/samsung/battery/samsung_battery.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/battery/samsung_battery.ko
cp --remove-destination Kernel/samsung/bma222/yas_acc_kernel_driver.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/bma222/yas_acc_kernel_driver.ko
cp --remove-destination Kernel/samsung/fm_si4709/Si4709_driver.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/fm_si4709/Si4709_driver.ko
cp --remove-destination Kernel/samsung/gp2a/gp2a.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/gp2a/gp2a.ko
cp --remove-destination Kernel/samsung/gps/gps.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/gps/gps.ko
###J4FS
cp --remove-destination Kernel/samsung/orientation/yas_ori_kernel_driver.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/orientation/yas_ori_kernel_driver.ko
cp --remove-destination Kernel/samsung/param/param.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/param/param.ko
###RFS&FSR
cp --remove-destination Kernel/samsung/vibetonz/vibetonz.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/vibetonz/vibetonz.ko
cp --remove-destination Kernel/samsung/yas529/yas_mag_kernel_driver.ko ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/yas529/yas_mag_kernel_driver.ko

## Step 2: Universal CF-root ramdisk
cp --remove-destination Kernel/crypto/pcbc.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/crypto/pcbc.ko
###Storage
cp --remove-destination Kernel/drivers/bluetooth/btwilink.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/drivers/bluetooth/btwilink.ko
cp --remove-destination Kernel/drivers/misc/ti-st/st_drv.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/drivers/misc/ti-st/st_drv.ko
cp --remove-destination Kernel/drivers/scsi/scsi_wait_scan.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/drivers/scsi/scsi_wait_scan.ko
cp --remove-destination Kernel/samsung/battery/samsung_battery.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/battery/samsung_battery.ko
cp --remove-destination Kernel/samsung/bma222/yas_acc_kernel_driver.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/bma222/yas_acc_kernel_driver.ko
cp --remove-destination Kernel/samsung/fm_si4709/Si4709_driver.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/fm_si4709/Si4709_driver.ko
cp --remove-destination Kernel/samsung/gp2a/gp2a.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/gp2a/gp2a.ko
cp --remove-destination Kernel/samsung/gps/gps.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/gps/gps.ko
###J4FS
cp --remove-destination Kernel/samsung/orientation/yas_ori_kernel_driver.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/orientation/yas_ori_kernel_driver.ko
cp --remove-destination Kernel/samsung/param/param.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/param/param.ko
###RFS&FSR
cp --remove-destination Kernel/samsung/vibetonz/vibetonz.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/vibetonz/vibetonz.ko
cp --remove-destination Kernel/samsung/yas529/yas_mag_kernel_driver.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/samsung/yas529/yas_mag_kernel_driver.ko
cp --remove-destination Kernel/fs/mbcache.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/fs/mbcache.ko
cp --remove-destination Kernel/fs/jbd2/jbd2.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/fs/jbd2/jbd2.ko
cp --remove-destination Kernel/fs/ext4/ext4.ko ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/fs/ext4/ext4.ko


# Strip modules
echo "Press enter to strip the modules"
read ANS
cd ramdisk_clean/lib/modules/2.6.35.7/kernel/
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded crypto/pcbc.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded drivers/bluetooth/btwilink.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded drivers/misc/ti-st/st_drv.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded drivers/scsi/scsi_wait_scan.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/battery/samsung_battery.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/bma222/yas_acc_kernel_driver.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/fm_si4709/Si4709_driver.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/gp2a/gp2a.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/gps/gps.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/orientation/yas_ori_kernel_driver.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/param/param.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/vibetonz/vibetonz.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/yas529/yas_mag_kernel_driver.ko

cd ../../../../../
cd ramdisk_CF-root_universal/lib/modules/2.6.35.7/kernel/
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded crypto/pcbc.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded drivers/bluetooth/btwilink.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded drivers/misc/ti-st/st_drv.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded drivers/scsi/scsi_wait_scan.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/battery/samsung_battery.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/bma222/yas_acc_kernel_driver.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/fm_si4709/Si4709_driver.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/gp2a/gp2a.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/gps/gps.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/orientation/yas_ori_kernel_driver.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/param/param.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/vibetonz/vibetonz.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded samsung/yas529/yas_mag_kernel_driver.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded fs/mbcache.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded fs/jbd2/jbd2.ko
	/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded fs/ext4/ext4.ko

echo "All done. Press enter to exit script."
read ANS

