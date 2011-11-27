# Build kernel and modules
echo "Press enter to build the kernel"
read ANS
cd Kernel/
./LATONA.sh
cd ..


# Copy to ramdisk
echo "Press enter to copy the modules"
read ANS

## Step 1: Clean ramdisk
cp --remove-destination /Kernel/crypto/pcbc.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/crypto/pcbc.ko
###Storage
cp --remove-destination /Kernel/drivers/bluetooth/btwilink.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/drivers/bluetooth/btwilink.ko
cp --remove-destination /Kernel/drivers/misc/ti-st/st_drv.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/drivers/misc/ti-st/st_drv.ko
cp --remove-destination /Kernel/drivers/scsi/scsi_wait_scan.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/drivers/scsi/scsi_wait_scan.ko
cp --remove-destination /Kernel/samsung/battery/samsung_battery.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/battery/samsung_battery.ko
cp --remove-destination /Kernel/samsung/bma222/yas_acc_kernel_driver.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/bma222/yas_acc_kernel_driver.ko
cp --remove-destination /Kernel/samsung/fm_si4709/Si4709_driver.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/fm_si4709/Si4709_driver.ko
cp --remove-destination /Kernel/samsung/gp2a/gp2a.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/gp2a/gp2a.ko
cp --remove-destination /Kernel/samsung/gps/gps.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/gps/gps.ko
###J4FS
cp --remove-destination /Kernel/samsung/orientation/yas_ori_kernel_driver.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/orientation/yas_ori_kernel_driver.ko
cp --remove-destination /Kernel/samsung/param/param.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/param/param.ko
###RFS&FSR
cp --remove-destination /Kernel/samsung/vibetonz/vibetonz.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/vibetonz/vibetonz.ko
cp --remove-destination /Kernel/samsung/yas529/yas_mag_kernel_driver.ko /ramdisk_clean/lib/modules/2.6.35.7/kernel/samsung/yas529/yas_mag_kernel_driver.ko

## Step 2: CF-root ramdisk
cp --remove-destination /Kernel/crypto/pcbc.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/crypto/pcbc.ko
###Storage
cp --remove-destination /Kernel/drivers/bluetooth/btwilink.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/drivers/bluetooth/btwilink.ko
cp --remove-destination /Kernel/drivers/misc/ti-st/st_drv.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/drivers/misc/ti-st/st_drv.ko
cp --remove-destination /Kernel/drivers/scsi/scsi_wait_scan.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/drivers/scsi/scsi_wait_scan.ko
cp --remove-destination /Kernel/samsung/battery/samsung_battery.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/battery/samsung_battery.ko
cp --remove-destination /Kernel/samsung/bma222/yas_acc_kernel_driver.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/bma222/yas_acc_kernel_driver.ko
cp --remove-destination /Kernel/samsung/fm_si4709/Si4709_driver.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/fm_si4709/Si4709_driver.ko
cp --remove-destination /Kernel/samsung/gp2a/gp2a.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/gp2a/gp2a.ko
cp --remove-destination /Kernel/samsung/gps/gps.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/gps/gps.ko
###J4FS
cp --remove-destination /Kernel/samsung/orientation/yas_ori_kernel_driver.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/orientation/yas_ori_kernel_driver.ko
cp --remove-destination /Kernel/samsung/param/param.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/param/param.ko
###RFS&FSR
cp --remove-destination /Kernel/samsung/vibetonz/vibetonz.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/vibetonz/vibetonz.ko
cp --remove-destination /Kernel/samsung/yas529/yas_mag_kernel_driver.ko /ramdisk_CF-root/lib/modules/2.6.35.7/kernel/samsung/yas529/yas_mag_kernel_driver.ko


# Strip modules
echo "Press enter to strip the modules"
read ANS
cd ramdisk_clean/lib/modules/2.6.35.7/kernel/
	for i in $(find . | grep .ko | grep './')
	do
		echo $i
		/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded $i
	done
cd ../../../../../
cd ramdisk_CF-root/lib/modules/2.6.35.7/kernel/
	for i in $(find . | grep .ko | grep './')
	do
		echo $i
		/opt/arm-linux-eabi-4.6.2/bin/arm-eabi-strip --strip-unneeded $i
	done

echo "All done. Press enter to exit script."
read ANS

