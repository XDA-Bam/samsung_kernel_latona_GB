# Custom Bootanimation
/sbin/ext/busybox cp /sbin/bootanimation /system/bin/bootanimation
/sbin/ext/busybox chmod 755 /system/bin/bootanimation
/sbin/ext/busybox cp /sbin/bootanimation.sh /system/bin/bootanimation.sh
/sbin/ext/busybox chmod 755 /system/bin/bootanimation.sh
/sbin/ext/busybox rm /system/bin/samsungani
/sbin/ext/busybox ln -s /system/bin/bootanimation /system/bin/samsungani

if /sbin/ext/busybox [ ! -f /system/media/bootanimation.zip ]; then
   if /sbin/ext/busybox [ -f /system/media/ODEAnim.zip ]; then
		/sbin/ext/busybox rm /system/media/sanim.zip
		/sbin/ext/busybox mv /system/media/ODEAnim.zip /system/media/bootanimation.zip
	else
		/sbin/ext/busybox mv /system/media/sanim.zip /system/media/bootanimation.zip
	fi;
else
	/sbin/ext/busybox rm /system/media/sanim.zip
fi;
/sbin/ext/busybox ln -s /system/media/bootanimation.zip /system/media/sanim.zip
