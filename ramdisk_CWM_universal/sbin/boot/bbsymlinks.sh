cd /sbin
for line in $(/sbin/busybox --list)  
do
	if /sbin/ext/busybox [ ! -f "$line" ]; then
		/sbin/ext/busybox ln -s busybox "$line"
	fi
done
