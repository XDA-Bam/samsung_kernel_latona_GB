#!/sbin/ext/busybox sh

# starting
echo "[ START ]" > /data/local/tmp/log_mods.log
date >> /data/local/tmp/log_mods.log

# execute tweaks
/system/bin/logwrapper /sbin/busybox run-parts /res/mods/scripts

# DONE
echo "[ DONE ]" >> /data/local/tmp/log_mods.log
