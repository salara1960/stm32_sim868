#!/bin/sh

mode="664"
group="wheel"
device="tmr"
drv=tmr_mod

case $1 in

	load)
	    modprobe $drv
	    sleep 1
	    chgrp $group /dev/${device}
	    chmod $mode /dev/${device}
            echo "Load driver "$drv".ko for device /dev/"$device" done."
	    exit 0
	;;

	unload)
	    modprobe -r $drv
	    echo "Unload driver "$drv".ko for device /dev/"$device" done."
	    exit 0
	;;

	*)
	    echo "Usage: ./drv.sh load|unload"
	    exit 0
	;;

esac

echo

