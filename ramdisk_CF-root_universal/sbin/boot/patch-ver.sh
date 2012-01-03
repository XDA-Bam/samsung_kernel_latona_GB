if	[ "" = "$1" ]
then
	echo --------------------------------------------------------------------------------
	echo - Usage
	echo -   : ./patch-ver.sh [file1.ko] [file2.ko]
	echo --------------------------------------------------------------------------------
	exit
fi

SORIG=`strings $1 |grep vermagic=2.6`
SORIG=`expr substr "$SORIG" 19 8`
SDIFF=`strings $2 |grep vermagic=2.6`
SDIFF=`expr substr "$SDIFF" 19 8`

if [ $SDIFF != $SORIG ]
then
	sed -i s/$SDIFF/$SORIG/g $2
	echo "Done"
fi
