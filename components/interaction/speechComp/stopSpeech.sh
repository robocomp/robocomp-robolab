#!/bin/sh

COMPNAME=speechComp
PID=`ps ax | grep python | grep $COMPNAME | grep Ice | awk '{ print $1 }'`

if test -z $PID; then
	echo "Cannot get $COMPNAME PID."
else
	kill -9 $PID
	echo "Killing $COMPNAME..."
fi
