#!/bin/bash

if [ $# -lt 1 ]; then
	echo Préciser le programme à charger >&2
	exit 1
fi

src=$1

if test -d $src; then
	i=$(( ${#src} - 1 ))
	if [ "${src:$i}" == '/' ]; then
		src=${src:0:$i}
	fi
	src=$src/${src##*/}.ino
fi

arduino --preferences-file ${0%/*}/preferences_teensy.txt --upload $src

killall teensy
