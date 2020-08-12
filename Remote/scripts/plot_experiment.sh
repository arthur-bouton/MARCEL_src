#!/bin/bash
# The program tracer can be found here: https://github.com/Bouty92/Tracer

if [ -n "$2" ] && [ "$2" != 0 ]; then
	offset="-o $2"
fi

if [ -n "$3" ] && [ "$3" != 0 ]; then
	band="-b $3"
fi

tracer -q -n20 -f $1 $offset $band -C-5/-4:x2/-2:x2 -L'Flip coefficient,Steering node,Boggie node,Steering rate,Boggie torque' &
tracer -q -n20 -f $1 $offset $band -C2/4,5/3,6 -L'Heading,Alpha,Beta,Steering angle,Boggie angle' &
tracer -q -n20 -f $1 $offset $band -C7:9/10:12/13:15 -L'fx,fy,fz,tx,ty,tz,tx,ty,tz' -T'Front,Front,Back' &

read

kill -- -$$
