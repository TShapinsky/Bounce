#!/bin/bash
echo "" > cameras.dat
for file in /dev/video*; do
    v4l2-ctl -d $file -c exposure_auto=1
    v4l2-ctl -d $file -c exposure_absolute=50
    command="$(udevadm info -n "$file" | grep "SERIAL_SHORT=")"
    echo $command $file >> cameras.dat
done
