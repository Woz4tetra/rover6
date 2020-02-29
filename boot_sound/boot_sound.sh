#!/usr/bin/env bash
amixer -c 1 sset PCM 20% > /dev/null 2>&1
omxplayer -o alsa:hw:1,0 /home/pi/.local/boot_sound/boot_sound.wav  > /dev/null 2>&1  &

