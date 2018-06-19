#!/bin/sh

sync() {
	unison -batch -prefer newer -ignore "Path {mbed,.temp,BUILD}" ~/tmp/midi-usb ~/Dropbox/sketch/mbed/midi-usb
}

while true
do
	sync
	sleep 10
done
