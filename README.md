CAN driver for "8 devices" USB2CAN converter
================================================

Note
----
THIS SOFTWARE COMES WITHOUT ANY WARRANTY!


License
-------
GPL v2


How to build
------------
* Make sure you have kernel version >= 2.6.33 installed
* Install kernel header files and gcc
* Simple type "make"


Set up interface
----------------
    modprobe can_raw
    modprobe can_dev
    insmod usb_8dev.ko
    ip link set can0 up type can bitrate 1000000 sample-point 0.875


Shut down interface
-------------------
    ip link set can0 down
    rmmod usb_8dev
