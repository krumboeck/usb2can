CAN driver for UAB "8 devices" USB2CAN converter
================================================

Note
----
THIS SOFTWARE COMES WITHOUT ANY WARRANTY!


License
-------
GPL v2


How to build
------------
* Install kernel header files and gcc
* Simple type "make"


Set up interface
----------------
    modprobe can_raw
    modprobe can_dev
    insmod usb2can.ko
    ip link set can0 up type can bitrate 125000


Shut down interface
-------------------
    ip link set can0 down
    rmmod usb2can
