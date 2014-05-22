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


DKMS integration
----------------
* Install the dkms package for your distro
* Check out the required branch from this repository
* Do a git archive --prefix=usb2can-0.1/ -o /usr/src/usb2can-0.1.tar HEAD
* Extract the archive in the /usr/src directory
* Add the module, e.g. with dkms add -m usb2can -v 0.1 --verbose
* Build the module, e.g. with dkms build -m usb2can -v 0.1 --verbose
* Install the module, e.g. with dkms install -m usb2can -v 0.1 --verbose
* You can also build packages for your distro, see man dkms


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
