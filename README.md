# forcedimension-python

![license](https://img.shields.io/github/v/release/EmDash00/forcedimension-python?display_name=tag)

Written and Mantained Ember Chow. Looking for the documentation? You can find it here:

https://forcedimension-python-documentation.readthedocs.io/en/latest/

## About

### What is ForceDimensionSDK?

ForceDimensionSDK is a set of C/C++ functions and drivers created by [Force Dimension](https://www.forcedimension.com/company/about) for working with haptic devices, used by many companies and researchers around the world.

### Why Python?

Up until this point, the drivers have been exclusively in C/C++ with Java bindings; however, a lot of research is conducted using Python and its various scientific computing libraries (numpy, scipy, sympy, pandas, matplotlib, etc).

This was also the case for Prof. Roth's lab at IU: Bloomington. Our lab group is primarily focused on researching human-robot, many times through the use of haptic devices.

The ForceDimensionSDK is a very well written interface that solves the problem of making applications that use haptic devices.

## Installation

#### Note: Available for Windows and Linux only.

### PyPI Package (recommended)

```
pip install forcedimension
```

`numpy` is an optional dependency. If you want to install that as well:

```
pip install forcedimension\[numpy\]
```



#### Install the [ForceDimensionSDK](https://www.forcedimension.com/software/sdk) for your computer. 
By default, the bindings will search in the following system-wide install locations.

* System-wide install locations (Linux):
    - `/usr/local/lib`
    - `/usr/lib`
* System-wide install location (Windows):
    - `C:\Program Files\ForceDimension\sdk-X.X.X\lib`

##### Non-system-wide Installs

If you do not wish to make a system-wide installation set the FORCEDIM_SDK environment variable to the root folder of the ForceDimensionSDK installation (the `lib` folder should be one level under the root installation folder)

##### System-wide install for Linux

This requires extra steps since the Makefile for the ForceDimensionSDK does not offer a `make install` target for a system-wide install.

1. Copy all files from `lib/release/lin-*-gcc` to `/usr/local/lib`  

2. Copy all files from `include` to `/usr/local/include`  

3. MAKE SURE the libraries have `755` level access using `chmod`. If they don't applications cannot link or load them.

4. Make a symbolic link to libdhd and libdrd that drop the version so they end in `.so`, so that the file names are `libdhd.so` and `libdrd.so`

These steps can be automated by adding these targets to the Makefile.

```makefile
install:
	cp include/* /usr/local/include
	cp lib/release/lin-*-gcc/* /usr/local/lib
	chmod 755 /usr/local/lib/libdhd.so.X.X.X
	chmod 755 /usr/local/lib/libdrd.so.X.X.X
	chmod 755 /usr/local/lib/libdhd.a
	chmod 755 /usr/local/lib/libdrd.a
	ln -s /usr/local/lib/libdhd.so /usr/local/lib/libdhd.so.X.X.X
	ln -s /usr/local/lib/libdrd.so /usr/local/lib/libdrd.so.X.X.X
```

```makefile
uninstall:
	rm /usr/local/include/dhdc.h
	rm /usr/local/include/drdc.h
	rm /usr/local/lib/libdhd.a
	rm /usr/local/lib/libdhd.so.3.9.1
	rm /usr/local/lib/libdhd.so
	rm /usr/local/lib/libdrd.a
	rm /usr/local/lib/libdrd.so.3.9.1
	rm /usr/local/lib/libdrd.so
```
#### Additional Setup

##### Windows

Device Manager is a Windows tool that can be used to see connected devices, their statuses, driver information, and driver installation.

The easiest way to bring it up is to use the windows search bar on Windows 10 and search “Device Manager”. Regardless of the method used, you will need administrator level privileges to launch the Device Manager.

Find your haptics device and right-click on it and open `Properities`. Do `Update driver>Browse my computer for driver software` and specify the drivers listed under the `drivers\usb` in the root install directory. Try restarting if drivers are not detected or changes do not take place.  

Your device should now be listed under `USB Haptic Devices`

##### Linux

Add a udev rule under `/etc/udev/rules.d` for your device. Here's an explaination from the [Arch Linux Wiki](https://wiki.archlinux.org/index.php/Udev#Waking_from_suspend_with_USB_device) about what they are and here's a good udev file for the [Novint Falcon](https://github.com/libnifalcon/libnifalcon/blob/master/linux/40-novint-falcon-udev.rules).

A good way to find the USB bus of the device is by unplugging the device, doing `ls -l /dev/bus/usb/00*`, replugging, the device and then performing it again. 

MAKE SURE you unplug the USB and not just the power because the unpowering the device does not unpower the USB communications (which get power through the computer).

Then perform `lsusb` and note the ID of your device, which is in the format `idVendor:idPorduct`

