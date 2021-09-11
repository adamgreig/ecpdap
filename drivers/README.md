# ECPDAP Platform-Specific Drivers

## Linux

Copy the `99-cmsis-dap.rules` file in this directory to your udev configuration
directory and restart udev:

```
sudo cp 99-cmsis-dap.rules /etc/udev/rules.d
sudo udevadm control --reload
```

This rules file permits user access to all CMSIS-DAP based probes; if you
already have OpenOCD installed system-wide this rule may already be in place,
or you can install custom rules for only a specific probe.

## Windows

For use with CMSIS-DAP v1 probes, no additional driver installation is
necessary, as the default HID drivers should work.

Most CMSIS-DAPv2 interfaces should use WinUSB drivers and not require any
additional installation, but this is not always set up correctly by each probe.
If your probe does not work in v2 mode, first check it does in fact support v2
(many do not) and check for any vendor-supplied drivers. If these do not work,
try using [Zadig](https://zadig.akeo.ie/) to set up a driver.  Download and run
the Zadig executable, select your probe's CMSIS-DAPv2 interface in the
dropdown, and click `Install Driver`, and click the `Replace Driver` button.
This will install a default WinUSB driver for the probe's VID:PID combination,
allowing ECPDAP to use the device.

## MacOS

Install libusb-compat via Homebrew:
```
brew install libusb-compat 
```
Confirmed working under MacOS 11.4 Big Sur
