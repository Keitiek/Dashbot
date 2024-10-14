Steps:

- connect the GPS USB
- check with `ls /dev/tty*` that `/dev/ttyACM0` has been created
- check with `sudo cat /dev/ttyACM0` that RTK receiver is receiving the stream
- create a udev rule with `sudo nano /etc/udev/rules.d/50-ardusimple.rules` so that RTK receiver is always accessible from a specific link, the rule to enter into the file is `KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK="tty_Ardusimple", GROUP="dialout", MODE="0666"`
- next restart RTK receiver (disconnect and connect again), and enter the following commands:
>
>`sudo service udev reload`
>
>`sudo service udev restart`
>
>`sudo udevadm trigger`
>
>`ls /dev/`

The successful end result is seeing `tty_Ardusimple` in the list.

[Source](https://www.ardusimple.com/how-to-use-ardusimple-rtk-receivers-and-get-gps-data-in-ros/)
