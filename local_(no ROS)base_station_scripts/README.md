# Local Base-Station Controller Scripts

Laptop-side DualSense UDP scripts for sending drive and arm controller data to
the rover.

First-time setup on Ubuntu:

```bash
sudo apt install libhidapi-hidraw0 libhidapi-libusb0
python3 -m pip install --user -r requirements.txt
```

List controllers and use the serial for the drive or arm script:

```bash
python3 drive_control.py --list-controllers
python3 drive_control.py --controller-serial SERIAL_FROM_LIST
python3 ee_control.py --controller-serial SERIAL_FROM_LIST
```

If listing works but opening the controller fails with `Could not open
connection to device`, install a udev rule so your logged-in user can open
DualSense hidraw devices:

```bash
sudo tee /etc/udev/rules.d/70-dualsense.rules >/dev/null <<'EOF'
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ce6", TAG+="uaccess"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0df2", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and reconnect the controller after reloading the rules.
