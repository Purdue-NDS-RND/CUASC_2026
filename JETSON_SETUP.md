# Jetson → Pixhawk MAVROS Connection Guide

This guide explains how to connect a Jetson companion computer to a Pixhawk flight controller using USB and MAVROS.

The connection allows the Jetson to send **GUIDED / offboard commands** to the drone through MAVROS.

---

# Hardware Connection

Connect the Jetson to the Pixhawk using a USB cable.

Jetson USB-A  ───── USB cable ───── Pixhawk USB-C

When connected, Linux will create a serial device such as:

/dev/ttyACM0

USB serial devices usually appear as `/dev/ttyACM*` or `/dev/ttyUSB*`.

---

# Identify the Pixhawk Serial Device

Before launching MAVROS, determine which serial device corresponds to the Pixhawk.

## 1. Check existing serial devices

Run:

```bash
ls /dev | grep ttyA
```

Example output:

```
ttyAMA0
```

---

## 2. Plug in the Pixhawk

Connect the Pixhawk USB cable to the Jetson.

---

## 3. Check again

Run the same command:

```bash
ls /dev | grep ttyA
```

Example output:

```
ttyACM0
ttyAMA0
```

---

## 4. Identify the new device

The **new device that appeared** is the Pixhawk.

Example:

| Device | Meaning |
|------|------|
| ttyACM0 | Pixhawk USB MAVLink interface |
| ttyAMA0 | Jetson internal UART |

Therefore the Pixhawk device is:

```
/dev/ttyACM0
```

---

## 5. Optional confirmation

Unplug the Pixhawk and run:

```bash
ls /dev | grep ttyA
```

You should see the device disappear.

Example:

```
ttyAMA0
```

This confirms that `/dev/ttyACM0` belongs to the Pixhawk.

---

# Test Temporary Permission Fix

If MAVROS fails with an error like:

```
DeviceError:serial:open: Permission denied
```

you can temporarily grant permission to the device to test the connection.

Run:

```bash
sudo chmod 666 /dev/ttyACM0
```

This gives temporary read/write access to the device.

Note: `/dev` devices are recreated by the system when unplugged or after reboot, so this change is not permanent.

Now try launching MAVROS.

---

# Launch MAVROS

Start MAVROS with the Pixhawk serial port.

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

---

# Verify Connection

Check the MAVROS state topic:

```bash
ros2 topic echo /mavros/state
```

Expected output:

```
connected: True
```

If this works after the temporary permission fix, then the issue was serial permissions.

---

# Permanent Permission Fix

Add your user to the serial device group:

```bash
sudo usermod -a -G dialout $USER
```

Most Linux systems use the `dialout` group to control access to serial ports like `/dev/ttyACM0`.

---

# Restart Session

Log out and log back in (or reboot):

```bash
sudo reboot
```

---

# Verify Permissions

After logging back in, run:

```bash
groups
```

Expected output should include:

```
dialout
```

---

# Final MAVROS Command

Once permissions are correct, launch MAVROS normally:

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

Your Jetson should now successfully communicate with the Pixhawk.

---

# SITL vs Real Hardware

| Environment | MAVROS Command |
|---|---|
| Simulation (SITL) | udp://:14550@ |
| Real Pixhawk | /dev/ttyACM0:115200 |

Example SITL command:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

Example real drone command:

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

---

# Summary

1. Plug Pixhawk into Jetson
2. Identify `/dev/ttyACM*` device
3. Test temporary permission fix
4. Launch MAVROS
5. Apply permanent `dialout` fix
6. Verify MAVROS connection

Your Jetson is now ready to send **autonomous commands** to the Pixhawk via MAVROS.
