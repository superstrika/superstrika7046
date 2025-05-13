# Activating a Neopixel node

### installing library "rpi_ws281x"
```bash
    pip3 install rpi_ws281x
```
### activating the Neopixel
```bash
    ros2 service call /neopixel_on_off std_srvs/srv/SetBool "{data: true}"
```
### publishing RGB values
```bash
    ros2 topic pub /neopixel_color std_msgs/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}" -1
```
### deactivating the Neopixel
```bash
    ros2 service call /neopixel_on_off std_srvs/srv/SetBool "{data: false}"
```

