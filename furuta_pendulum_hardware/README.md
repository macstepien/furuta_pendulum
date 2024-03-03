# Setup
```
pip install odrive==0.51
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/91-odrive.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
export PATH=$PATH:~/.local/bin
pip install ipython==8.10
```


# Motor configuration

```
odrv0.erase_configuration()
```

```
odrv0.config.brake_resistance=2.0
odrv0.axis0.controller.config.vel_limit = 40.0

odrv0.axis0.motor.config.current_lim = 12.0 
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.torque_constant = 8.27/80.0
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_GIMBAL
odrv0.axis0.motor.config.calibration_current = 1.0
odrv0.axis0.config.calibration_lockin.current = 1.0
odrv0.axis0.encoder.config.bandwidth = 1000.0
odrv0.axis1.encoder.config.bandwidth = 1000.0

odrv0.axis0.encoder.config.cpr = 8192

odrv0.axis0.encoder.config.use_index = True
```

```
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```

```
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_index_search = True
```


```
odrv0.save_configuration()
```

```
dump_errors(odrv0)
```

```
odrv0.axis0.controller.config.pos_gain = 35.0
odrv0.axis0.controller.config.vel_gain = 0.14083333313465118
odrv0.axis0.controller.config.vel_integrator_gain = 7.041666507720947
```

IMPORTANT - don't copy all commands at once, do it one after the other

```
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
```
```
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
```
```
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
```
```
odrv0.axis0.controller.input_pos = 0.0
```
```
odrv0.axis0.controller.start_anticogging_calibration()
```
```
import matplotlib
matplotlib.use('TkAgg')
start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
```

Wait until odrv0.axis0.controller.config.anticogging.calib_anticogging == False
```
odrv0.axis0.controller.config.anticogging.pre_calibrated = True
```
Additionally can be checked odrv0.axis0.controller.config.anticogging.anticogging_enabled

```
odrv0.save_configuration()
```

Configuration is finished.

## Usage
```
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
odrv0.axis0.controller.input_torque = 0.0
```

Resetting encoder count:
```
odrv0.axis1.encoder.set_linear_count(0)
```


## Saving and restoring configs

Saving config:
```
odrivetool backup-config ./odrive.json
```

Restoring config:
```
odrivetool restore-config odrive_config.json
```



# Notes
 * in gimbal mode current limit means voltage limit
 * motor used - max current around 3A.
 * potential error:

    ERROR: the following packages/stacks could not have their rosdep keys resolved
    to system dependencies:
    odrive_demo_description: Cannot locate rosdep definition for [ros2_control_demo_description]

    CMake Error at CMakeLists.txt:28 (message):
      Failed to find libusb-1.0
    ```
    sudo apt-get install libusb-1.0-0-dev
    ```

<!-- TODO -->
<!-- echo 0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer -->