
odrive

pip install odrive==0.51
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/91-odrive.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
export PATH=$PATH:~/.local/bin

pip install ipython==8.10


-------
odrv0.config.brake_resistance=2.0
odrv0.config.dc_bus_undervoltage_trip_level = 8.0
odrv0.config.dc_max_positive_current = 2.0
odrv0.config.dc_max_negative_current = -1.0
odrv0.config.max_regen_current = 0
odrv0.save_configuration()

odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.calibration_current = 5
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_GIMBAL
odrv0.axis0.motor.config.current_lim = 12
odrv0.axis0.motor.config.requested_current_range = 12
odrv0.save_configuration()

odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr = 8192
odrv0.axis0.config.calibration_lockin.current = 5
odrv0.axis0.config.calibration_lockin.ramp_time = 0.4
odrv0.axis0.config.calibration_lockin.ramp_distance = 3.14159265
odrv0.axis0.config.calibration_lockin.accel = 5
odrv0.axis0.config.calibration_lockin.vel = 10
odrv0.save_configuration()

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.vel_limit = 20
odrv0.axis0.controller.config.pos_gain = 30
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.2
odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
odrv0.axis0.trap_traj.config.vel_limit = 20
odrv0.axis0.trap_traj.config.accel_limit = 5
odrv0.axis0.trap_traj.config.decel_limit = 5
odrv0.save_configuration()
odrv0.reboot()
------


odrv0.config.brake_resistance=2.0
odrv0.axis0.controller.config.vel_limit = 2.0

<!-- in gimball mode current limit means voltage limit -->
odrv0.axis0.motor.config.current_lim = 12.0 
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.torque_constant = 8.27/80.0
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_GIMBAL
<!-- in gimball mode calibration current means calibration voltage -->
odrv0.axis0.motor.config.calibration_current = 1.0
odrv0.axis0.config.calibration_lockin.current = 1.0

odrv0.axis0.encoder.config.cpr = 8192
odrv0.save_configuration()

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

<!-- Save calibration -->
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_index_search = True

odrv0.save_configuration()

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.start_anticogging_calibration()
# Wait until odrv0.axis0.controller.config.anticogging.calib_anticogging == False
odrv0.axis0.controller.config.anticogging.pre_calibrated = True

odrv0.save_configuration()




odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
odrv0.axis0.controller.input_torque = 0.0


odrv0.axis0.error = 0



In [273]:  odrv0.axis0.controller.config.pos_gain
Out[273]: 20.0

In [274]:  odrv0.axis0.controller.config.pos_gain = 35

In [275]:  odrv0.axis0.controller.config.vel_gain
Out[275]: 0.1666666716337204

In [276]:  odrv0.axis0.controller.config.vel_gain = 0.14083333313465118

In [277]: odrv0.axis0.controller.config.vel_integrator_gain
Out[277]: 0.3333333432674408

In [278]: odrv0.axis0.controller.config.vel_integrator_gain = 7.041666507720947



 start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])


ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
odrive_demo_description: Cannot locate rosdep definition for [ros2_control_demo_description]

CMake Error at CMakeLists.txt:28 (message):
  Failed to find libusb-1.0

   sudo apt-get install libusb-1.0-0-dev


<!-- Resetting encoder counter -->
odrv0.axis1.encoder.set_linear_count(0)

odrv0.axis0.controller.config.vel_limit = 8.0
odrv0.axis0.motor.config.current_lim = 14

## Motors
Max current around 3A.

<!-- TODO -->
<!-- echo 0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer -->


## Running


TODO!!!!:
cant have watchdog on start because:
axis0
  axis: Error(s):
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
  motor: no error
  fet_thermistor: no error
  motor_thermistor: no error
  encoder: no error
  controller: no error
axis1
  axis: Error(s):
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
  motor: no error
  fet_thermistor: no error
  motor_thermistor: no error
  encoder: no error
  controller: no error

odrv0.axis0.config.enable_watchdog = False
odrv0.axis1.config.enable_watchdog = False
odrv0.save_configuration()

# Configuring

1.
```
odrv0.erase_configuration()
```

2.
```
odrv0.config.brake_resistance=2.0
odrv0.axis0.controller.config.vel_limit = 2.0

odrv0.axis0.motor.config.current_lim = 12.0 
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.torque_constant = 8.27/80.0
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_GIMBAL
odrv0.axis0.motor.config.calibration_current = 1.0
odrv0.axis0.config.calibration_lockin.current = 1.0

odrv0.axis0.encoder.config.cpr = 8192

odrv0.axis0.encoder.config.use_index = True

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_index_search = True

odrv0.save_configuration()


!!! check errors
dump_errors(odrv0)

5.

odrv0.axis0.controller.config.pos_gain = 35.0
odrv0.axis0.controller.config.vel_gain = 0.14083333313465118
odrv0.axis0.controller.config.vel_integrator_gain = 7.041666507720947

<!-- IMPORTANT - don't copy all commands at once, do it one after the other -->
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.start_anticogging_calibration()

start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])

# Wait until odrv0.axis0.controller.config.anticogging.calib_anticogging == False
odrv0.axis0.controller.config.anticogging.pre_calibrated = True
<!-- Additionally can be checked odrv0.axis0.controller.config.anticogging.anticogging_enabled -->

odrv0.save_configuration()


7.
odrv0.axis0.controller.config.vel_limit = 8.0
odrv0.axis0.motor.config.current_lim = 14
odrv0.save_configuration()


6.
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
odrv0.axis0.controller.input_torque = 0.0



Usage:
odrv0.axis1.encoder.set_linear_count(0)


odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH